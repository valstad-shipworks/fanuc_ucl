//! Kernel transmit-error reporting for the Stream Motion socket.
//!
//! Linux converts a transmit-path `ENOBUFS` into a successful `send`:
//! `udp_send_skb` clears the error and only bumps `UDP_MIB_SNDBUFERRORS`
//! unless `IP_RECVERR` is set on the socket. ICMP destination-unreachable
//! from the controller is discarded on the same terms. With `IP_RECVERR` both
//! land on the socket error queue, `send` reports them, and the detail is
//! readable through `MSG_ERRQUEUE`.

#[cfg(target_os = "linux")]
use std::os::fd::AsRawFd;

#[cfg(target_os = "linux")]
const CMSG_CAP: usize = 256;

/// Bounds one drain so a socket producing errors faster than we read them
/// cannot stall the interpolation cycle.
#[cfg(target_os = "linux")]
const MAX_DRAIN: usize = 32;

/// A transmit error the kernel queued against the socket.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct TxError {
    pub errno: i32,
    pub origin: u8,
}

impl TxError {
    /// Where the error came from, for logging.
    pub fn origin_str(&self) -> &'static str {
        #[cfg(target_os = "linux")]
        match self.origin {
            libc::SO_EE_ORIGIN_LOCAL => return "local",
            libc::SO_EE_ORIGIN_ICMP => return "icmp",
            libc::SO_EE_ORIGIN_ICMP6 => return "icmp6",
            _ => {}
        }
        "unknown"
    }
}

impl std::fmt::Display for TxError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{} ({})",
            std::io::Error::from_raw_os_error(self.errno),
            self.origin_str()
        )
    }
}

/// Turns on transmit-error reporting, returning the descriptor to drain later.
///
/// `None` where the kernel cannot answer: non-Linux targets, or snare's shim
/// socket, whose `as_raw_fd` is `-1` because no kernel socket backs it. The
/// returned descriptor stays owned by `socket` and is only valid for its
/// lifetime.
#[cfg(target_os = "linux")]
pub(crate) fn enable_tx_error_reporting(socket: &snare::net::UdpSocket) -> Option<i32> {
    let fd = socket.as_raw_fd();
    if fd < 0 {
        return None;
    }
    let on: libc::c_int = 1;
    let rc = unsafe {
        libc::setsockopt(
            fd,
            libc::IPPROTO_IP,
            libc::IP_RECVERR,
            &on as *const _ as *const libc::c_void,
            size_of::<libc::c_int>() as libc::socklen_t,
        )
    };
    if rc == -1 {
        tracing::warn!(
            error = %std::io::Error::last_os_error(),
            "Could not enable IP_RECVERR; transmit drops will be invisible"
        );
        return None;
    }
    Some(fd)
}

#[cfg(not(target_os = "linux"))]
pub(crate) fn enable_tx_error_reporting(_socket: &snare::net::UdpSocket) -> Option<i32> {
    None
}

/// Reads every queued transmit error into `out`, clearing the socket's error
/// queue. Must be called on each readable wakeup once reporting is on: the
/// error queue keeps the descriptor `POLLERR` until it is emptied.
#[cfg(target_os = "linux")]
pub(crate) fn drain_error_queue(fd: i32, out: &mut Vec<TxError>) {
    // The union gives the cmsghdr alignment CMSG_FIRSTHDR expects; CMSG_CAP
    // holds a sock_extended_err plus the offender address.
    #[repr(C)]
    union CmsgBuf {
        _align: libc::cmsghdr,
        buf: [u8; CMSG_CAP],
    }

    // The error queue carries a copy of the offending datagram; we only want
    // the control message, so the payload goes in the bin.
    let mut discard = [0u8; 64];

    for _ in 0..MAX_DRAIN {
        let mut cmsg = CmsgBuf { buf: [0; CMSG_CAP] };
        let mut iov = libc::iovec {
            iov_base: discard.as_mut_ptr().cast(),
            iov_len: discard.len(),
        };
        let mut msg: libc::msghdr = unsafe { std::mem::zeroed() };
        msg.msg_iov = &mut iov;
        msg.msg_iovlen = 1;
        msg.msg_control = unsafe { cmsg.buf.as_mut_ptr() }.cast();
        msg.msg_controllen = CMSG_CAP as _;

        let n = unsafe { libc::recvmsg(fd, &mut msg, libc::MSG_ERRQUEUE | libc::MSG_DONTWAIT) };
        if n < 0 {
            return;
        }
        if let Some(err) = parse_tx_error(&msg) {
            out.push(err);
        }
    }
}

#[cfg(not(target_os = "linux"))]
pub(crate) fn drain_error_queue(_fd: i32, _out: &mut Vec<TxError>) {}

#[cfg(target_os = "linux")]
fn parse_tx_error(msg: &libc::msghdr) -> Option<TxError> {
    let mut c = unsafe { libc::CMSG_FIRSTHDR(msg) };
    while !c.is_null() {
        let (level, typ) = unsafe { ((*c).cmsg_level, (*c).cmsg_type) };
        if level == libc::IPPROTO_IP && typ == libc::IP_RECVERR {
            let ee: libc::sock_extended_err = unsafe {
                std::ptr::read_unaligned(libc::CMSG_DATA(c) as *const libc::sock_extended_err)
            };
            return Some(TxError {
                errno: ee.ee_errno as i32,
                origin: ee.ee_origin,
            });
        }
        c = unsafe { libc::CMSG_NXTHDR(msg, c) };
    }
    None
}

#[cfg(all(test, target_os = "linux"))]
mod test {
    use super::*;
    use std::net::UdpSocket;

    #[test]
    fn test_icmp_unreachable_surfaces() {
        let sock = UdpSocket::bind("127.0.0.1:0").unwrap();
        let dead = UdpSocket::bind("127.0.0.1:0").unwrap();
        let dead_addr = dead.local_addr().unwrap();
        drop(dead);
        sock.connect(dead_addr).unwrap();
        assert!(enable_tx_error_reporting_raw(&sock).is_some());

        // Loopback returns the ICMP port-unreachable to the sender, but only
        // after the first datagram has been processed, so the error lands on
        // a later syscall.
        let mut found = Vec::new();
        for _ in 0..50 {
            let _ = sock.send(b"ping");
            drain_error_queue(std::os::fd::AsRawFd::as_raw_fd(&sock), &mut found);
            if !found.is_empty() {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(10));
        }

        let err = found.first().expect("no transmit error reported");
        assert_eq!(err.errno, libc::ECONNREFUSED);
        assert_eq!(err.origin, libc::SO_EE_ORIGIN_ICMP);
    }

    fn enable_tx_error_reporting_raw(socket: &UdpSocket) -> Option<i32> {
        let fd = std::os::fd::AsRawFd::as_raw_fd(socket);
        let on: libc::c_int = 1;
        let rc = unsafe {
            libc::setsockopt(
                fd,
                libc::IPPROTO_IP,
                libc::IP_RECVERR,
                &on as *const _ as *const libc::c_void,
                size_of::<libc::c_int>() as libc::socklen_t,
            )
        };
        (rc != -1).then_some(fd)
    }
}
