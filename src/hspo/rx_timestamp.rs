use std::{io, net::SocketAddr, time::SystemTime};

#[cfg(unix)]
use std::{os::fd::AsRawFd, time::Duration};

#[cfg(unix)]
const CMSG_CAP: usize = 256;

/// Requests kernel receive timestamps on the socket.
///
/// Linux uses `SO_TIMESTAMPING` (rx software, CLOCK_REALTIME); other unix uses
/// `SO_TIMESTAMP`. Fails with `Unsupported` on non-unix targets and with
/// `EBADF` against snare's shim socket — callers fall back to `recv_from` and
/// stamping at user-space receive.
///
/// Even after success individual datagrams may carry no timestamp: Linux
/// turns generation on through a deferred static key, so the first packets
/// can arrive unstamped. Handle `None` per packet.
#[cfg(unix)]
pub fn enable_rx_timestamping(socket: &impl AsRawFd) -> io::Result<()> {
    let fd = socket.as_raw_fd();
    #[cfg(target_os = "linux")]
    let rc = {
        let flags: libc::c_uint =
            libc::SOF_TIMESTAMPING_RX_SOFTWARE | libc::SOF_TIMESTAMPING_SOFTWARE;
        unsafe {
            libc::setsockopt(
                fd,
                libc::SOL_SOCKET,
                libc::SO_TIMESTAMPING,
                &flags as *const _ as *const libc::c_void,
                size_of::<libc::c_uint>() as libc::socklen_t,
            )
        }
    };
    #[cfg(not(target_os = "linux"))]
    let rc = {
        let on: libc::c_int = 1;
        unsafe {
            libc::setsockopt(
                fd,
                libc::SOL_SOCKET,
                libc::SO_TIMESTAMP,
                &on as *const _ as *const libc::c_void,
                size_of::<libc::c_int>() as libc::socklen_t,
            )
        }
    };
    if rc == -1 {
        Err(io::Error::last_os_error())
    } else {
        Ok(())
    }
}

#[cfg(not(unix))]
pub fn enable_rx_timestamping<T>(_socket: &T) -> io::Result<()> {
    Err(io::Error::new(
        io::ErrorKind::Unsupported,
        "kernel rx timestamps are unix-only",
    ))
}

/// `recv_from` via `recvmsg`, returning the kernel receive timestamp when one
/// was attached to the datagram. Only meaningful after a successful
/// [`enable_rx_timestamping`].
#[cfg(unix)]
pub fn recv_from_timestamped(
    socket: &impl AsRawFd,
    buf: &mut [u8],
) -> io::Result<(usize, SocketAddr, Option<SystemTime>)> {
    // The union guarantees the cmsghdr alignment CMSG_FIRSTHDR expects;
    // CMSG_CAP holds SCM_TIMESTAMPING's [timespec; 3] or a timeval with room
    // for unrelated control messages.
    #[repr(C)]
    union CmsgBuf {
        _align: libc::cmsghdr,
        buf: [u8; CMSG_CAP],
    }
    let mut cmsg = CmsgBuf { buf: [0; CMSG_CAP] };

    let mut storage: libc::sockaddr_storage = unsafe { std::mem::zeroed() };
    let mut iov = libc::iovec {
        iov_base: buf.as_mut_ptr() as *mut libc::c_void,
        iov_len: buf.len(),
    };
    let mut msg: libc::msghdr = unsafe { std::mem::zeroed() };
    msg.msg_name = (&raw mut storage).cast();
    msg.msg_namelen = size_of::<libc::sockaddr_storage>() as libc::socklen_t;
    msg.msg_iov = &mut iov;
    msg.msg_iovlen = 1;
    msg.msg_control = unsafe { cmsg.buf.as_mut_ptr() }.cast();
    msg.msg_controllen = CMSG_CAP as _;

    let n = unsafe { libc::recvmsg(socket.as_raw_fd(), &mut msg, 0) };
    if n < 0 {
        return Err(io::Error::last_os_error());
    }
    let ts = if msg.msg_flags & libc::MSG_CTRUNC != 0 {
        None
    } else {
        parse_rx_timestamp(&msg)
    };
    Ok((n as usize, sockaddr_to_addr(&storage)?, ts))
}

#[cfg(not(unix))]
pub fn recv_from_timestamped<T>(
    _socket: &T,
    _buf: &mut [u8],
) -> io::Result<(usize, SocketAddr, Option<SystemTime>)> {
    Err(io::Error::new(
        io::ErrorKind::Unsupported,
        "kernel rx timestamps are unix-only",
    ))
}

#[cfg(unix)]
fn parse_rx_timestamp(msg: &libc::msghdr) -> Option<SystemTime> {
    let mut c = unsafe { libc::CMSG_FIRSTHDR(msg) };
    while !c.is_null() {
        let (level, typ) = unsafe { ((*c).cmsg_level, (*c).cmsg_type) };
        #[cfg(target_os = "linux")]
        if level == libc::SOL_SOCKET && typ == libc::SCM_TIMESTAMPING {
            // scm_timestamping is [timespec; 3]: ts[0] software, ts[2]
            // hardware. All-zero ts[0] means hardware-only; treat as absent.
            let ts0: libc::timespec =
                unsafe { std::ptr::read_unaligned(libc::CMSG_DATA(c) as *const libc::timespec) };
            if (ts0.tv_sec, ts0.tv_nsec) != (0, 0)
                && let (Ok(s), Ok(ns)) = (u64::try_from(ts0.tv_sec), u32::try_from(ts0.tv_nsec))
            {
                return Some(SystemTime::UNIX_EPOCH + Duration::new(s, ns));
            }
        }
        #[cfg(not(target_os = "linux"))]
        if level == libc::SOL_SOCKET
            && typ == libc::SCM_TIMESTAMP
            && let tv =
                unsafe { std::ptr::read_unaligned(libc::CMSG_DATA(c) as *const libc::timeval) }
            && let (Ok(s), Ok(us)) = (u64::try_from(tv.tv_sec), u32::try_from(tv.tv_usec))
        {
            return Some(SystemTime::UNIX_EPOCH + Duration::new(s, us * 1_000));
        }
        c = unsafe { libc::CMSG_NXTHDR(msg, c) };
    }
    None
}

#[cfg(unix)]
fn sockaddr_to_addr(storage: &libc::sockaddr_storage) -> io::Result<SocketAddr> {
    match storage.ss_family as libc::c_int {
        libc::AF_INET => {
            let a = unsafe { &*(storage as *const _ as *const libc::sockaddr_in) };
            Ok(SocketAddr::from((
                std::net::Ipv4Addr::from(u32::from_be(a.sin_addr.s_addr)),
                u16::from_be(a.sin_port),
            )))
        }
        libc::AF_INET6 => {
            let a = unsafe { &*(storage as *const _ as *const libc::sockaddr_in6) };
            Ok(SocketAddr::V6(std::net::SocketAddrV6::new(
                std::net::Ipv6Addr::from(a.sin6_addr.s6_addr),
                u16::from_be(a.sin6_port),
                a.sin6_flowinfo,
                a.sin6_scope_id,
            )))
        }
        _ => Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "unknown address family",
        )),
    }
}

#[cfg(all(test, unix))]
mod test {
    use super::*;
    use std::net::UdpSocket;

    #[test]
    fn test_loopback_timestamp() {
        let rx = UdpSocket::bind("127.0.0.1:0").unwrap();
        let tx = UdpSocket::bind("127.0.0.1:0").unwrap();
        enable_rx_timestamping(&rx).unwrap();
        rx.set_read_timeout(Some(Duration::from_secs(2))).unwrap();

        // Linux enables rx timestamp generation through a deferred static
        // key, so packets sent right after setsockopt may go unstamped.
        // Retry until the key has flipped.
        let mut buf = [0u8; 64];
        let mut stamped = None;
        for _ in 0..50 {
            let before = SystemTime::now();
            tx.send_to(b"ping", rx.local_addr().unwrap()).unwrap();
            let (n, addr, ts) = recv_from_timestamped(&rx, &mut buf).unwrap();
            assert_eq!(&buf[..n], b"ping");
            assert_eq!(addr, tx.local_addr().unwrap());
            if let Some(ts) = ts {
                stamped = Some((before, ts, SystemTime::now()));
                break;
            }
            std::thread::sleep(Duration::from_millis(20));
        }

        let (before, ts, after) = stamped.expect("kernel rx timestamp missing");
        assert!(ts >= before - Duration::from_secs(1));
        assert!(ts <= after + Duration::from_secs(1));
    }

    #[test]
    fn test_no_timestamp_without_enable() {
        let rx = UdpSocket::bind("127.0.0.1:0").unwrap();
        let tx = UdpSocket::bind("127.0.0.1:0").unwrap();
        rx.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
        tx.send_to(b"ping", rx.local_addr().unwrap()).unwrap();

        let mut buf = [0u8; 64];
        let (n, addr, ts) = recv_from_timestamped(&rx, &mut buf).unwrap();
        assert_eq!(&buf[..n], b"ping");
        assert_eq!(addr, tx.local_addr().unwrap());
        assert!(ts.is_none());
    }
}
