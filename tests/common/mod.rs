#![allow(dead_code)]

use std::io::{Read, Write};
use std::net::{IpAddr, SocketAddr, TcpListener, TcpStream};
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::Duration;

use fanuc_ucl::hmi::server::{HmiRequest, ack_resp, init_ack, parse_request};
use snare::{Sim, SimBuilder};

/// One thread at a time, switching only at waits, on a virtual clock that
/// moves only when every thread is blocked. A wait the drivers hide from the
/// sim shows up as a stall, and a deadline kept on another clock as a wrong
/// virtual duration.
pub fn builder() -> SimBuilder {
    Sim::builder()
        .deterministic()
        .seed(7)
        .strict_sockopts()
        .stuck_after(Duration::from_secs(30))
}

pub fn sim() -> Sim {
    builder().build()
}

pub type Participant = Box<dyn FnOnce() + Send>;

/// Starts participants that must all begin at the same virtual instant, then
/// waits for `client` to finish. Servers bind their sockets before this, so
/// the client can never reach an address nobody is listening on yet.
pub fn run<T: Send + 'static>(
    participants: Vec<Participant>,
    client: impl FnOnce() -> T + Send + 'static,
) -> T {
    let setup = snare::sched::setup_scope("test-spawn");
    let others: Vec<_> = participants.into_iter().map(std::thread::spawn).collect();
    let client = std::thread::spawn(client);
    drop(setup);
    let out = client.join().unwrap();
    for h in others {
        h.join().unwrap();
    }
    out
}

/// Reads `\r\n`-terminated lines from `stream` until it closes.
pub fn rmi_lines(stream: &mut TcpStream, mut on_line: impl FnMut(&str) -> Option<String>) {
    let mut pending = String::new();
    let mut buf = [0u8; 4096];
    loop {
        let n = match stream.read(&mut buf) {
            Ok(0) | Err(_) => return,
            Ok(n) => n,
        };
        pending.push_str(std::str::from_utf8(&buf[..n]).unwrap());
        while let Some(end) = pending.find("\r\n") {
            let line: String = pending.drain(..end + 2).collect();
            if let Some(reply) = on_line(line.trim_end())
                && stream.write_all(format!("{reply}\r\n").as_bytes()).is_err()
            {
                return;
            }
        }
    }
}

pub const RMI_CONNECT_REPLY: &str = r#"{"Communication":"FRC_Connect","ErrorID":0,"PortNumber":16002,"MajorVersion":7,"MinorVersion":1}"#;

/// An RMI controller at `ip` for one session: the FRC_Connect handshake on
/// 16001, then 16002 answering FRC_Disconnect, until the client closes it.
pub fn rmi_server(ip: IpAddr) -> Participant {
    let control = TcpListener::bind(SocketAddr::new(ip, 16001)).unwrap();
    let session = TcpListener::bind(SocketAddr::new(ip, 16002)).unwrap();
    Box::new(move || {
        let (mut c, _) = control.accept().unwrap();
        rmi_lines(&mut c, |_| Some(RMI_CONNECT_REPLY.into()));
        let (mut s, _) = session.accept().unwrap();
        rmi_lines(&mut s, |line| {
            line.contains("FRC_Disconnect")
                .then(|| r#"{"Communication":"FRC_Disconnect","ErrorID":0}"#.into())
        })
    })
}

/// An SNPX server at `ip`:60008 for one connection, acknowledging every
/// request and counting them, until the client closes it.
pub fn hmi_server(ip: IpAddr, requests: Arc<AtomicU32>) -> Participant {
    let listener = TcpListener::bind(SocketAddr::new(ip, 60008)).unwrap();
    Box::new(move || {
        let (mut s, _) = listener.accept().unwrap();
        let mut pending = Vec::new();
        let mut buf = [0u8; 4096];
        loop {
            let n = match s.read(&mut buf) {
                Ok(0) | Err(_) => return,
                Ok(n) => n,
            };
            pending.extend_from_slice(&buf[..n]);
            while let Some((req, used)) = parse_request(&pending) {
                pending.drain(..used);
                requests.fetch_add(1, Ordering::SeqCst);
                let reply = match req {
                    HmiRequest::Init { .. } => init_ack(),
                    other => ack_resp(other.seq()),
                };
                if s.write_all(&reply).is_err() {
                    return;
                }
            }
        }
    })
}
