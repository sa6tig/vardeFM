//! The server: `GET /v1/status` and the WebSocket `GET /v1/stream`, on std threads.
//!
//! One acceptor thread takes connections; each connection gets a thread of its own, which reads
//! the request head, checks `Host` and `Origin`, and answers it or turns it into a stream. The
//! `publish_*` methods only put messages in each client's outbox and never wait on a socket, so a
//! slow client costs that client only: once its outbox is full, what does not fit is dropped and
//! counted.
//!
//! The API is read-only. A stream client that sends a message is closed with code 1008.

use crate::loopback::LoopbackAddr;
use crate::waterfall::WaterfallRow;
use crate::wire::{ApiEvent, Envelope, Hello, Status, StatusSnapshot, StreamHeader, WireTime};
use std::collections::{HashMap, VecDeque};
use std::io::{self, Cursor, Read, Write};
use std::net::{Shutdown, SocketAddr, TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};
use tungstenite::protocol::frame::coding::CloseCode;
use tungstenite::protocol::{CloseFrame, WebSocketConfig};
use tungstenite::{Bytes, Message, Utf8Bytes, WebSocket};

/// Events kept for `GET /v1/status`, in each of its two lists.
pub const RECENT: usize = 64;
/// Events held for each stream client; the radio reports a few per second.
pub const EVENT_ROOM: usize = 256;
/// Waterfall rows held for each stream client: 3.2 s at 10 rows per second.
pub const ROW_ROOM: usize = 32;
/// Longest message or frame taken from a client, in bytes. Any message closes the stream with
/// code 1008; a longer one is not read, and drops the connection without that code.
pub const MAX_CLIENT_MESSAGE: usize = 1024;
/// Stream clients at once; one more is answered 503.
pub const MAX_CLIENTS: usize = 8;
/// Connections served at once, stream clients included; more are closed unanswered.
const MAX_CONNECTIONS: usize = 32;
/// Longest request head read, in bytes; a longer one is answered 431.
const MAX_HEAD: usize = 4096;
/// Most header lines in a request head.
const MAX_HEADERS: usize = 64;
/// Time a client has to send its whole request head, and to take an HTTP answer.
const HEAD_TIMEOUT: Duration = Duration::from_secs(2);
/// Longest an answered connection is read for the rest of its request before it is closed.
const DRAIN_TIME: Duration = Duration::from_millis(100);
/// Most bytes read to drain an answered connection.
const DRAIN_MAX: usize = 64 * 1024;
/// Longest a stream closed as read-only waits for the client's close.
const CLOSE_WAIT: Duration = Duration::from_secs(1);
/// How long a stream thread waits for a client message before it sends what is queued.
const POLL: Duration = Duration::from_millis(10);
/// Time a stream client has to take a write; a client that stops reading ends its own stream.
const WRITE_TIMEOUT: Duration = Duration::from_secs(5);
/// Longest a dropped server waits for its connection threads.
const STOP_WAIT: Duration = Duration::from_millis(500);

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

/// What waits to be sent to one stream client.
#[derive(Default)]
struct Outbox {
    events: VecDeque<Utf8Bytes>,
    /// Events that did not fit since the client last took its outbox.
    lost_events: u64,
    rows: VecDeque<Bytes>,
    /// The latest status, if it changed since the client last took it.
    status: Option<Utf8Bytes>,
    /// The client's thread has ended.
    gone: bool,
}

#[derive(Default)]
struct Client {
    outbox: Mutex<Outbox>,
}

struct Shared {
    hello: Utf8Bytes,
    status: Mutex<Status>,
    /// Recent shared events (gaps, warnings) and recent `Vfm...` events, oldest first.
    recent: Mutex<(VecDeque<ApiEvent>, VecDeque<ApiEvent>)>,
    clients: Mutex<Vec<Arc<Client>>>,
    client_rows_dropped: AtomicU64,
    stop: AtomicBool,
    /// A clone of every open connection, to shut them down when the server stops.
    sockets: Mutex<HashMap<u64, TcpStream>>,
    connections: AtomicUsize,
    threads: Mutex<Vec<JoinHandle<()>>>,
}

/// A running server. Dropping it closes every connection and waits up to half a second for its
/// threads.
pub struct ApiServer {
    shared: Arc<Shared>,
    local: SocketAddr,
    acceptor: Option<JoinHandle<()>>,
}

impl ApiServer {
    /// Binds `addr` and starts serving; every stream client is first sent `hello`.
    pub fn start(addr: LoopbackAddr, hello: Hello) -> io::Result<Self> {
        let addr = addr.get();
        let listener =
            TcpListener::bind(addr).map_err(|e| io::Error::new(e.kind(), format!("cannot listen on {addr}: {e}")))?;
        let local = listener.local_addr()?;
        let status = Status { mode: hello.mode.clone(), ..Status::default() };
        let shared = Arc::new(Shared {
            hello: encode(&ApiEvent::Hello(hello)),
            status: Mutex::new(status),
            recent: Mutex::default(),
            clients: Mutex::default(),
            client_rows_dropped: AtomicU64::new(0),
            stop: AtomicBool::new(false),
            sockets: Mutex::default(),
            connections: AtomicUsize::new(0),
            threads: Mutex::default(),
        });
        let acceptor = {
            let shared = Arc::clone(&shared);
            thread::Builder::new().name("vardefm-api".into()).spawn(move || accept_loop(&listener, &shared))?
        };
        Ok(Self { shared, local, acceptor: Some(acceptor) })
    }

    /// The address listened on, with the port the system chose for a requested port 0.
    pub fn local_addr(&self) -> SocketAddr {
        self.local
    }

    /// Replaces the latest counters and sends them to the stream clients. The server fills in
    /// `client_rows_dropped` itself.
    pub fn publish_status(&self, mut status: Status) {
        status.client_rows_dropped = self.client_rows_dropped();
        let text = encode(&ApiEvent::Status(status.clone()));
        *lock(&self.shared.status) = status;
        self.each_client(|o| o.status = Some(text.clone()));
    }

    /// Sends an event to the stream clients and keeps it for `GET /v1/status`. A `Status` goes
    /// to [`publish_status`](Self::publish_status); a `Hello` is the server's own and is ignored.
    pub fn publish_event(&self, event: ApiEvent) {
        match event {
            ApiEvent::Hello(_) => return,
            ApiEvent::Status(status) => return self.publish_status(status),
            _ => {}
        }
        let text = encode(&event);
        {
            let mut recent = lock(&self.shared.recent);
            let list = if event.is_vfm() { &mut recent.1 } else { &mut recent.0 };
            if list.len() == RECENT {
                list.pop_front();
            }
            list.push_back(event);
        }
        self.each_client(|o| {
            if o.events.len() < EVENT_ROOM {
                o.events.push_back(text.clone());
            } else {
                o.lost_events += 1;
            }
        });
    }

    /// Sends a waterfall row to the stream clients as a binary frame. A client whose outbox is
    /// full does not get it, and [`client_rows_dropped`](Self::client_rows_dropped) counts it.
    pub fn publish_row(&self, row: &WaterfallRow) {
        let mut frame = Vec::with_capacity(StreamHeader::LEN + row.bins.len());
        frame.extend_from_slice(&StreamHeader::waterfall(row.seq, WireTime(row.t)).encode());
        frame.extend_from_slice(&row.bins);
        let frame = Bytes::from(frame);
        let dropped = &self.shared.client_rows_dropped;
        self.each_client(|o| {
            if o.rows.len() < ROW_ROOM {
                o.rows.push_back(frame.clone());
            } else {
                dropped.fetch_add(1, Ordering::Relaxed);
            }
        });
    }

    /// Rows lost so far to slow clients, over all clients.
    pub fn client_rows_dropped(&self) -> u64 {
        self.shared.client_rows_dropped.load(Ordering::Relaxed)
    }

    /// Stream clients connected now.
    pub fn clients(&self) -> usize {
        let mut n = 0;
        self.each_client(|_| n += 1);
        n
    }

    /// Calls `f` with the outbox of every client still there, and forgets those that have gone.
    fn each_client(&self, mut f: impl FnMut(&mut Outbox)) {
        lock(&self.shared.clients).retain(|c| {
            let mut outbox = lock(&c.outbox);
            if !outbox.gone {
                f(&mut outbox);
            }
            !outbox.gone
        });
    }
}

impl Drop for ApiServer {
    fn drop(&mut self) {
        self.shared.stop.store(true, Ordering::Release);
        if let Some(acceptor) = self.acceptor.take() {
            // The acceptor waits in `accept`; a connection of our own wakes it to see `stop`.
            if TcpStream::connect_timeout(&self.local, Duration::from_secs(1)).is_ok() {
                let _ = acceptor.join();
            }
        }
        for socket in lock(&self.shared.sockets).values() {
            let _ = socket.shutdown(Shutdown::Both);
        }
        // Most threads end at once. On Windows a shutdown does not end a send already waiting on a
        // client that reads nothing; that thread ends at its write timeout, and is not waited for.
        let deadline = Instant::now() + STOP_WAIT;
        let threads = std::mem::take(&mut *lock(&self.shared.threads));
        for t in threads {
            while !t.is_finished() && Instant::now() < deadline {
                thread::sleep(Duration::from_millis(1));
            }
            if t.is_finished() {
                let _ = t.join();
            }
        }
    }
}

fn encode(event: &ApiEvent) -> Utf8Bytes {
    // Serialising these types cannot fail: their maps have string keys and their floats are
    // written finite. An empty message is a smaller harm than a panic.
    serde_json::to_string(&Envelope::new(event.clone())).unwrap_or_default().into()
}

fn accept_loop(listener: &TcpListener, shared: &Arc<Shared>) {
    let mut next_id = 0u64;
    for conn in listener.incoming() {
        if shared.stop.load(Ordering::Acquire) {
            return;
        }
        let Ok(stream) = conn else {
            // Such as running out of file descriptors: give it a moment.
            thread::sleep(POLL);
            continue;
        };
        if shared.connections.load(Ordering::Acquire) >= MAX_CONNECTIONS {
            continue;
        }
        let Ok(clone) = stream.try_clone() else { continue };
        let id = next_id;
        next_id += 1;
        lock(&shared.sockets).insert(id, clone);
        shared.connections.fetch_add(1, Ordering::AcqRel);
        let task = {
            let shared = Arc::clone(shared);
            move || {
                serve(stream, &shared);
                lock(&shared.sockets).remove(&id);
                shared.connections.fetch_sub(1, Ordering::AcqRel);
            }
        };
        match thread::Builder::new().name("vardefm-api-conn".into()).spawn(task) {
            Ok(handle) => {
                let mut threads = lock(&shared.threads);
                threads.retain(|t| !t.is_finished());
                threads.push(handle);
            }
            Err(_) => {
                lock(&shared.sockets).remove(&id);
                shared.connections.fetch_sub(1, Ordering::AcqRel);
            }
        }
    }
}

/// Where a request goes, decided from its head.
enum Route {
    Status,
    Stream,
    Answer(&'static str, &'static str),
}

fn serve(mut stream: TcpStream, shared: &Arc<Shared>) {
    let _ = stream.set_write_timeout(Some(HEAD_TIMEOUT));
    let deadline = Instant::now() + HEAD_TIMEOUT;
    let Ok(timer) = stream.try_clone() else { return };
    let head = match read_head(&mut stream, deadline, |left| timer.set_read_timeout(Some(left))) {
        Ok(head) => head,
        Err(HeadError::TooLarge) => return respond(&mut stream, "431 Request Header Fields Too Large", "", b""),
        Err(HeadError::Closed) => return,
    };
    match route(&head) {
        Route::Status => {
            let snapshot = {
                let mut status = lock(&shared.status).clone();
                status.client_rows_dropped = shared.client_rows_dropped.load(Ordering::Relaxed);
                let recent = lock(&shared.recent);
                StatusSnapshot::new(status, recent.0.iter().chain(&recent.1).cloned())
            };
            let body = serde_json::to_vec(&snapshot).unwrap_or_default();
            respond(&mut stream, "200 OK", "Content-Type: application/json\r\n", &body);
        }
        Route::Stream => stream_session(stream, head, shared),
        Route::Answer(status, headers) => respond(&mut stream, status, headers, b""),
    }
}

enum HeadError {
    TooLarge,
    Closed,
}

/// Reads up to the end of the request head, by `deadline` however the bytes are spread. What
/// follows the head, if anything came with it, is kept too. `set_timeout` limits the next read.
fn read_head(
    stream: &mut impl Read,
    deadline: Instant,
    mut set_timeout: impl FnMut(Duration) -> io::Result<()>,
) -> Result<Vec<u8>, HeadError> {
    let mut buf = Vec::with_capacity(1024);
    let mut chunk = [0u8; 1024];
    loop {
        if buf.windows(4).any(|w| w == b"\r\n\r\n") {
            return Ok(buf);
        }
        if buf.len() >= MAX_HEAD {
            return Err(HeadError::TooLarge);
        }
        let left = deadline.saturating_duration_since(Instant::now());
        if left.is_zero() || set_timeout(left).is_err() {
            return Err(HeadError::Closed);
        }
        let want = (MAX_HEAD - buf.len()).min(chunk.len());
        match stream.read(&mut chunk[..want]) {
            Ok(0) => return Err(HeadError::Closed),
            Ok(n) => buf.extend_from_slice(&chunk[..n]),
            Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
            Err(_) => return Err(HeadError::Closed),
        }
    }
}

fn route(head: &[u8]) -> Route {
    let mut headers = [httparse::EMPTY_HEADER; MAX_HEADERS];
    let mut req = httparse::Request::new(&mut headers);
    match req.parse(head) {
        Ok(httparse::Status::Complete(_)) => {}
        Err(httparse::Error::TooManyHeaders) => return Route::Answer("431 Request Header Fields Too Large", ""),
        _ => return Route::Answer("400 Bad Request", ""),
    }
    let values = |name: &str| -> Vec<&[u8]> {
        req.headers.iter().filter(|h| h.name.eq_ignore_ascii_case(name)).map(|h| h.value).collect()
    };
    fn text(v: &[u8]) -> Option<&str> {
        std::str::from_utf8(v).ok().map(str::trim)
    }
    let (hosts, origins) = (values("host"), values("origin"));
    // One Host, at most one Origin: with two, which one a check reads is a guess.
    let host = match hosts.as_slice() {
        [h] => text(h),
        _ => None,
    };
    let origin = match origins.as_slice() {
        [] => Some(None),
        [o] => text(o).map(Some),
        _ => None,
    };
    if !origin.is_some_and(|origin| host_allowed(host, origin)) {
        return Route::Answer("403 Forbidden", "");
    }
    let upgrade = values("upgrade").iter().any(|v| text(v).is_some_and(|v| v.eq_ignore_ascii_case("websocket")));
    let path = req.path.unwrap_or("").split('?').next().unwrap_or("");
    match (req.method.unwrap_or(""), path) {
        ("GET", "/v1/status") => Route::Status,
        ("GET", "/v1/stream") if upgrade => Route::Stream,
        ("GET", "/v1/stream") => Route::Answer("426 Upgrade Required", "Upgrade: websocket\r\n"),
        (_, "/v1/status" | "/v1/stream") => Route::Answer("405 Method Not Allowed", "Allow: GET\r\n"),
        _ => Route::Answer("404 Not Found", ""),
    }
}

/// A loopback bind does not stop a web page in a local browser from reaching the API, directly
/// or through a name rebound to 127.0.0.1. Such requests carry a foreign `Origin` or `Host`.
fn host_allowed(host: Option<&str>, origin: Option<&str>) -> bool {
    let host_ok = host.is_some_and(is_loopback_authority);
    let origin_ok = origin.is_none_or(|origin| {
        origin.strip_prefix("http://").or_else(|| origin.strip_prefix("https://")).is_some_and(is_loopback_authority)
    });
    host_ok && origin_ok
}

/// Whether `authority` is a loopback host with an optional numeric port, and nothing more.
fn is_loopback_authority(authority: &str) -> bool {
    let (host, port) = match authority.strip_prefix("[::1]") {
        Some(port) => ("[::1]", port),
        None => authority.split_at(authority.find(':').unwrap_or(authority.len())),
    };
    let port_ok = port.is_empty()
        || port.strip_prefix(':').is_some_and(|p| !p.is_empty() && p.bytes().all(|b| b.is_ascii_digit()));
    port_ok && (host == "[::1]" || host == "127.0.0.1" || host.eq_ignore_ascii_case("localhost"))
}

fn respond(stream: &mut TcpStream, status: &str, headers: &str, body: &[u8]) {
    let head = format!("HTTP/1.1 {status}\r\n{headers}Content-Length: {}\r\nConnection: close\r\n\r\n", body.len());
    let _ = stream.write_all(head.as_bytes()).and_then(|()| stream.write_all(body)).and_then(|()| stream.flush());
    let _ = stream.shutdown(Shutdown::Write);
    // Closing with request bytes unread would reset the connection, and a reset can discard the
    // answer before the client reads it. Read what is left, for a short while.
    let deadline = Instant::now() + DRAIN_TIME;
    let mut sink = [0u8; 1024];
    let mut drained = 0;
    while drained < DRAIN_MAX {
        let left = deadline.saturating_duration_since(Instant::now());
        if left.is_zero() || stream.set_read_timeout(Some(left)).is_err() {
            break;
        }
        match stream.read(&mut sink) {
            Ok(0) | Err(_) => break,
            Ok(n) => drained += n,
        }
    }
}

/// The connection with the request head, already read, put back in front.
struct Replay {
    head: Cursor<Vec<u8>>,
    inner: TcpStream,
}

impl Read for Replay {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if (self.head.position() as usize) < self.head.get_ref().len() {
            self.head.read(buf)
        } else {
            self.inner.read(buf)
        }
    }
}

impl Write for Replay {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.inner.write(buf)
    }
    fn flush(&mut self) -> io::Result<()> {
        self.inner.flush()
    }
}

/// Marks the client gone when its thread ends, however it ends.
struct Leave(Arc<Client>);

impl Drop for Leave {
    fn drop(&mut self) {
        let mut outbox = lock(&self.0.outbox);
        *outbox = Outbox { gone: true, ..Outbox::default() };
    }
}

fn stream_session(mut stream: TcpStream, head: Vec<u8>, shared: &Shared) {
    // Registered before the handshake answer, so the client misses nothing published after it.
    let client = Arc::new(Client::default());
    {
        let mut clients = lock(&shared.clients);
        clients.retain(|c| !lock(&c.outbox).gone);
        if clients.len() >= MAX_CLIENTS {
            drop(clients);
            return respond(&mut stream, "503 Service Unavailable", "", b"");
        }
        clients.push(Arc::clone(&client));
    }
    let _leave = Leave(Arc::clone(&client));
    let config =
        WebSocketConfig::default().max_message_size(Some(MAX_CLIENT_MESSAGE)).max_frame_size(Some(MAX_CLIENT_MESSAGE));
    let replay = Replay { head: Cursor::new(head), inner: stream };
    let Ok(mut ws) = tungstenite::accept_with_config(replay, Some(config)) else { return };
    // Any error means the client has gone or is too slow; either way the stream ends.
    let _ = run(&mut ws, &client, shared);
}

fn run(ws: &mut WebSocket<Replay>, client: &Client, shared: &Shared) -> tungstenite::Result<()> {
    let tcp = &ws.get_ref().inner;
    tcp.set_read_timeout(Some(POLL))?;
    tcp.set_write_timeout(Some(WRITE_TIMEOUT))?;
    ws.write(Message::Text(shared.hello.clone()))?;
    let queued = lock(&client.outbox).status.take();
    let current = queued.unwrap_or_else(|| encode(&ApiEvent::Status(lock(&shared.status).clone())));
    ws.write(Message::Text(current))?;
    ws.flush()?;
    loop {
        if shared.stop.load(Ordering::Acquire) {
            return Ok(());
        }
        match ws.read() {
            Ok(Message::Text(_) | Message::Binary(_)) => {
                let reason = "this API is read-only".into();
                ws.close(Some(CloseFrame { code: CloseCode::Policy, reason }))?;
                return wait_for_close(ws);
            }
            // Reading a close queues the answer; flushing sends it.
            Ok(Message::Close(_)) => return ws.flush(),
            // A ping's pong is queued by the read and goes out with the next flush.
            Ok(_) => {}
            Err(tungstenite::Error::Io(e))
                if matches!(e.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut) => {}
            Err(e) => return Err(e),
        }
        let (events, lost, status, rows) = {
            let mut o = lock(&client.outbox);
            let o = &mut *o;
            (
                std::mem::take(&mut o.events),
                std::mem::take(&mut o.lost_events),
                o.status.take(),
                std::mem::take(&mut o.rows),
            )
        };
        // Events are lost only while the outbox is full, so the ones taken came before them.
        for text in events {
            ws.write(Message::Text(text))?;
        }
        if lost > 0 {
            let warning =
                ApiEvent::Warning { code: "events_lagged".to_owned(), text: format!("this client lost {lost} events") };
            ws.write(Message::Text(encode(&warning)))?;
        }
        if let Some(status) = status {
            ws.write(Message::Text(status))?;
        }
        for row in rows {
            ws.write(Message::Binary(row))?;
        }
        ws.flush()?;
    }
}

/// After our close, reads until the client's close: dropping the connection with its messages
/// unread would reset it, and a reset can discard our close frame before the client reads it.
fn wait_for_close(ws: &mut WebSocket<Replay>) -> tungstenite::Result<()> {
    let deadline = Instant::now() + CLOSE_WAIT;
    while Instant::now() < deadline {
        match ws.read() {
            Ok(_) => {}
            Err(tungstenite::Error::Io(e))
                if matches!(e.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut) => {}
            Err(tungstenite::Error::ConnectionClosed) => return Ok(()),
            Err(e) => return Err(e),
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn loopback_hosts_pass_with_or_without_a_port() {
        for host in ["127.0.0.1", "127.0.0.1:8770", "localhost:8770", "LOCALHOST", "[::1]", "[::1]:8770"] {
            assert!(host_allowed(Some(host), None), "{host}");
        }
    }

    #[test]
    fn foreign_or_missing_hosts_are_refused() {
        assert!(!host_allowed(None, None));
        for host in [
            "evil.example",
            "evil.example:8770",
            "127.0.0.1.evil.example",
            "localhost.evil.example:8770",
            "localhost:",
            "localhost:80x",
            "localhost:80@evil.example",
            "[::1].evil.example",
            "127.0.0.2:8770",
        ] {
            assert!(!host_allowed(Some(host), None), "{host}");
        }
    }

    #[test]
    fn only_loopback_origins_pass() {
        let host = Some("127.0.0.1:8770");
        for origin in ["http://127.0.0.1:8770", "http://localhost:3000", "https://localhost", "http://[::1]:5173"] {
            assert!(host_allowed(host, Some(origin)), "{origin}");
        }
        for origin in [
            "null",
            "http://evil.example",
            "https://evil.example:8770",
            "http://127.0.0.1.evil.example",
            "http://localhost@evil.example",
            "http://localhost:80@evil.example",
            "file://",
            "127.0.0.1:8770",
        ] {
            assert!(!host_allowed(host, Some(origin)), "{origin}");
        }
    }

    fn answer(head: &str) -> &'static str {
        match route(head.as_bytes()) {
            Route::Status => "status",
            Route::Stream => "stream",
            Route::Answer(status, _) => status,
        }
    }

    #[test]
    fn requests_are_routed_by_method_path_and_upgrade() {
        let get = |path: &str, extra: &str| format!("GET {path} HTTP/1.1\r\nHost: 127.0.0.1:8770\r\n{extra}\r\n");
        assert_eq!(answer(&get("/v1/status", "")), "status");
        assert_eq!(answer(&get("/v1/status?x=1", "")), "status");
        assert_eq!(answer(&get("/v1/stream", "Upgrade: websocket\r\nConnection: Upgrade\r\n")), "stream");
        assert!(answer(&get("/v1/stream", "")).starts_with("426"));
        assert!(answer(&get("/v1/command", "")).starts_with("404"));
        assert!(answer("POST /v1/status HTTP/1.1\r\nHost: 127.0.0.1\r\n\r\n").starts_with("405"));
        assert!(answer("nonsense\r\n\r\n").starts_with("400"));
    }

    #[test]
    fn a_second_host_or_origin_is_refused() {
        let two_hosts = "GET /v1/status HTTP/1.1\r\nHost: 127.0.0.1\r\nHost: evil.example\r\n\r\n";
        assert!(answer(two_hosts).starts_with("403"));
        let two_origins =
            "GET /v1/status HTTP/1.1\r\nHost: 127.0.0.1\r\nOrigin: http://localhost\r\nOrigin: http://evil.example\r\n\r\n";
        assert!(answer(two_origins).starts_with("403"));
    }

    fn soon() -> Instant {
        Instant::now() + HEAD_TIMEOUT
    }

    #[test]
    fn a_head_past_its_deadline_is_not_read() {
        let mut text = Cursor::new(b"GET / HTTP/1.1\r\nHost: x\r\n\r\n".to_vec());
        assert!(matches!(read_head(&mut text, Instant::now(), |_| Ok(())), Err(HeadError::Closed)));
    }

    #[test]
    fn a_head_is_read_to_its_end_and_no_further_than_the_limit() {
        let mut text = Cursor::new(b"GET / HTTP/1.1\r\nHost: x\r\n\r\nrest".to_vec());
        assert_eq!(read_head(&mut text, soon(), |_| Ok(())).ok().unwrap(), b"GET / HTTP/1.1\r\nHost: x\r\n\r\nrest");
        let mut long = Cursor::new([b"GET / HTTP/1.1\r\nX: ".as_slice(), &[b'a'; 2 * MAX_HEAD]].concat());
        assert!(matches!(read_head(&mut long, soon(), |_| Ok(())), Err(HeadError::TooLarge)));
        let mut cut = Cursor::new(b"GET / HTTP/1.1\r\n".to_vec());
        assert!(matches!(read_head(&mut cut, soon(), |_| Ok(())), Err(HeadError::Closed)));
    }
}
