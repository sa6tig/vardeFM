//! Captures of a monitor API stream: one JSON object per line, for `vardefm watch --save` and
//! `--replay`.
//!
//! A line is `{"ms":<since the capture started>,"text":<the text message>}` for a JSON message, or
//! `{"ms":...,"bin":<base64 of the binary frame>}` for a waterfall row or a constellation. The
//! text message is stored as a string, exactly as it came, so a capture replays what the server
//! sent even when this tool does not understand it.

use std::io::{self, BufRead, Write};

use base64::engine::general_purpose::STANDARD;
use base64::Engine;
use serde_json::{json, Value};

/// One message of the stream.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Message {
    /// A JSON envelope.
    Text(String),
    /// A binary frame: a waterfall row or a constellation.
    Binary(Vec<u8>),
}

/// A message and when it arrived.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Record {
    /// Milliseconds since the capture started.
    pub ms: u64,
    pub msg: Message,
}

impl Record {
    /// The record as one line of JSON, without the newline.
    pub fn to_line(&self) -> String {
        match &self.msg {
            Message::Text(text) => json!({ "ms": self.ms, "text": text }).to_string(),
            Message::Binary(bytes) => json!({ "ms": self.ms, "bin": STANDARD.encode(bytes) }).to_string(),
        }
    }

    /// Reads one line written by [`Record::to_line`].
    pub fn from_line(line: &str) -> Result<Self, String> {
        let value: Value = serde_json::from_str(line).map_err(|e| e.to_string())?;
        let ms = value.get("ms").and_then(Value::as_u64).ok_or("no \"ms\"")?;
        let msg = match (value.get("text"), value.get("bin")) {
            (Some(Value::String(text)), None) => Message::Text(text.clone()),
            (None, Some(Value::String(bin))) => Message::Binary(STANDARD.decode(bin).map_err(|e| e.to_string())?),
            _ => return Err("needs one of \"text\" or \"bin\", as a string".into()),
        };
        Ok(Self { ms, msg })
    }
}

/// Writes records to a capture, one line each, flushed per line so a capture cut short by a
/// crash keeps everything before it.
pub struct CaptureWriter<W: Write> {
    out: W,
}

impl<W: Write> CaptureWriter<W> {
    pub fn new(out: W) -> Self {
        Self { out }
    }

    pub fn write(&mut self, record: &Record) -> io::Result<()> {
        writeln!(self.out, "{}", record.to_line())?;
        self.out.flush()
    }
}

/// Reads every record of a capture. Blank lines are skipped; any other line that is not a
/// record is an error naming its line number.
pub fn read_all(input: impl BufRead) -> Result<Vec<Record>, String> {
    let mut records = Vec::new();
    for (n, line) in input.lines().enumerate() {
        let line = line.map_err(|e| format!("line {}: {e}", n + 1))?;
        if line.trim().is_empty() {
            continue;
        }
        records.push(Record::from_line(&line).map_err(|e| format!("line {}: {e}", n + 1))?);
    }
    Ok(records)
}
