//! VARA-compatible host command protocol (command port, CR-terminated ASCII lines).
//!
//! See docs/PROTOCOL.md, "Host interface", for the full list and deviations from VARA.

use vardefm_modem::MAX_DIGIS;

/// A parsed host command.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Command {
    MyCall(Vec<String>),
    Listen(bool),
    Connect {
        source: String,
        target: String,
        /// Digipeaters, in order from us (at most [`MAX_DIGIS`]).
        via: Vec<String>,
    },
    Disconnect,
    Abort,
    Version,
    /// Accepted for compatibility, no effect (PUBLIC, COMPRESSION, CHAT, CWID, BWxxx, ... SESSION).
    Ignored,
}

/// Parse one command line. `Err` means the modem answers `WRONG`.
#[allow(clippy::result_unit_err)]
pub fn parse(line: &str) -> Result<Command, ()> {
    let words: Vec<&str> = line.split_whitespace().collect();
    let Some(first) = words.first() else { return Err(()) };
    let kw = first.to_ascii_uppercase();
    let args = &words[1..];
    let arg = |i: usize| args.get(i).map(|s| s.to_ascii_uppercase());
    let on_off = |a: Option<String>| match a.as_deref() {
        Some("ON") => Ok(true),
        Some("OFF") => Ok(false),
        _ => Err(()),
    };
    match kw.as_str() {
        "MYCALL" => {
            if args.is_empty() || args.len() > 5 || !args.iter().all(|c| valid_call(c)) {
                return Err(());
            }
            Ok(Command::MyCall(args.iter().map(|c| c.to_ascii_uppercase()).collect()))
        }
        "LISTEN" if args.len() == 1 => on_off(arg(0)).map(Command::Listen),
        "CONNECT" => {
            // `CONNECT src dst [VIA digi1 [digi2]]`, at most two digipeaters as in VARA FM.
            let via = match args.get(2) {
                None if args.len() == 2 => &args[2..],
                Some(v) if v.eq_ignore_ascii_case("VIA") && (4..=3 + MAX_DIGIS).contains(&args.len()) => &args[3..],
                _ => return Err(()),
            };
            if !args[..2].iter().chain(via).all(|c| valid_call(c)) {
                return Err(());
            }
            Ok(Command::Connect {
                source: args[0].to_ascii_uppercase(),
                target: args[1].to_ascii_uppercase(),
                via: via.iter().map(|c| c.to_ascii_uppercase()).collect(),
            })
        }
        "DISCONNECT" if args.is_empty() => Ok(Command::Disconnect),
        "ABORT" if args.is_empty() => Ok(Command::Abort),
        "VERSION" if args.is_empty() => Ok(Command::Version),
        "PUBLIC" | "CHAT" | "CWID" if args.len() == 1 => on_off(arg(0)).map(|_| Command::Ignored),
        "COMPRESSION" if matches!(arg(0).as_deref(), Some("OFF" | "TEXT" | "FILES")) && args.len() == 1 => {
            Ok(Command::Ignored)
        }
        "WINLINK" | "P2P" if arg(0).as_deref() == Some("SESSION") && args.len() == 1 => Ok(Command::Ignored),
        "BW500" | "BW2300" | "BW2750" if args.is_empty() => Ok(Command::Ignored),
        _ => Err(()),
    }
}

/// Call sign as VARA defines it: 3-7 characters A-Z/0-9, optional SSID -1..-15, -T or -R.
pub fn valid_call(c: &str) -> bool {
    let c = c.to_ascii_uppercase();
    let (base, ssid) = match c.split_once('-') {
        Some((b, s)) => (b, Some(s)),
        None => (c.as_str(), None),
    };
    let base_ok = (3..=7).contains(&base.len()) && base.chars().all(|ch| ch.is_ascii_alphanumeric());
    let ssid_ok = match ssid {
        None => true,
        Some("T" | "R") => true,
        Some(s) => s.parse::<u8>().is_ok_and(|n| (1..=15).contains(&n)) && !s.starts_with('0'),
    };
    base_ok && ssid_ok
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_commands() {
        assert_eq!(parse("MYCALL la1abc LA2XYZ-7"), Ok(Command::MyCall(vec!["LA1ABC".into(), "LA2XYZ-7".into()])));
        assert_eq!(parse("LISTEN ON"), Ok(Command::Listen(true)));
        assert_eq!(parse("LISTEN off"), Ok(Command::Listen(false)));
        assert_eq!(
            parse("CONNECT LA1ABC LA2XYZ-7"),
            Ok(Command::Connect { source: "LA1ABC".into(), target: "LA2XYZ-7".into(), via: vec![] })
        );
        assert_eq!(
            parse("CONNECT LA1ABC LA2XYZ-7 via la3ddd LA4EEE-2"),
            Ok(Command::Connect {
                source: "LA1ABC".into(),
                target: "LA2XYZ-7".into(),
                via: vec!["LA3DDD".into(), "LA4EEE-2".into()]
            })
        );
        let one = parse("CONNECT LA1ABC LA2XYZ VIA LA3AAA");
        assert!(matches!(one, Ok(Command::Connect { via, .. }) if via == ["LA3AAA"]));
        assert_eq!(parse("DISCONNECT"), Ok(Command::Disconnect));
        assert_eq!(parse("ABORT"), Ok(Command::Abort));
        assert_eq!(parse("VERSION"), Ok(Command::Version));
        for ok in ["PUBLIC ON", "COMPRESSION TEXT", "CHAT OFF", "CWID ON", "BW2300", "P2P SESSION", "WINLINK SESSION"] {
            assert_eq!(parse(ok), Ok(Command::Ignored), "{ok}");
        }
        for bad in [
            "",
            "FOO",
            "LISTEN",
            "LISTEN MAYBE",
            "CONNECT LA1ABC",
            "CONNECT A B via C",
            "CONNECT LA1ABC LA2XYZ VIA",
            "CONNECT LA1ABC LA2XYZ OVER LA3DDD",
            "CONNECT LA1ABC LA2XYZ LA3DDD",
            "CONNECT LA1ABC LA2XYZ VIA LA3/P",
            "CONNECT LA1ABC LA2XYZ VIA LA3AAA LA3BBB LA3CCC",
            "CONNECT LA1ABC LA2XYZ VIA LA3AAA LA3BBB LA3CCC LA3DDD",
            "MYCALL X",
            "MYCALL LA1ABC-0",
        ] {
            assert_eq!(parse(bad), Err(()), "{bad}");
        }
    }

    #[test]
    fn call_signs() {
        for ok in ["LA1ABC", "N0CALL", "AB1-15", "LA2XYZ-T", "LA2XYZ-R", "W1AW"] {
            assert!(valid_call(ok), "{ok}");
        }
        for bad in ["AB", "TOOLONGX", "LA1ABC-16", "LA1ABC-", "LA1-ABC-1", "LA1ABC-01", "LA1/P"] {
            assert!(!valid_call(bad), "{bad}");
        }
    }
}
