//! The only addresses the API may listen on: a [`LoopbackAddr`] is the only way to name one.

use std::fmt;
use std::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr};
use std::str::FromStr;

/// A socket address on 127.0.0.1 or `::1`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LoopbackAddr(SocketAddr);

/// An address refused as a listening address.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NotLoopback(String);

impl fmt::Display for NotLoopback {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "refusing to listen on {}: the API binds 127.0.0.1 or [::1] only, written as an address and \
             port such as 127.0.0.1:8770",
            self.0
        )
    }
}

impl std::error::Error for NotLoopback {}

impl LoopbackAddr {
    /// The address.
    pub const fn get(self) -> SocketAddr {
        self.0
    }
}

impl TryFrom<SocketAddr> for LoopbackAddr {
    type Error = NotLoopback;

    /// Takes the two addresses exactly. `IpAddr::is_loopback` would also pass the rest of
    /// 127.0.0.0/8, which nothing needs.
    fn try_from(addr: SocketAddr) -> Result<Self, NotLoopback> {
        match addr.ip() {
            IpAddr::V4(Ipv4Addr::LOCALHOST) | IpAddr::V6(Ipv6Addr::LOCALHOST) => Ok(Self(addr)),
            _ => Err(NotLoopback(addr.to_string())),
        }
    }
}

impl FromStr for LoopbackAddr {
    type Err = NotLoopback;

    /// Reads an address and port. A host name is refused, not looked up: what it resolves to is
    /// not this program's to promise.
    fn from_str(s: &str) -> Result<Self, NotLoopback> {
        s.parse::<SocketAddr>().map_err(|_| NotLoopback(s.to_owned()))?.try_into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn takes_the_two_loopback_addresses() {
        for text in ["127.0.0.1:8770", "127.0.0.1:0", "[::1]:8770"] {
            let addr: LoopbackAddr = text.parse().unwrap();
            assert_eq!(addr.get().to_string(), text);
        }
    }

    #[test]
    fn refuses_every_address_that_is_not_loopback() {
        for text in [
            "0.0.0.0:8770",
            "[::]:8770",
            "192.168.1.10:8770",
            "10.0.0.1:8770",
            "8.8.8.8:8770",
            "[fe80::1]:8770",
            "[2001:db8::1]:8770",
            // Loopback by other names; still refused.
            "127.0.0.2:8770",
            "[::ffff:127.0.0.1]:8770",
        ] {
            let err = text.parse::<LoopbackAddr>().unwrap_err();
            assert!(err.to_string().contains("refusing to listen"), "{err}");
            let addr: SocketAddr = text.parse().unwrap();
            assert!(LoopbackAddr::try_from(addr).is_err(), "{text}");
        }
    }

    #[test]
    fn refuses_host_names_and_addresses_without_a_port() {
        for text in ["localhost:8770", "example.org:8770", "127.0.0.1", "", ":8770"] {
            assert!(text.parse::<LoopbackAddr>().is_err(), "{text}");
        }
    }
}
