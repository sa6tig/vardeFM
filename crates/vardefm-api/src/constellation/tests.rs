use super::*;

/// `n` symbols whose in-phase values encode as the symbol's number (up to 127).
fn symbols(n: usize, carriers: usize) -> Vec<Vec<Complex32>> {
    let one = f32::from(SCALE);
    (0..n).map(|s| (0..carriers).map(|c| Complex32::new(s as f32 / one, -(c as f32) / 100.0)).collect()).collect()
}

fn decode(frame: &[u8]) -> (StreamHeader, ConstellationFrame) {
    let (header, payload) = StreamHeader::decode(frame).unwrap();
    (header, ConstellationFrame::decode(payload).unwrap())
}

#[test]
fn a_short_burst_goes_out_whole() {
    let carriers: Vec<usize> = (0..40).map(|i| i + i / 5 + 1).collect();
    let syms = symbols(12, 40);
    let c = Constellation {
        t: WireTime(0x5_0000_0010),
        level: 3,
        ncw: 2,
        bits_per_symbol: 4,
        carriers: &carriers,
        symbols: &syms,
    };
    let frame = c.encode(0x1_0007);
    assert_eq!(frame.len(), StreamHeader::LEN + PAYLOAD_HEADER_LEN + 3 * 12 * 40);
    let (header, f) = decode(&frame);
    assert_eq!(header, StreamHeader { stream: 2, format: 1, seq: 7, t_low: 0x10 });
    assert_eq!((f.level, f.ncw, f.scale, f.bits_per_symbol, f.data_carriers), (3, 2, 48, 4, 40));
    assert_eq!(f.points.len(), 480);
    // Symbol 5, carrier 7: 5/48 - 0.07j.
    assert_eq!(f.points[5 * 40 + 7], Point { i: 5, q: -3, carrier: carriers[7] as u8 });
    assert_eq!(&frame[8..16], &[3, 2, 48, 4, 0xe0, 0x01, 40, 0]);
}

#[test]
fn a_long_burst_keeps_whole_symbols_and_every_carrier() {
    let carriers: Vec<usize> = (0..51).collect();
    for n in [40, 41, 120, 1000] {
        let syms = symbols(n, 51);
        let c =
            Constellation { t: WireTime(0), level: 5, ncw: 9, bits_per_symbol: 6, carriers: &carriers, symbols: &syms };
        let (_, f) = decode(&c.encode(0));
        assert!(f.points.len() <= MAX_POINTS, "{n}: {}", f.points.len());
        assert_eq!(f.points.len() % 51, 0, "{n}: whole symbols");
        for (s, symbol) in f.points.chunks(51).enumerate() {
            assert!(symbol.iter().map(|p| p.carrier).eq(0..51), "{n}: symbol {s} has every carrier in order");
        }
    }
    // 40 symbols of 51 points fit (2040); 41 do not, and every other one is kept.
    let syms = symbols(40, 51);
    let c = Constellation { t: WireTime(0), level: 5, ncw: 9, bits_per_symbol: 6, carriers: &carriers, symbols: &syms };
    assert_eq!(kept_symbols(&decode(&c.encode(0)).1, 51), (0..40).collect::<Vec<i8>>());
    let syms = symbols(41, 51);
    let c = Constellation { t: WireTime(0), level: 5, ncw: 9, bits_per_symbol: 6, carriers: &carriers, symbols: &syms };
    assert_eq!(kept_symbols(&decode(&c.encode(0)).1, 51), (0..=40).step_by(2).collect::<Vec<i8>>());
    let syms = symbols(127, 51);
    let c = Constellation { t: WireTime(0), level: 5, ncw: 9, bits_per_symbol: 6, carriers: &carriers, symbols: &syms };
    assert_eq!(kept_symbols(&decode(&c.encode(0)).1, 51), (0..127).step_by(4).collect::<Vec<i8>>());
}

/// The numbers of the symbols a frame kept, from their in-phase values (see `symbols`).
fn kept_symbols(f: &ConstellationFrame, carriers: usize) -> Vec<i8> {
    f.points.chunks(carriers).map(|s| s[0].i).collect()
}

#[test]
fn carriers_that_do_not_fit_are_left_out() {
    // 300 carriers: indices from 256 up do not fit a u8, and a frame holds 255 carriers at most.
    let carriers: Vec<usize> = (0..300).collect();
    let syms = symbols(20, 300);
    let c = Constellation { t: WireTime(0), level: 1, ncw: 1, bits_per_symbol: 2, carriers: &carriers, symbols: &syms };
    let (_, f) = decode(&c.encode(0));
    assert_eq!(f.data_carriers, 255);
    assert!(f.points.len() <= MAX_POINTS);
    assert_eq!(f.points.len() % 255, 0);
    assert!(f.points.chunks(255).all(|s| s.iter().map(|p| p.carrier).eq(0..=254)));
    // Indices past 255 are skipped, not wrapped: 250..255, then nothing.
    let high: Vec<usize> = (250..300).collect();
    let syms = symbols(2, 50);
    let c = Constellation { t: WireTime(0), level: 1, ncw: 1, bits_per_symbol: 2, carriers: &high, symbols: &syms };
    let (_, f) = decode(&c.encode(0));
    assert_eq!((f.data_carriers, f.points.len()), (6, 12));
    assert!(f.points.iter().map(|p| p.carrier).eq((250..=255).chain(250..=255)));
}

#[test]
fn points_are_clamped_and_nan_is_0() {
    let carriers = [0, 1, 2, 3];
    let syms = vec![vec![
        Complex32::new(10.0, -10.0),
        Complex32::new(f32::NAN, f32::INFINITY),
        Complex32::new(1.0, -1.0),
        Complex32::new(0.01, -0.011),
    ]];
    let c = Constellation { t: WireTime(0), level: 0, ncw: 1, bits_per_symbol: 1, carriers: &carriers, symbols: &syms };
    let (_, f) = decode(&c.encode(0));
    let iq: Vec<(i8, i8)> = f.points.iter().map(|p| (p.i, p.q)).collect();
    assert_eq!(iq, [(127, -128), (0, 0), (48, -48), (0, -1)]);
}

#[test]
fn a_frame_that_does_not_add_up_is_refused() {
    let carriers = [0, 1];
    let syms = symbols(3, 2);
    let c = Constellation { t: WireTime(0), level: 1, ncw: 1, bits_per_symbol: 2, carriers: &carriers, symbols: &syms };
    let frame = c.encode(0);
    let payload = &frame[StreamHeader::LEN..];
    assert!(ConstellationFrame::decode(payload).is_some());
    assert!(ConstellationFrame::decode(&payload[..payload.len() - 1]).is_none());
    assert!(ConstellationFrame::decode(&payload[..7]).is_none());
    let mut extra = payload.to_vec();
    extra.extend([0, 0, 0]);
    assert!(ConstellationFrame::decode(&extra).is_none());
}

#[test]
fn the_format_matches_the_frames() {
    let f = VfmConstellationFormat::current();
    assert_eq!(
        (f.stream, f.format, f.scale, usize::from(f.max_points)),
        (STREAM_VFM_CONSTELLATION, FORMAT_VFM_CONSTELLATION, SCALE, MAX_POINTS)
    );
}
