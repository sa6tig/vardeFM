//! Drawing the model: burst log and status on top, waterfall and constellation below.

use ratatui::buffer::Buffer;
use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::symbols::Marker;
use ratatui::text::{Line, Span};
use ratatui::widgets::canvas::{Canvas, Points};
use ratatui::widgets::{Block, Cell, Paragraph, Row as TableRow, Table, Wrap};
use ratatui::Frame;
use vardefm_api::wire::LinkState;

use super::model::{Conn, Entry, Model, RATE};

/// Display settings the keys change.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViewOpts {
    /// Added to the tracked noise floor, in dB.
    pub floor_offset_db: f32,
    /// dB from the bottom of the colour scale to its top.
    pub span_db: f32,
}

impl Default for ViewOpts {
    fn default() -> Self {
        Self { floor_offset_db: 0.0, span_db: 60.0 }
    }
}

pub const MIN_SPAN_DB: f32 = 10.0;
pub const MAX_SPAN_DB: f32 = 120.0;

/// Waterfall time ticks, in seconds.
const TICK_S: f64 = 5.0;
/// Width of the frequency labels left of the waterfall.
const GUTTER: u16 = 5;

const RX_OK: Color = Color::Green;
const RX_BAD: Color = Color::Red;
const TX: Color = Color::Yellow;
const BUSY: Color = Color::DarkGray;

pub fn draw(f: &mut Frame, model: &Model, opts: &ViewOpts) {
    let [head, top, bottom] =
        Layout::vertical([Constraint::Length(1), Constraint::Percentage(45), Constraint::Fill(1)]).areas(f.area());
    f.render_widget(Paragraph::new(header(model)), head);

    let [log, status] = Layout::horizontal([Constraint::Fill(1), Constraint::Length(40)]).areas(top);
    draw_log(f, model, log);
    let inner_width = usize::from(status.width.saturating_sub(2));
    let status_text = Paragraph::new(status_lines(model, opts, inner_width)).wrap(Wrap { trim: false });
    f.render_widget(status_text.block(Block::bordered().title(" Status ")), status);

    // Braille cells hold 2x4 dots and a terminal cell is about twice as tall as it is wide, so
    // twice as many columns as rows draws the constellation about square.
    let side = (bottom.height.saturating_mul(2)).min(bottom.width / 3);
    let [fall, constel] = Layout::horizontal([Constraint::Fill(1), Constraint::Length(side)]).areas(bottom);
    draw_waterfall(f, model, opts, fall);
    draw_constellation(f, model, constel);
}

fn seconds(t: u64) -> String {
    format!("{:.1}", t as f64 / RATE)
}

fn header(model: &Model) -> Line<'static> {
    let (state, colour) = match &model.conn {
        Conn::Connecting => ("connecting".to_owned(), Color::Yellow),
        Conn::Connected => ("connected".to_owned(), Color::Green),
        Conn::Lost(why) => (format!("retrying: {why}"), Color::Red),
        Conn::Ended => ("replay ended".to_owned(), Color::Yellow),
    };
    let mode = model.hello.as_ref().map_or_else(String::new, |h| format!("  {} {}", h.server, h.mode));
    Line::from(vec![
        Span::styled(" vardefm watch ", Style::new().add_modifier(Modifier::BOLD | Modifier::REVERSED)),
        Span::raw(format!(" {}  ", model.source)),
        Span::styled(state, Style::new().fg(colour)),
        Span::raw(mode),
        Span::styled("   q quit  \u{2191}\u{2193} floor  +/- span  a auto", Style::new().fg(Color::DarkGray)),
    ])
}

fn draw_log(f: &mut Frame, model: &Model, area: Rect) {
    let shown = usize::from(area.height.saturating_sub(3));
    let start = model.log.len().saturating_sub(shown);
    let rows = model.log.iter().skip(start).map(|line| match &line.entry {
        Entry::Rx(rx) => {
            let colour = if rx.cw_ok < rx.ncw { RX_BAD } else { RX_OK };
            TableRow::new(vec![
                Cell::from(seconds(rx.t.0)),
                Cell::from("RX"),
                Cell::from(rx.kind.clone()),
                Cell::from(rx.level.map_or("-".into(), |l| l.to_string())),
                Cell::from(format!("{}/{}", rx.cw_ok, rx.ncw)),
                Cell::from(format!("{:.1}", rx.snr_db)),
                Cell::from(format!("{:.1}", rx.peak_dbfs)),
                Cell::from(""),
                Cell::from(""),
                Cell::from(format!("{:.2}", rx.t_end.0.saturating_sub(rx.t.0) as f64 / RATE)),
            ])
            .style(Style::new().fg(colour))
        }
        Entry::Tx(tx) => {
            let bytes =
                if tx.resent_bytes > 0 { format!("{} ({})", tx.bytes, tx.resent_bytes) } else { tx.bytes.to_string() };
            TableRow::new(vec![
                Cell::from(seconds(tx.t.0)),
                Cell::from(if tx.repeat { "RPT" } else { "TX" }),
                Cell::from(tx.kind.clone()),
                Cell::from(tx.level.map_or("-".into(), |l| l.to_string())),
                Cell::from(tx.ncw.to_string()),
                Cell::from(""),
                Cell::from(""),
                Cell::from(bytes),
                Cell::from(tx.retry.to_string()),
                Cell::from(format!("{:.2}", tx.air_s)),
            ])
            .style(Style::new().fg(TX))
        }
        Entry::Gap(t) => TableRow::new(vec![Cell::from(seconds(*t)), Cell::from("GAP"), Cell::from("audio lost")])
            .style(Style::new().fg(Color::Magenta)),
        Entry::Restart { mode } => TableRow::new(vec![Cell::from("----"), Cell::from("NEW"), Cell::from(mode.clone())])
            .style(Style::new().fg(Color::DarkGray)),
    });
    let widths = [
        Constraint::Length(8),
        Constraint::Length(3),
        Constraint::Length(11),
        Constraint::Length(3),
        Constraint::Length(5),
        Constraint::Length(6),
        Constraint::Length(6),
        Constraint::Length(11),
        Constraint::Length(3),
        Constraint::Length(5),
    ];
    let head = TableRow::new(["t s", "dir", "kind", "lvl", "cw", "SNR", "peak", "bytes", "try", "dur"])
        .style(Style::new().add_modifier(Modifier::BOLD));
    let title = format!(" Bursts ({}) ", model.log.len());
    f.render_widget(Table::new(rows, widths).header(head).block(Block::bordered().title(title)), area);
}

/// Indent of a warning's text under its time and code.
const WARNING_INDENT: usize = 2;

/// `text` word-wrapped to `width` columns, every line indented by `indent`, so a wrapped
/// warning stays under its heading. A word longer than a line is cut.
pub fn hanging(text: &str, width: usize, indent: usize) -> Vec<String> {
    let room = width.saturating_sub(indent).max(1);
    let pad = " ".repeat(indent);
    let mut lines = Vec::new();
    let mut line = String::new();
    for word in text.split_whitespace() {
        let mut word: Vec<char> = word.chars().collect();
        while word.len() > room {
            if !line.is_empty() {
                lines.push(format!("{pad}{line}"));
                line.clear();
            }
            lines.push(format!("{pad}{}", word.drain(..room).collect::<String>()));
        }
        let word: String = word.into_iter().collect();
        if word.is_empty() {
            continue;
        }
        let len = line.chars().count();
        if len > 0 && len + 1 + word.chars().count() > room {
            lines.push(format!("{pad}{line}"));
            line.clear();
        }
        if !line.is_empty() {
            line.push(' ');
        }
        line.push_str(&word);
    }
    if !line.is_empty() {
        lines.push(format!("{pad}{line}"));
    }
    lines
}

fn status_lines(model: &Model, opts: &ViewOpts, width: usize) -> Vec<Line<'static>> {
    let mut lines = Vec::new();
    let dim = Style::new().fg(Color::DarkGray);
    match &model.status {
        Some(s) => {
            lines.push(Line::raw(format!("t {} s", seconds(s.t.0))));
            lines.push(Line::raw(format!("cpu {:.1}%  tap {:.1}%  step {} ms", s.cpu, s.tap_cpu, s.longest_push_ms)));
            lines.push(Line::raw(format!("overruns {} ({} samples)  gaps {}", s.overruns, s.dropped_frames, s.gaps)));
            lines.push(Line::raw(format!("frames a slow client lost {}", s.client_rows_dropped)));
            lines.push(Line::raw(match &s.link {
                Some(link) => {
                    let state = match link.state {
                        LinkState::Idle => "idle",
                        LinkState::Listening => "listening",
                        LinkState::Connecting => "connecting",
                        LinkState::Connected => "connected",
                        LinkState::Disconnecting => "disconnecting",
                    };
                    let peer = link.peer.as_deref().map_or(String::new(), |p| format!(" {p}"));
                    let level = link.level.map_or(String::new(), |l| format!(" L{l}"));
                    format!("link {state}{peer}{level}  tx queued {}", link.tx_queued)
                }
                None => "link -".into(),
            }));
        }
        None => lines.push(Line::styled("no status yet", dim)),
    }
    let busy = model.busy.back().is_some_and(|&(_, b)| b);
    lines.push(Line::styled(
        if busy { "channel busy" } else { "channel free" },
        Style::new().fg(if busy { Color::Yellow } else { Color::Green }),
    ));
    let floor = match (model.hello.as_ref(), model.floor.value()) {
        (Some(h), Some(v)) => {
            format!("{:.1} dBFS", h.waterfall.db_floor + h.waterfall.db_step * f32::from(v) + opts.floor_offset_db)
        }
        _ => "-".into(),
    };
    let offset = if opts.floor_offset_db == 0.0 { String::new() } else { format!(" ({:+.0})", opts.floor_offset_db) };
    lines.push(Line::raw(format!("floor {floor}{offset}  span {:.0} dB", opts.span_db)));
    if model.unknown > 0 {
        lines.push(Line::styled(format!("messages not understood {}", model.unknown), dim));
    }
    let red = Style::new().fg(Color::Red);
    for (t, code, text) in model.warnings.iter().rev() {
        lines.push(Line::styled(format!("{} s {code}", seconds(*t)), red.add_modifier(Modifier::BOLD)));
        lines.extend(hanging(text, width, WARNING_INDENT).into_iter().map(|l| Line::styled(l, red)));
    }
    lines
}

/// The largest bin of each of `n` frequency bands, highest band first. Band `k` from the
/// bottom covers bins `k * len / n` up to `(k + 1) * len / n`, at least one bin.
pub fn bands(bins: &[u8], n: usize) -> Vec<u8> {
    if bins.is_empty() {
        return vec![0; n];
    }
    (0..n)
        .rev()
        .map(|k| {
            let lo = (k * bins.len() / n).min(bins.len() - 1);
            let hi = ((k + 1) * bins.len() / n).clamp(lo + 1, bins.len());
            bins[lo..hi].iter().copied().max().unwrap_or(0)
        })
        .collect()
}

/// A colour for `level` (0 at the floor, 1 at the top of the span): black, blue, cyan, yellow,
/// red, white.
pub fn colour(level: f32) -> Color {
    const STOPS: [(f32, [f32; 3]); 6] = [
        (0.0, [0.0, 0.0, 0.0]),
        (0.2, [0.0, 0.0, 140.0]),
        (0.45, [0.0, 170.0, 220.0]),
        (0.7, [240.0, 230.0, 0.0]),
        (0.9, [240.0, 40.0, 0.0]),
        (1.0, [255.0, 255.0, 255.0]),
    ];
    let x = if level.is_finite() { level.clamp(0.0, 1.0) } else { 0.0 };
    let i = STOPS.iter().rposition(|&(at, _)| at <= x).unwrap_or(0).min(STOPS.len() - 2);
    let ((a, ca), (b, cb)) = (STOPS[i], STOPS[i + 1]);
    let w = (x - a) / (b - a);
    let mix = |j: usize| (ca[j] + (cb[j] - ca[j]) * w).round() as u8;
    Color::Rgb(mix(0), mix(1), mix(2))
}

fn draw_waterfall(f: &mut Frame, model: &Model, opts: &ViewOpts, area: Rect) {
    let title = match &model.hello {
        Some(h) => format!(" Waterfall 0-{:.1} kHz ", f32::from(h.waterfall.bins) * h.waterfall.bin_hz / 1000.0),
        None => " Waterfall ".to_owned(),
    };
    let block = Block::bordered().title(title);
    let inner = block.inner(area);
    f.render_widget(block, area);
    paint_waterfall(f.buffer_mut(), model, opts, inner);
}

/// Time runs left to right, newest at the right edge, one column per row. Below the spectrum
/// a strip marks the bursts and busy time, and below that a time scale.
fn paint_waterfall(buf: &mut Buffer, model: &Model, opts: &ViewOpts, area: Rect) {
    let Some(hello) = &model.hello else { return };
    if area.width <= GUTTER || area.height < 3 {
        return;
    }
    let fmt = &hello.waterfall;
    let spectrum_rows = area.height - 2;
    let width = usize::from(area.width - GUTTER);
    let x0 = area.x + GUTTER;
    let hop = (RATE / f64::from(fmt.rows_per_s.max(0.1))) as u64;

    // Frequency labels.
    let top_hz = f32::from(fmt.bins) * fmt.bin_hz;
    for r in (0..spectrum_rows).step_by(4) {
        let hz = top_hz * (1.0 - (f32::from(r) + 0.5) / f32::from(spectrum_rows));
        buf.set_string(area.x, area.y + r, format!("{:>4.1}", hz / 1000.0), Style::new().fg(Color::DarkGray));
    }

    let floor_db =
        model.floor.value().map_or(fmt.db_floor, |v| fmt.db_floor + fmt.db_step * f32::from(v)) + opts.floor_offset_db;
    let level = |v: u8| (fmt.db_floor + fmt.db_step * f32::from(v) - floor_db) / opts.span_db;

    let skip = model.rows.len().saturating_sub(width);
    let blank = width.saturating_sub(model.rows.len());
    let mut last_tick = None;
    for (i, row) in model.rows.iter().skip(skip).enumerate() {
        let x = x0 + (blank + i) as u16;
        let bands = bands(&row.bins, usize::from(spectrum_rows) * 2);
        for r in 0..spectrum_rows {
            if let Some(cell) = buf.cell_mut((x, area.y + r)) {
                let (upper, lower) = (bands[2 * usize::from(r)], bands[2 * usize::from(r) + 1]);
                cell.set_symbol("\u{2580}").set_fg(colour(level(upper))).set_bg(colour(level(lower)));
            }
        }
        if let Some(c) = marker(model, row.t, row.t + hop) {
            buf.set_string(x, area.y + spectrum_rows, "\u{2588}", Style::new().fg(c));
        }
        let tick = (row.t as f64 / RATE / TICK_S) as u64;
        if last_tick.is_some_and(|last| last != tick) && x + 6 <= area.right() {
            buf.set_string(x, area.y + spectrum_rows + 1, format!("|{:.0}s", tick as f64 * TICK_S), Style::new());
        }
        last_tick = Some(tick);
    }
}

/// The colour that marks the span `t0..t1` of the current session, if anything happened in it.
fn marker(model: &Model, t0: u64, t1: u64) -> Option<Color> {
    let overlaps = |a: u64, b: u64| a < t1 && b > t0;
    let mut found = None;
    for line in model.log.iter().rev().filter(|l| l.session == model.session) {
        match &line.entry {
            Entry::Tx(tx) if overlaps(tx.t.0, tx.t.0 + (f64::from(tx.air_s) * RATE) as u64) => return Some(TX),
            Entry::Rx(rx) if overlaps(rx.t.0, rx.t_end.0.max(rx.t.0 + 1)) => {
                found = Some(if rx.cw_ok < rx.ncw || found == Some(RX_BAD) { RX_BAD } else { RX_OK });
            }
            _ => {}
        }
    }
    found.or_else(|| model.busy_at(t0).then_some(BUSY))
}

/// Ideal points of a constellation with unit mean energy: BPSK, or square QAM.
pub fn ideal_points(bits_per_symbol: u8) -> Vec<(f64, f64)> {
    if bits_per_symbol == 1 {
        return vec![(-1.0, 0.0), (1.0, 0.0)];
    }
    if bits_per_symbol == 0 || bits_per_symbol % 2 == 1 || bits_per_symbol > 12 {
        return Vec::new();
    }
    let m = 1i32 << (bits_per_symbol / 2);
    let norm = (2.0 * f64::from(m * m - 1) / 3.0).sqrt();
    let axis: Vec<f64> = (0..m).map(|i| f64::from(2 * i - (m - 1)) / norm).collect();
    axis.iter().flat_map(|&i| axis.iter().map(move |&q| (i, q))).collect()
}

fn draw_constellation(f: &mut Frame, model: &Model, area: Rect) {
    let Some(shown) = &model.constellation else {
        f.render_widget(Block::bordered().title(" Constellation "), area);
        return;
    };
    let frame = &shown.frame;
    let title = match &shown.burst {
        Some(rx) => format!(" {} L{} {:.1} dB {}/{} ", rx.kind, frame.level, rx.snr_db, rx.cw_ok, rx.ncw),
        None => format!(" L{} at {} s ", frame.level, seconds(shown.t)),
    };
    let ideal = ideal_points(frame.bits_per_symbol);
    let outer = ideal.iter().map(|&(i, q)| i.abs().max(q.abs())).fold(1.0, f64::max);
    let bound = outer * 1.35;
    let scale = f64::from(frame.scale.max(1));
    let points: Vec<(f64, f64)> =
        frame.points.iter().map(|p| (f64::from(p.i) / scale, f64::from(p.q) / scale)).collect();
    let canvas = Canvas::default()
        .block(Block::bordered().title(title))
        .marker(Marker::Braille)
        .x_bounds([-bound, bound])
        .y_bounds([-bound, bound])
        .paint(move |ctx| {
            ctx.draw(&Points { coords: &ideal, color: Color::DarkGray });
            ctx.layer();
            ctx.draw(&Points { coords: &points, color: Color::Cyan });
        });
    f.render_widget(canvas, area);
}
