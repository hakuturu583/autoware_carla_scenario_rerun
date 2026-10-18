//! End-to-end timing of a directory conversion (not a CLI: a benchmark
//! harness for `cargo run --release --example convert_dir`).
//!
//! `convert_dir DIR [--out-dir D] [--map-dir D] [--jobs N] [--simd] [--repeat N]`

use std::path::PathBuf;
use std::time::Instant;

use autoware_carla_scenario_rerun::{convert_dir, ConvertOptions, JsonParser};

fn main() {
    env_logger::init();
    let mut args = std::env::args().skip(1);
    let dir = PathBuf::from(args.next().expect("DIR"));
    let (mut out_dir, mut options, mut jobs, mut repeat) =
        (None, ConvertOptions::default(), None, 1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--out-dir" => out_dir = args.next().map(PathBuf::from),
            "--map-dir" => options.map_dir = args.next().map(PathBuf::from),
            "--jobs" => jobs = args.next().and_then(|n| n.parse().ok()),
            "--simd" => options.parser = JsonParser::SimdJson,
            "--repeat" => repeat = args.next().and_then(|n| n.parse().ok()).unwrap_or(1),
            other => panic!("unknown argument {other}"),
        }
    }
    for round in 0..repeat {
        let start = Instant::now();
        let results = convert_dir(&dir, out_dir.as_deref(), &options, jobs).expect("listing");
        let wall = start.elapsed().as_secs_f64();
        let (mut bytes, mut lines, mut ticks, mut rrd, mut busy) = (0u64, 0u64, 0u64, 0u64, 0f64);
        let mut failed = 0;
        for r in &results {
            match &r.result {
                Ok(s) => {
                    bytes += s.bytes;
                    lines += s.lines;
                    ticks += s.ticks;
                    rrd += s.rrd_bytes;
                    busy += s.seconds;
                    if round == repeat - 1 {
                        println!(
                            "{:<60} {:>6} ticks {:>8} B -> {:>8} B  parse {:>6.2} ms  map {:>6.2} ms{}  write {:>6.2} ms  total {:>6.2} ms",
                            s.input.file_name().unwrap().to_string_lossy(),
                            s.ticks,
                            s.bytes,
                            s.rrd_bytes,
                            s.parse_seconds * 1e3,
                            s.map_seconds * 1e3,
                            if s.map_cached { " (cached)" } else { "" },
                            s.write_seconds * 1e3,
                            s.seconds * 1e3
                        );
                    }
                }
                Err(e) => {
                    failed += 1;
                    eprintln!("FAILED {e}");
                }
            }
        }
        let n = results.len();
        println!(
            "round {round}: {n} files ({failed} failed) in {:.1} ms wall: {:.1} MB/s, {:.0} lines/s, {:.0} ticks/s; per file {:.2} ms (sum of per-file {:.1} ms, {:.3} ms/tick); {:.1} KB .rrd/file",
            wall * 1e3,
            bytes as f64 / 1e6 / wall,
            lines as f64 / wall,
            ticks as f64 / wall,
            busy * 1e3 / n.max(1) as f64,
            busy * 1e3,
            busy * 1e3 / ticks.max(1) as f64,
            rrd as f64 / 1e3 / n.max(1) as f64
        );
    }
}
