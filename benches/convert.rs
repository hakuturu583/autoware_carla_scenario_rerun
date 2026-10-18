//! `cargo bench`: parsing (the tick fast path with serde_json and simd-json,
//! and the generated record types), a Lanelet2 map, and whole conversions.
//!
//! The inputs are the repository's fixtures: a real version-1 log
//! (`tests/data/v1`, 334 ticks) and a synthetic long one (20,000 ticks of 12
//! actors) made from it; `ACSR_BENCH_MAP` names a Lanelet2 map to time
//! (the fixture map otherwise).

use std::hint::black_box;
use std::path::PathBuf;

use autoware_carla_scenario_rerun::map::{self, Projection, Projector};
use autoware_carla_scenario_rerun::parse::{
    parse_bytes, parse_ticks_generated, JsonParser, ParseOptions,
};
use autoware_carla_scenario_rerun::{convert, ConvertOptions};
use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};

fn root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn real_log() -> PathBuf {
    root().join("tests/data/v1/TrafficLightComplianceScenario-1.trajectory.jsonl")
}

fn long_log() -> Vec<u8> {
    let mut out = String::new();
    for line in std::fs::read_to_string(real_log()).unwrap().lines() {
        if !line.contains("\"type\":\"tick\"") {
            out.push_str(line);
            out.push('\n');
        }
    }
    for i in 0..20_000u64 {
        let actors: Vec<String> = (1..=12)
            .map(|a| {
                format!(
                    "[{a},{:.3},{:.3},0.059,0.287,0.082,-89.609,{:.3},0.0,0.0]",
                    i as f64 * 0.1,
                    a as f64 * 3.5,
                    1.0 + a as f64
                )
            })
            .collect();
        out.push_str(&format!(
            "{{\"type\":\"tick\",\"frame\":{},\"t\":{:.3},\"actors\":[{}],\"control\":{{\"1\":[0.3,-0.01,0.0,1]}}{}}}\n",
            1000 + i,
            i as f64 * 0.05,
            actors.join(","),
            if i % 200 == 0 { ",\"lights\":{\"77\":\"Red\",\"76\":\"Green\"}" } else { "" }
        ));
    }
    out.into_bytes()
}

fn parsing(c: &mut Criterion) {
    let real = std::fs::read(real_log()).unwrap();
    let long = long_log();
    for (name, bytes) in [("real_334_ticks", &real), ("long_20k_ticks", &long)] {
        let mut group = c.benchmark_group(format!("parse/{name}"));
        group.throughput(Throughput::Bytes(bytes.len() as u64));
        for (label, parser, chunk) in [
            ("serde_json", JsonParser::SerdeJson, 1 << 20),
            ("simd_json", JsonParser::SimdJson, 1 << 20),
            ("serde_json_1thread", JsonParser::SerdeJson, usize::MAX),
            ("simd_json_1thread", JsonParser::SimdJson, usize::MAX),
        ] {
            let options = ParseOptions {
                parser,
                strict: false,
                chunk_bytes: chunk,
            };
            group.bench_function(BenchmarkId::from_parameter(label), |b| {
                b.iter(|| parse_bytes(black_box(bytes), &options).unwrap())
            });
        }
        group.bench_function(
            BenchmarkId::from_parameter("generated_types_1thread"),
            |b| b.iter(|| parse_ticks_generated(black_box(bytes)).unwrap()),
        );
        group.finish();
    }
}

fn map_parse(c: &mut Criterion) {
    let path = std::env::var_os("ACSR_BENCH_MAP")
        .map(PathBuf::from)
        .unwrap_or_else(|| root().join("tests/data/maps/TestTown/lanelet2_map.osm"));
    let bytes = std::fs::read(&path).unwrap();
    let projection = Projection {
        projector: Projector::Local,
        origin: (0.0, 0.0, 0.0),
    };
    let mut group = c.benchmark_group("map");
    group.throughput(Throughput::Bytes(bytes.len() as u64));
    group.bench_function("osm_parse_and_project", |b| {
        b.iter(|| map::osm::parse(black_box(&bytes), &projection).unwrap())
    });
    group.finish();
}

fn conversion(c: &mut Criterion) {
    let dir = tempfile::tempdir().unwrap();
    let long = dir.path().join("Long.trajectory.jsonl");
    std::fs::write(&long, long_log()).unwrap();
    let options = ConvertOptions {
        no_map: true,
        ..Default::default()
    };
    let mut group = c.benchmark_group("convert");
    group.sample_size(20);
    for (name, input) in [("real_334_ticks", real_log()), ("long_20k_ticks", long)] {
        let out = dir.path().join(format!("{name}.rrd"));
        group.throughput(Throughput::Bytes(std::fs::metadata(&input).unwrap().len()));
        group.bench_function(name, |b| {
            b.iter(|| convert(&input, &out, &options).unwrap())
        });
    }
    group.finish();
}

criterion_group!(benches, parsing, map_parse, conversion);
criterion_main!(benches);
