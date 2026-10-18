//! Property tests of the tick fast path: for any ticks, what it builds equals
//! what the generated `TickRecord` builds, whatever the chunk size and the
//! JSON parser.

use autoware_carla_scenario_rerun::parse::{
    parse_bytes, parse_ticks_generated, JsonParser, ParseOptions,
};
use proptest::prelude::*;
use serde_json::json;

fn tick_line(
    frame: u64,
    t: f64,
    actors: &[(u32, [f64; 9])],
    control: &[(u32, [f64; 3], i64)],
    lights: &[(u32, usize)],
) -> String {
    const STATES: [&str; 5] = ["Red", "Yellow", "Green", "Off", "Unknown"];
    let actors: Vec<_> = actors
        .iter()
        .map(|(id, v)| {
            let mut row = vec![json!(id)];
            row.extend(v.iter().map(|x| json!(x)));
            row
        })
        .collect();
    let control: serde_json::Map<_, _> = control
        .iter()
        .map(|(id, c, gear)| (id.to_string(), json!([c[0], c[1], c[2], gear])))
        .collect();
    let mut tick =
        json!({"type": "tick", "frame": frame, "t": t, "actors": actors, "control": control});
    if !lights.is_empty() {
        let lights: serde_json::Map<_, _> = lights
            .iter()
            .map(|(id, s)| (id.to_string(), json!(STATES[*s])))
            .collect();
        tick["lights"] = serde_json::Value::Object(lights);
    }
    tick.to_string()
}

/// Numbers as the recorder writes them: rounded to 3 decimals. (Past ~15
/// significant digits serde_json's default float parsing is not correctly
/// rounded, simd-json's is: the two would disagree in the last bit.)
fn finite() -> impl Strategy<Value = f64> {
    (-1.0e5f64..1.0e5).prop_map(|x| (x * 1000.0).round() / 1000.0)
}

prop_compose! {
    fn tick()(
        frame in 0u64..(1u64 << 53),
        t in (0.0f64..1.0e4).prop_map(|x| (x * 1000.0).round() / 1000.0),
        actors in prop::collection::btree_map(0u32..1000, prop::array::uniform9(finite()), 1..6),
        control in prop::collection::btree_map(0u32..1000, (prop::array::uniform3((0.0f64..1.0).prop_map(|x| (x * 1000.0).round() / 1000.0)), -1i64..6), 0..3),
        lights in prop::collection::btree_map(0u32..1000, 0usize..5, 0..4),
    ) -> String {
        let actors: Vec<_> = actors.into_iter().collect();
        let control: Vec<_> = control.into_iter().map(|(id, (c, g))| (id, c, g)).collect();
        let lights: Vec<_> = lights.into_iter().collect();
        tick_line(frame, t, &actors, &control, &lights)
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(128))]

    #[test]
    fn fast_path_equals_generated_types(ticks in prop::collection::vec(tick(), 1..40), chunk in 1usize..4096, simd in any::<bool>()) {
        let text = ticks.join("\n") + "\n";
        let expected = parse_ticks_generated(text.as_bytes()).unwrap();
        let options = ParseOptions {
            parser: if simd { JsonParser::SimdJson } else { JsonParser::SerdeJson },
            strict: true,
            chunk_bytes: chunk,
        };
        let parsed = parse_bytes(text.as_bytes(), &options).unwrap();
        prop_assert_eq!(parsed.lines, ticks.len() as u64);
        // Keys are written sorted (BTreeMap): file order is id order, which is
        // how the generated path orders its maps.
        prop_assert_eq!(parsed.ticks, expected);
    }

    #[test]
    fn garbage_never_panics(bytes in prop::collection::vec(any::<u8>(), 0..512)) {
        let _ = parse_bytes(&bytes, &ParseOptions::default());
        let _ = parse_bytes(&bytes, &ParseOptions { parser: JsonParser::SimdJson, ..Default::default() });
    }
}
