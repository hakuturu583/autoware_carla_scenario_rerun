//! The parser handles exactly the record types of the vendored schema.

use autoware_carla_scenario_rerun::generated::RECORD_TYPES;
use autoware_carla_scenario_rerun::parse::HANDLED_TYPES;

#[test]
fn every_schema_record_type_is_handled() {
    let schema: Vec<&str> = RECORD_TYPES.iter().map(|(t, _)| *t).collect();
    assert_eq!(
        HANDLED_TYPES, schema,
        "the schema's record types changed: handle the new ones in parse.rs (and rrd.rs)"
    );
}

#[test]
fn vendored_schema_matches_the_generated_types() {
    let text = std::fs::read_to_string(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/schema/trajectory.schema.json"
    ))
    .unwrap();
    let schema: serde_json::Value = serde_json::from_str(&text).unwrap();
    assert_eq!(
        schema["x-schema-version"].as_u64(),
        Some(autoware_carla_scenario_rerun::generated::SCHEMA_VERSION)
    );
}
