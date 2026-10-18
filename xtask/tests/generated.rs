//! The checked-in record types are what the vendored schema generates.

#[test]
fn generated_rs_is_up_to_date() {
    let schema = std::fs::read_to_string(xtask::schema_path()).unwrap();
    let expected = xtask::generate(&schema).unwrap();
    let actual = std::fs::read_to_string(xtask::generated_path()).unwrap();
    assert!(
        actual == expected,
        "src/generated.rs is out of date with schema/trajectory.schema.json: run `cargo run -p xtask`"
    );
}
