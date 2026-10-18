#!/usr/bin/env bash
# Vendor the trajectory log schema from an autoware_carla_scenario checkout and
# regenerate the record types from it.
#
#   tools/update_schema.sh ~/workspace/autoware_carla_scenario
#
# Copies the schema, records where it came from (schema/SOURCE), and runs
# `cargo run -p xtask`, which rewrites src/generated.rs. Then `cargo test`:
# tests/schema.rs fails until a new record type is handled (parse.rs, rrd.rs),
# and xtask/tests/generated.rs keeps generated.rs and the schema in step.
set -euo pipefail
acs=${1:?usage: tools/update_schema.sh ACS_CHECKOUT}
here=$(cd "$(dirname "$0")/.." && pwd)
rel=autoware_carla_scenario/src/autoware_carla_scenario/schemas/trajectory.schema.json
cp "$acs/$rel" "$here/schema/trajectory.schema.json"
commit=$(git -C "$acs" rev-parse HEAD)
dirty=$(git -C "$acs" status --porcelain -- "$rel" | head -c1)
version=$(python3 -c 'import json, sys; print(json.load(open(sys.argv[1])).get("x-schema-version", 1))' "$here/schema/trajectory.schema.json")
cat > "$here/schema/SOURCE" <<SRC
# Where schema/trajectory.schema.json was copied from (tools/update_schema.sh writes this).
repository: autowarefoundation/autoware_carla_scenario
path: $rel
ref: $commit${dirty:+ (with uncommitted changes)}
x-schema-version: $version
sha256: $(sha256sum "$here/schema/trajectory.schema.json" | cut -d' ' -f1)
SRC
(cd "$here" && cargo run -q -p xtask)
echo "vendored schema version $version from $acs ($commit); now run: cargo test --workspace"
