#!/usr/bin/env bash
# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

# The first argument overrides GRACE_CONVERTER_BIN and the default build path.
BIN="${GRACE_CONVERTER_BIN:-$REPO_ROOT/out/build/ci-fast/backend/grace_backend_convert}"
if (( $# > 0 )); then
  BIN="$1"
  shift
fi
(( $# == 0 )) || {
  echo "Usage: $0 [path-to-grace_backend_convert]" >&2
  exit 2
}

# Inputs
YAML_IN="${GRACE_PIPELINE_YAML_IN:-$SCRIPT_DIR/data/maps/yaml/default/problem.yaml}"
GRID_MAP_IN="${GRACE_PIPELINE_GRID_MAP_IN:-$SCRIPT_DIR/data/maps/movingAI/default/random-32-32-20.map}"
GRID_SCEN_IN="${GRACE_PIPELINE_GRID_SCEN_IN:-$SCRIPT_DIR/data/maps/movingAI/default/random-32-32-20-random-1.scen}"

# Output dir
OUTDIR="${GRACE_PIPELINE_OUTDIR:-$REPO_ROOT/results/pipeline}"
mkdir -p "$OUTDIR"

# Parameters
WALL=0.25
ROBOT_R=0.25

# Cell size for rasterizing continuous -> grid (finer grid for export)
RASTER_CELL=0.5

# Cell size to INTERPRET an existing MovingAI map/scen dataset
GRID_CELL=1.0

# Default robot type when synthesizing YAML from scenarios
DEFAULT_TYPE="unicycle_first_order_0"

# --- Helpers -----------------------------------------------------------------

die() { echo "[FATAL] $*" >&2; exit 1; }

need_file() { [[ -f "$1" ]] || die "Missing file: $1"; }

run() {
  printf '\n>'
  printf ' %q' "$@"
  printf '\n'
  "$@"
}

ensure_bin() {
  [[ -n "$BIN" ]] || die "Converter path is empty"
  [[ -x "$BIN" ]] || die "Converter '$BIN' is not executable"
}

# --- Checks ------------------------------------------------------------------

ensure_bin
need_file "$YAML_IN"
need_file "$GRID_MAP_IN"
need_file "$GRID_SCEN_IN"

echo "Using converter: $BIN"
echo "Output directory: $OUTDIR"

# --- (1) Continuous (problem.yaml) -> Roadmap (graph.xml) + tasks.xml -------

GRAPH_XML="$OUTDIR/cont_to_roadmap.xml"
TASKS_XML_FROM_YAML="$OUTDIR/cont_tasks.xml"

run "$BIN" --mode c2r --in "$YAML_IN" --out "$GRAPH_XML" \
  --wall "$WALL" --radius "$ROBOT_R" --prm-samples 1500 --prm-time 3.0

# Convert scenarios embedded in YAML into tasks.xml (nearest nodes on roadmap)
run "$BIN" --mode y2t --in "$YAML_IN" --out "$TASKS_XML_FROM_YAML" \
  --graph "$GRAPH_XML"

# --- (2) Continuous (problem.yaml) -> Grid (.map) + .scen --------------------

GRID_MAP_OUT="$OUTDIR/cont_to_grid.map"
GRID_SCEN_OUT="$OUTDIR/cont_to_grid.scen"

# Map export (MovingAI)
run "$BIN" --mode c2g --in "$YAML_IN" --out "$GRID_MAP_OUT" \
  --grid-fmt movingai --cell "$RASTER_CELL" --wall "$WALL" \
  --radius "$ROBOT_R"

# Scenario export (YAML robots -> .scen). The .scen will reference the map filename.
run "$BIN" --mode y2s --in "$YAML_IN" --out "$GRID_SCEN_OUT" \
  --map "$GRID_MAP_OUT" --cell "$RASTER_CELL"

# --- (3) Grid (.map + .scen) -> Continuous (problem.yaml with env+robots) ----

YAML_FROM_GRID="$OUTDIR/grid_to_yaml.yaml"

# Single step: combine grid environment + scenario robots into one YAML
run "$BIN" --mode s2y --in "$GRID_SCEN_IN" --out "$YAML_FROM_GRID" \
  --map "$GRID_MAP_IN" --grid-fmt movingai --cell "$GRID_CELL" \
  --default-type "$DEFAULT_TYPE"

# (Optional) If you also want an environment-only YAML (no robots), uncomment:
# ENV_ONLY_YAML="$OUTDIR/grid_env_only.yaml"
# run "$BIN" --mode g2y --in "$GRID_MAP_IN" --out "$ENV_ONLY_YAML" \
#   --grid-fmt movingai --cell "$GRID_CELL"

# --- Summary -----------------------------------------------------------------

echo -e "\n=== Done. Artifacts ==="
echo "  Roadmap XML (GraphML):      $GRAPH_XML"
echo "  Tasks from YAML -> roadmap: $TASKS_XML_FROM_YAML"
echo "  Grid map (MovingAI):        $GRID_MAP_OUT"
echo "  Grid scen (MovingAI):       $GRID_SCEN_OUT"
echo "  YAML from grid+scen:        $YAML_FROM_GRID"
# echo "  YAML env-only from grid:     $ENV_ONLY_YAML"
echo "==========================================================================="
