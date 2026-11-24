#!/usr/bin/env bash
# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

set -euo pipefail

# concat_with_headers.sh
# Usage:
#   ./concat_with_headers.sh [-o output-file] file1 file2 ...
# Example:
#   ./concat_with_headers.sh -o all.txt "A file.txt" B C

# Defaults
OUT="concatenated.txt"
DEFAULT_INPUTS=(
"main.cpp"
# "main_mapConvert.cpp"

"inc/app/SimulationApp.h"
"src/app/SimulationApp.cpp"

"inc/Simulation.h"
"src/Simulation.cpp"

# "inc/env/ICapabilities.h"
# "inc/env/BaseEnvironment.h"
# "src/env/BaseEnvironment.cpp"
# "inc/env/ContinuousEnvironment.h"
# "src/env/ContinuousEnvironment.cpp"
"inc/env/GridEnvironment.h"
"src/env/GridEnvironment.cpp"
# "inc/env/RoadmapEnvironment.h"
# "src/env/RoadmapEnvironment.cpp"

# "inc/io/ContinuousYaml.h"
# "src/io/ContinuousYaml.cpp"
"inc/io/GridLORR.h"
"src/io/GridLORR.cpp"
"inc/io/GridMovingAI.h"
"src/io/GridMovingAI.cpp"
"inc/io/GridStartGoal.h"
"inc/io/GridXML.h"
"src/io/GridXML.cpp"
# "inc/io/MetricsCsvObserver.h"
# "inc/io/RoadmapXML.h"
# "src/io/RoadmapXML.cpp"
# "inc/io/SimContinuousYamlOps.h"
# "src/io/SimContinuousYamlOps.cpp"
# "inc/io/StateCsvObserver.h"

"inc/planner/Planner.h"
# "inc/planner/plugin/plugin_api.h"
# "inc/planner/plugin/PlannerFactory.h"
# "src/planner/plugin/PlannerFactory.cpp"
# "inc/planner/plugin/PluginLoader.h"
# "src/planner/plugin/PluginLoader.cpp"
# "inc/planner/plugin/PluginOwnedPlanner.h"
# "src/planner/plugin/plugin_entry.cpp"

# "inc/robot/ActionSpace.h"
# "src/robot/ActionSpace.cpp"
# "inc/robot/Controller.h"
# "src/robot/Controller.cpp"
# "inc/robot/RobotSystem.h"
# "src/robot/RobotSystem.cpp"

# "inc/sim/GoalCheck.h"
# "src/sim/GoalCheck.cpp"
# "inc/sim/GoalCommon.h"
# "inc/sim/GridConflictAuditor.h"
# "src/sim/GridConflictAuditor.cpp"
# "inc/sim/ISimObserver.h"
# "inc/sim/PlanVerifier.h"
# "src/sim/PlanVerifier.cpp"
"inc/sim/SpawnRobot.h"
"src/sim/SpawnRobot.cpp"

# "inc/tools/MapConvert.h"
# "src/tools/MapConvert.cpp"

# "inc/types/Footprint.h"
"inc/types/GridCoords.h"

# "inc/util/AlgoFootprints.h"
# "inc/util/Box2dIdHash.h"
# "inc/util/TempDirGuard.h"
# "inc/util/Timer.h"
# "inc/util/Types.h"
# "inc/util/Logger.h"
# "src/util/Logger.cpp"
# "inc/util/Metrics.h"
# "src/util/Metrics.cpp"
# "inc/util/ProcRunner.h"
# "src/util/ProcRunner.cpp"
# "inc/util/ResultsDir.h"
# "src/util/ResultsDir.cpp"
)
condense=true

# Parse options
while getopts ":o:h" opt; do
  case "$opt" in
    o) OUT="$OPTARG" ;;
    h) usage ;;
    \?) echo "Invalid option: -$OPTARG" >&2; usage ;;
    :) echo "Option -$OPTARG requires an argument." >&2; usage ;;
  esac
done
shift $((OPTIND-1))

# If no CLI inputs, use the header-defined list
if [ "$#" -gt 0 ]; then
  inputs=("$@")
else
  inputs=("${DEFAULT_INPUTS[@]}")
fi

# If the desired output file is also one of the inputs, write to a temp file first
need_temp=false
for f in "${inputs[@]}"; do
  # Compare resolved paths to be safer
  if [ "$(realpath -m "$f")" = "$(realpath -m "$OUT")" ]; then
    need_temp=true
    break
  fi
done

if $need_temp; then
  tmp=$(mktemp --suffix=.tmp.concat) || tmp="/tmp/concat.$$"
  OUT_TMP="$tmp"
else
  OUT_TMP="$OUT"
fi

# Truncate/create the output temp file
: > "$OUT_TMP" || { echo "Cannot write to output '$OUT_TMP'." >&2; exit 1; }

index=1
for f in "${inputs[@]}"; do
  if [ ! -e "$f" ]; then
    echo "Warning: '$f' does not exist — skipping." >&2
    continue
  fi
  if [ -d "$f" ]; then
    echo "Warning: '$f' is a directory — skipping." >&2
    continue
  fi

  # Write header: (N) filename
  # Use printf to avoid surprises with filenames starting with '-'
  printf '(%d) %s\n\n' "$index" "$f" >> "$OUT_TMP"

  if $condense; then
    # Append file content with whole-line // comments and blank lines removed
    sed -e '/^[[:space:]]*\/\//d' \
        -e '/^[[:space:]]*$/d' -- "$f" >> "$OUT_TMP"
  else
    # Append file content; use cat -- to avoid treating filenames that start with '-' as options
    cat -- "$f" >> "$OUT_TMP"
  fi

  # Add two newlines to separate entries (keeps output neat)
  printf '\n\n' >> "$OUT_TMP"

  index=$((index+1))
done

# If we wrote to a temp file, move it to the requested output atomically
if $need_temp; then
  mv -- "$OUT_TMP" "$OUT"
fi

echo "Done: created '$OUT'"
