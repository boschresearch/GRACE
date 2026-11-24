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
# "input/Input.h"
# "input/Input.cpp"
"platform/PlannerCatalog.h"
"platform/PlannerCatalog.cpp"
"platform/PlatformBase.h"
"platform/PlatformCore.cpp"
"platform/PlatformDefaults.h"
"platform/PlatformEditors.cpp"
"platform/PlatformOverlays.cpp"
"platform/PlatformPlanning.cpp"
# "render/ColorUtil.h"
# "render/DebugConfig.h"
# "render/DebugConfig.cpp"
# "render/DebugDrawQueue.h"
# "render/DebugDrawQueue.cpp"
# "render/RendererSfml.h"
# "render/RendererSfml.cpp"
"sample/Sample.h"
"sample/Sample.cpp"
# "sample/SampleContext.h"
# "sample/SampleContext.cpp"
# "sample/SampleRegistry.h"
# "sample/SampleRegistry.cpp"
# "sample/SampleTasks.h"
# "sample/SampleTasks.cpp"
"ui/panels/AgentsPanel.cpp"
"ui/panels/Draw.cpp"
"ui/panels/EnvironmentPanel.cpp"
"ui/panels/MetricsCompareUI.h"
"ui/panels/MetricsCompareUI.cpp"
"ui/panels/MetricsUI.h"
"ui/panels/MetricsUI.cpp"
"ui/panels/PlannerPanel.cpp"
"ui/panels/SimulationPanel.cpp"
# "ui/panels/UiWidgets.h"
# "ui/panels/UiWidgets.cpp"
# "ui/panels/UiWorkflow.h"
# "ui/panels/UiWorkflow.cpp"
# "ui/widgets/FilePicker.h"
# "ui/widgets/FilePicker.cpp"
# "ui/UiOverlays.h"
# "ui/UiOverlays.cpp"
# "ui/UiStyle.h"
# "ui/UiStyle.cpp"
# "ui/UiTools.h"
# "ui/UiTools.cpp"
# "ui/ViewController.h"
# "ui/ViewController.cpp"
"main.cpp"
"Sample_MAPF_Continuous.cpp"
"Sample_MAPF_CrossCompare.cpp"
"Sample_MAPF_Grid.cpp"
"Sample_MAPF_Roadmap.cpp"
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
