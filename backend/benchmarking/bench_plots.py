#!/usr/bin/env python3
# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

"""
Bench plotting utility for MAPF benchmarking.

Reads one or more session-level CSVs produced by benchmark.cpp (the "all_runs.csv" files)
with columns:
  session,map,scen,solver,takeK,rep,agents,plan_success,
  planner_runtime_s,sim_runtime_s,makespan_s,sum_of_cost_s,
  sum_length_m,avg_length_m,avg_completion_s,run_dir

Generates:
  - success_rate_by_[group].csv and .png/.svg
  - runtime/makespan/flowtime/length distributions (boxplots)
  - CDFs for planner_runtime_s and makespan_s
  - a compact LaTeX table summarizing key stats by solver (and optionally map)

Usage examples:
  python bench_plots.py --csv results/20250901_153000/all_runs.csv --out plots
  python bench_plots.py --csv runA.csv runB.csv --out plots --by solver map \
         --filter-success-only

Notes:
- We deliberately avoid seaborn and custom colors; single-figure-per-plot policy.
- Failed plans (plan_success==0) are excluded from metrics like makespan unless
  --include-failed is specified for a particular plot (see flags below).
"""

import argparse
import os
import sys
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# ----------------------------- Helpers -----------------------------

def _ensure_dir(p: Path) -> None:
    p.mkdir(parents=True, exist_ok=True)


def _safe_name(s: str) -> str:
    return (
        str(s)
        .replace("/", "_")
        .replace("\\", "_")
        .replace(",", "_")
        .replace(" ", "_")
    )


def _read_csvs(csv_paths):
    frames = []
    for p in csv_paths:
        df = pd.read_csv(p)
        df["source_csv"] = str(p)
        frames.append(df)
    if not frames:
        raise SystemExit("No CSVs loaded.")
    df = pd.concat(frames, ignore_index=True)
    # Ensure expected columns exist
    expected = {
        "session","map","scen","solver","takeK","rep","agents","plan_success",
        "planner_runtime_s","sim_runtime_s","makespan_s","sum_of_cost_s",
        "sum_length_m","avg_length_m","avg_completion_s","run_dir"
    }
    missing = expected.difference(df.columns)
    if missing:
        raise SystemExit(f"Missing columns in CSV: {sorted(missing)}")
    return df


def _group_stats(df: pd.DataFrame, by):
    # Aggregate typical stats with NaN-safe reductions
    def nan_iqr(x):
        q75, q25 = np.nanpercentile(x, 75), np.nanpercentile(x, 25)
        return q75 - q25

    gb = df.groupby(by, dropna=False)
    out = gb.agg(
        runs=("plan_success", "size"),
        success=("plan_success", "sum"),
        success_rate=("plan_success", "mean"),
        agents_mean=("agents", "mean"),
        planner_runtime_s_median=("planner_runtime_s", "median"),
        planner_runtime_s_mean=("planner_runtime_s", "mean"),
        makespan_s_median=("makespan_s", "median"),
        makespan_s_mean=("makespan_s", "mean"),
        sum_of_cost_s_median=("sum_of_cost_s", "median"),
        sum_length_m_median=("sum_length_m", "median"),
        avg_completion_s_median=("avg_completion_s", "median"),
        avg_length_m_median=("avg_length_m", "median"),
    )
    # Add dispersion for runtimes
    out["planner_runtime_s_iqr"] = (
        gb["planner_runtime_s"].apply(lambda x: nan_iqr(np.asarray(x, dtype=float)))
    )
    out.reset_index(inplace=True)
    return out


# ----------------------------- Plotters -----------------------------

def plot_success_rate(df: pd.DataFrame, by, outdir: Path, suffix: str = ""):
    stats = _group_stats(df, by)
    stats.sort_values("success_rate", ascending=False, inplace=True)

    fig, ax = plt.subplots(figsize=(8, 4.8))
    labels = ["|".join(map(str, row[by])) if isinstance(by, list) else str(row[by])
              for _, row in stats.iterrows()]
    ax.bar(range(len(stats)), stats["success_rate"].values)
    ax.set_xticks(range(len(stats)))
    ax.set_xticklabels(labels, rotation=45, ha="right")
    ax.set_ylabel("Success rate")
    ax.set_ylim(0, 1)
    ax.set_title(f"Success rate by {'+'.join(by) if isinstance(by, list) else by}")
    fig.tight_layout()

    stem = f"success_rate_by_{'_'.join(by if isinstance(by, list) else [by])}{suffix}"
    for ext in ("png", "svg"):
        fig.savefig(outdir / f"{stem}.{ext}", dpi=160)
    plt.close(fig)

    stats.to_csv(outdir / f"{stem}.csv", index=False)


def boxplot_metric(df: pd.DataFrame, metric: str, by: str, outdir: Path, title: str, suffix: str = ""):
    # We assume df already filtered for plan_success if desired
    order = sorted(df[by].dropna().unique())
    data = [df[df[by] == k][metric].dropna().values for k in order]

    fig, ax = plt.subplots(figsize=(8, 4.8))
    ax.boxplot(data, labels=[str(k) for k in order], showfliers=False)
    ax.set_xlabel(by)
    ax.set_ylabel(metric)
    ax.set_title(title)
    fig.tight_layout()

    stem = f"box_{metric}_by_{_safe_name(by)}{suffix}"
    for ext in ("png", "svg"):
        fig.savefig(outdir / f"{stem}.{ext}", dpi=160)
    plt.close(fig)


def cdf_metric(df: pd.DataFrame, metric: str, group: str, outdir: Path, title: str, suffix: str = ""):
    fig, ax = plt.subplots(figsize=(8, 4.8))
    for gval, sub in df.groupby(group, dropna=False):
        x = np.sort(sub[metric].dropna().values)
        if x.size == 0:
            continue
        y = np.linspace(0, 1, x.size)
        ax.plot(x, y, label=str(gval))
    ax.set_xlabel(metric)
    ax.set_ylabel("CDF")
    ax.set_title(title)
    ax.legend()
    fig.tight_layout()

    stem = f"cdf_{metric}_by_{_safe_name(group)}{suffix}"
    for ext in ("png", "svg"):
        fig.savefig(outdir / f"{stem}.{ext}", dpi=160)
    plt.close(fig)


# ----------------------------- LaTeX Table -----------------------------

def write_latex_table(df: pd.DataFrame, by, outdir: Path, filename: str = "summary_table.tex"):
    stats = _group_stats(df, by)
    cols = by + [
        "runs", "success", "success_rate",
        "planner_runtime_s_median", "makespan_s_median",
        "sum_length_m_median"
    ]
    stats = stats[cols]
    # Round for readability
    for c in ["success_rate", "planner_runtime_s_median", "makespan_s_median", "sum_length_m_median"]:
        stats[c] = stats[c].astype(float).round(3)

    # Build LaTeX (booktabs)
    header = "\\begin{tabular}{" + ("l" * len(by)) + "rrrrr}\\toprule\n"
    hnames = by + ["Runs", "Succ.", "Succ.\\%", "Med. Plan(s)", "Med. Make(s)", "Med. Len(m)"]
    lines = [" ".join(hnames) + "\\\\ \\midrule\n"]

    for _, row in stats.iterrows():
        cells = [str(row[k]) for k in by]
        cells += [
            f"{int(row['runs'])}",
            f"{int(row['success'])}",
            f"{row['success_rate']*100:.1f}",
            f"{row['planner_runtime_s_median']:.3f}",
            f"{row['makespan_s_median']:.3f}",
            f"{row['sum_length_m_median']:.3f}",
        ]
        lines.append(" & ".join(cells) + "\\\\\n")

    footer = "\\bottomrule\n\\end{tabular}\n"
    tex = header + "".join(lines) + footer

    out = outdir / filename
    with open(out, "w") as f:
        f.write(tex)


# ----------------------------- Main -----------------------------

# ----------------------------- Map Subplots (requested) -----------------------------

def scatter_subplots_by_map(df: pd.DataFrame, x: str, y: str, outdir: Path, suffix: str = ""):
    # Create a grid of subplots where each subplot corresponds to a distinct map
    if "map" not in df.columns:
        return
    maps = sorted(df["map"].dropna().unique())
    if len(maps) == 0:
        return

    # Arrange subplots in a roughly square grid (max 3 columns for readability)
    import math
    cols = min(3, max(1, len(maps)))
    rows = math.ceil(len(maps) / cols)

    fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 4 * rows), squeeze=False)

    for idx, m in enumerate(maps):
        r, c = divmod(idx, cols)
        ax = axes[r][c]
        sub = df[df["map"] == m][[x, y]].dropna()
        if not sub.empty:
            ax.scatter(sub[x].values, sub[y].values, s=12)
        ax.set_title(str(m))
        ax.set_xlabel(x)
        ax.set_ylabel(y)

    # Hide any unused axes
    total = rows * cols
    for k in range(len(maps), total):
        r, c = divmod(k, cols)
        axes[r][c].set_visible(False)

    fig.suptitle(f"{y} vs {x} by map")
    fig.tight_layout(rect=[0, 0, 1, 0.96])

    stem = f"subplot_{y}_vs_{x}_by_map{suffix}"
    for ext in ("png", "svg"):
        fig.savefig(outdir / f"{stem}.{ext}", dpi=160)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description="Plot benchmark results from all_runs.csv")
    p.add_argument("--csv", nargs="+", required=True, help="One or more all_runs.csv files")
    p.add_argument("--out", required=False, default="", help="Output directory for plots and summaries (optional if a single --csv is provided)")
    p.add_argument("--by", nargs="*", default=["solver"], help="Grouping keys for success-rate/table (e.g., solver map)")
    p.add_argument("--filter-success-only", action="store_true", help="Exclude plan_success==0 from distribution plots")
    p.add_argument("--suffix", default="", help="Suffix to append to output filenames (e.g., _k22)")
    p.add_argument("--per-map", action="store_true", help="Also produce per-map success and boxplots")
    p.add_argument("--map-subplots", action="store_true",
                   help="Create subplots by map for (sum_of_cost_s vs avg_completion_s) and (agents vs avg_completion_s)")

    args = p.parse_args()

    csvs = [Path(c) for c in args.csv]
    # Decide output directory: if --out given use it; else if a single CSV is provided,
    # place plots under <csv_dir>/plots as requested. If multiple CSVs and no --out, error.
    if args.out:
        outdir = Path(args.out)
    else:
        if len(csvs) == 1:
            outdir = csvs[0].parent / "plots"
        else:
            sys.stderr.write("[bench_plots] Please provide --out when using multiple --csv inputs.")
            sys.exit(2)
    _ensure_dir(outdir)

    csvs = [Path(c) for c in args.csv]
    df = _read_csvs(csvs)

    # Convert some columns to numeric just in case
    for col in ["planner_runtime_s", "sim_runtime_s", "makespan_s", "sum_of_cost_s",
                "sum_length_m", "avg_length_m", "avg_completion_s", "agents", "plan_success"]:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")

    # High-level success rate
    group_keys = args.by
    plot_success_rate(df, group_keys, outdir, suffix=args.suffix)

    # Distribution plots (optionally only successful runs)
    df_distr = df[df["plan_success"] == 1] if args.filter_success_only else df

    # Boxplots by solver (default) or by first grouping key
    group_for_box = group_keys[0] if group_keys else "solver"

    # Planner runtime boxplot
    boxplot_metric(df_distr, "planner_runtime_s", group_for_box, outdir,
                   title=f"Planner runtime by {group_for_box}", suffix=args.suffix)

    # Makespan boxplot
    boxplot_metric(df_distr, "makespan_s", group_for_box, outdir,
                   title=f"Makespan by {group_for_box}", suffix=args.suffix)

    # Flowtime (sum_of_cost_s) boxplot
    boxplot_metric(df_distr, "sum_of_cost_s", group_for_box, outdir,
                   title=f"Flowtime (sum of completion) by {group_for_box}", suffix=args.suffix)

    # Path length boxplot
    boxplot_metric(df_distr, "sum_length_m", group_for_box, outdir,
                   title=f"Total path length by {group_for_box}", suffix=args.suffix)

    # CDFs by solver
    cdf_metric(df_distr, "planner_runtime_s", group_for_box, outdir,
               title=f"CDF of planner runtime by {group_for_box}", suffix=args.suffix)
    cdf_metric(df_distr, "makespan_s", group_for_box, outdir,
               title=f"CDF of makespan by {group_for_box}", suffix=args.suffix)

    # LaTeX table
    write_latex_table(df, group_keys, outdir, filename=f"summary_by_{'_'.join(group_keys)}{args.suffix}.tex")

    # Optional: per-map breakdowns
    if args.per_map and "map" in df.columns:
        for m in sorted(df["map"].unique()):
            sub = df[df["map"] == m]
            stem = f"_{_safe_name(m)}{args.suffix}"
            plot_success_rate(sub, group_keys, outdir, suffix=stem)
            sub_distr = sub[sub["plan_success"] == 1] if args.filter_success_only else sub
            boxplot_metric(sub_distr, "planner_runtime_s", group_for_box, outdir,
                           title=f"Planner runtime by {group_for_box} (map={m})", suffix=stem)
            boxplot_metric(sub_distr, "makespan_s", group_for_box, outdir,
                           title=f"Makespan by {group_for_box} (map={m})", suffix=stem)

    # --- Requested subplots by map ---
    if args.map_subplots:
        # Respect the same success-only filtering choice used above
        df_for_sub = df[df["plan_success"] == 1] if args.filter_success_only else df
        scatter_subplots_by_map(df_for_sub, "sum_of_cost_s", "avg_completion_s", outdir, suffix=args.suffix)
        scatter_subplots_by_map(df_for_sub, "agents", "avg_completion_s", outdir, suffix=args.suffix)

    print(f"[bench_plots] Wrote outputs to: {outdir}")


if __name__ == "__main__":
    main()
