#!/usr/bin/env python3
# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

# driver.py
# Adapter: MovingAI .map + .scen  ->  POGEMA env  -> MAPF-GPT inference -> CSV paths
import argparse
from pathlib import Path
import sys
import csv
import os
import torch
# --- MAPF-GPT / POGEMA deps ---
from pogema_toolbox.create_env import Environment
from pogema_toolbox.run_episode import run_episode
from pogema_toolbox.registry import ToolboxRegistry

from create_env import create_eval_env
from gpt.inference import MAPFGPTInference, MAPFGPTInferenceConfig

PASSABLE = set(['.', 'G'])  # MovingAI free cells commonly '.' or 'G'


def read_grid_txt(path: Path) -> str:
    return Path(path).read_text()

def read_xy_csv(path: Path):
    xy = []
    with open(path, "r") as f:
        for line in f:
            if not line.strip():
                continue
            x_str, y_str = line.strip().split(",")
            xy.append([int(y_str), int(x_str)]) # platform uses (x=col, y=row), planner uses the opposite
    return xy

def read_movingai_map(path: Path) -> str:
    """Return POGEMA grid string using '.' for free, '#' for obstacle."""
    header_done = False
    rows = []
    with open(path, 'r') as f:
        for line in f:
            line = line.rstrip('\n')
            if not header_done:
                if line.strip().lower() == 'map':
                    header_done = True
                continue
            # map rows
            row = ''.join('.' if ch in PASSABLE else '#' for ch in line)
            rows.append(row)
    if not rows:
        raise RuntimeError("Empty MAP file payload")
    return "\n".join(rows)

def infer_size_from_grid(grid_str: str) -> int:
    # keep only non-empty lines
    lines = [ln for ln in grid_str.splitlines() if ln.strip()]
    if not lines:
        raise ValueError("grid_str is empty")
    rows = len(lines)
    cols = max(len(ln) for ln in lines)
    return max(rows, cols)

def read_scen(path: Path, k: int):
    """Return lists of (row,col) starts/goals (0-based), reading first K entries."""
    starts, goals = [], []
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('version'):
                continue
            parts = line.split()
            # Standard MovingAI .scen is tab-separated; be robust
            if len(parts) < 9:
                parts = [p for chunk in line.split('\t') for p in chunk.split()]
            if len(parts) < 9:
                continue
            # fields: bucket map width height sx sy gx gy dist
            sx, sy, gx, gy = int(parts[4]), int(parts[5]), int(parts[6]), int(parts[7])
            # MovingAI uses (x=col, y=row)
            starts.append((sy, sx))
            goals.append((gy, gx))
            if len(starts) >= k:
                break
    if len(starts) < k:
        raise RuntimeError(f"Scenario has only {len(starts)} entries, need {k}")
    return starts, goals

def main():
    ap = argparse.ArgumentParser()

    # use progma style maps
    ap.add_argument("--grid_file", type=Path)
    ap.add_argument("--starts_xy", type=Path)
    ap.add_argument("--targets_xy", type=Path)

    # if use moving ai maps
    ap.add_argument("--map", type=Path, default=None)
    ap.add_argument("--scen", type=Path, default=None)

    ap.add_argument("--k", required=True, type=int)
    ap.add_argument("--outputPaths", required=True, type=Path)
    ap.add_argument("--model", default="2M-DDG",
                    choices=["2M", "6M", "85M", "2M-DDG"])
    ap.add_argument("--device", default="cuda", choices=["cuda", "cpu", "mps"])
    ap.add_argument("--max_steps", type=int, default=256)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--svg", type=Path, default=None)
    args = ap.parse_args()

    print("[driver] argv:", sys.argv)
    print("[driver] CWD:", os.getcwd())
    print("[driver] exists(grid):", os.path.exists(str(args.grid_file)) if args.grid_file else None)
    print("[driver] exists(starts):", os.path.exists(str(args.starts_xy)) if args.starts_xy else None)
    print("[driver] exists(targets):", os.path.exists(str(args.targets_xy)) if args.targets_xy else None)
    print("[driver] k:", args.k, "device:", args.device, "model:", args.model)

    # Torch seeding
    torch.manual_seed(args.seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed(args.seed)
    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested, but PyTorch cannot access a CUDA device")
    if args.device == "mps" and not torch.backends.mps.is_available():
        raise RuntimeError("MPS was requested, but PyTorch cannot access an MPS device")
    torch.backends.cudnn.deterministic = True
    torch.set_grad_enabled(False)

    if args.grid_file and args.starts_xy and args.targets_xy:
        grid_str = read_grid_txt(args.grid_file)
        agents_xy  = read_xy_csv(args.starts_xy)
        targets_xy = read_xy_csv(args.targets_xy)
        assert len(agents_xy) == args.k and len(targets_xy) == args.k

    else:
        grid_str = read_movingai_map(args.map)
        starts_rc, goals_rc = read_scen(args.scen, args.k)

        agents_xy  = [[c, r] for (r, c) in starts_rc]
        targets_xy = [[c, r] for (r, c) in goals_rc]

    cfg = Environment(
        map=grid_str,
        num_agents=args.k,
        size=infer_size_from_grid(grid_str),
        agents_xy=agents_xy,
        targets_xy=targets_xy,

        observation_type="MAPF",
        on_target="nothing",
        max_episode_steps=args.max_steps,
        seed=args.seed,
        obs_radius=5,
        collision_system="soft",
    )
    env = create_eval_env(cfg)
    obs, info = env.reset()

    algo = MAPFGPTInference(
        MAPFGPTInferenceConfig(path_to_weights=f"hf_weights/model-{args.model}.pt",
                               device=args.device)
    )
    algo.reset_states()

    positions = []  # (agent, t, row, col)
    t = 0
    data = env.get_agents_xy(ignore_borders=True)
    for a, (y, x) in enumerate(data):
        positions.append((a, 0, int(y), int(x)))

    with torch.inference_mode():
        while True:

            actions = algo.act(obs)
            obs, reward, terminated, truncated, info = env.step(actions)
            t += 1

            data = env.get_agents_xy(ignore_borders=True)
            for a, (y, x) in enumerate(data):
                positions.append((a, t, int(y), int(x)))
            # terminated/truncated can be lists/arrays per-agent
            all_term = hasattr(terminated, "__iter__") and all(terminated)
            all_trunc = hasattr(truncated, "__iter__") and all(truncated)
            if all_term or all_trunc or t >= args.max_steps:
                break

    # write CSV
    args.outputPaths.parent.mkdir(parents=True, exist_ok=True)
    with open(args.outputPaths, "w", newline="") as f:
        w = csv.writer(f)
        for a, tt, r, c in positions:
            w.writerow([a, tt, c, r])  # in csv we use first c then r, then from platform it is naturally x y

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"[mapfgptddg_driver] ERROR: {e}", file=sys.stderr)
        sys.exit(2)
