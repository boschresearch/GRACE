#!/usr/bin/env python3
# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

# Adapter: grid.txt + starts_xy.csv + targets_xy.csv  -> SCRIMP rollout -> paths.csv
import argparse, os, sys, csv
from pathlib import Path
import numpy as np
import torch
print(os.getcwd())
# --- SCRIMP repo imports ---
from model import Model
from alg_parameters import NetParameters, EnvParameters, SetupParameters
from episodic_buffer import EpisodicBuffer
from mapf_gym import MAPFEnv, State

PASSABLE = {'.', 'G'}  # treat '.' and 'G' as free

def read_grid_txt(path: Path):
    txt = path.read_text().splitlines()
    # normalize: keep non-empty, strip trailing
    rows = [ln.rstrip("\n") for ln in txt if ln.strip()]
    H = len(rows)
    W = max(len(r) for r in rows)
    # build world: -1 obstacle, 0 free
    world = -np.ones((H, W), dtype=np.int32)
    for i, row in enumerate(rows):
        for j, ch in enumerate(row):
            world[i, j] = 0 if ch in PASSABLE else -1
    return world

def read_xy_csv(p: Path):
    out = []
    with open(p, "r") as f:
        for line in f:
            if not line.strip(): continue
            xs, ys = line.strip().split(",")
            out.append((int(ys), int(xs)))  # (row=y, col=x)
    return out

def make_env_from_grid(world, starts_rc, goals_rc, k):
    H, W = world.shape
    world_agents = world.copy()
    goals = np.zeros_like(world_agents, dtype=np.int32)

    assert len(starts_rc) == k and len(goals_rc) == k, "starts/goals size mismatch"
    for a in range(k):
        r, c = starts_rc[a]
        gr, gc = goals_rc[a]
        assert world_agents[r, c] >= 0, f"start on obstacle for agent {a}"
        assert world_agents[gr, gc] >= 0, f"goal on obstacle for agent {a}"
        world_agents[r, c] = a + 1
        goals[gr, gc] = a + 1

    env = MAPFEnv(num_agents=k, size=H, prob=0.0)  # size/prob unused once we set world
    env.world = State(world_agents, goals, k)  # install fixed world/goals
    env.num_agents = k
    env.max_on_goal = 0
    return env

def compute_obs_vector(env, k):
    # Mirror what joint_step() does to construct obs + vector before first step
    obs = np.zeros((1, k, NetParameters.NUM_CHANNEL, EnvParameters.FOV_SIZE, EnvParameters.FOV_SIZE), dtype=np.float32)
    vec = np.zeros((1, k, NetParameters.VECTOR_LEN), dtype=np.float32)
    for i in range(k):
        state = env.observe(i + 1)
        obs[:, i, :, :, :] = state[0]
        vec[:, i, :3] = state[1]
    return obs, vec


def _to_cpu(x):
    if isinstance(x, torch.Tensor):
        return x.detach().to("cpu")
    if isinstance(x, (list, tuple)):
        return type(x)(_to_cpu(t) for t in x)
    return x

def _to_device(x, device):
    if isinstance(x, torch.Tensor):
        return x.to(device, non_blocking=False)
    if isinstance(x, (list, tuple)):
        return type(x)(_to_device(t, device) for t in x)
    return x


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--grid_file",    type=Path, required=True)
    ap.add_argument("--starts_xy",    type=Path, required=True)
    ap.add_argument("--targets_xy",   type=Path, required=True)
    ap.add_argument("--k",            type=int,  required=True)
    ap.add_argument("--outputPaths",  type=Path, required=True)

    ap.add_argument("--checkpoint",   type=Path, default=Path("final/net_checkpoint.pkl"))
    ap.add_argument("--device",       type=str,  default="cuda", choices=["cuda","cpu","mps"])
    ap.add_argument("--max_steps",    type=int,  default=256)
    ap.add_argument("--seed",         type=int,  default=SetupParameters.SEED)
    args = ap.parse_args()

    print("[scrimp] CWD:", os.getcwd())
    print("[scrimp] grid:", args.grid_file, "starts:", args.starts_xy, "targets:", args.targets_xy)

    torch.manual_seed(args.seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed(args.seed)
    torch.backends.cudnn.deterministic = True

    # Inputs
    world      = read_grid_txt(args.grid_file)
    starts_rc  = read_xy_csv(args.starts_xy)   # (r,c)
    goals_rc   = read_xy_csv(args.targets_xy)  # (r,c)
    k          = args.k

    # Env with fixed map/starts/goals
    env = make_env_from_grid(world, starts_rc, goals_rc, k)

    # ---------- Device selection ----------
    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested, but PyTorch cannot access a CUDA device")
    if args.device == "mps" and not torch.backends.mps.is_available():
        raise RuntimeError("MPS was requested, but PyTorch cannot access an MPS device")
    device = torch.device(args.device)
    print("[scrimp] device:", device)

    # (Optional) cap allocator growth a bit; safe to ignore if unsupported
    if device.type == "cuda" and hasattr(torch.cuda, "set_per_process_memory_fraction"):
        try:
            torch.cuda.set_per_process_memory_fraction(0.9, device=device)
        except Exception:
            pass

    # ---------- Model on chosen device ----------
    model  = Model(0, device)
    ckpt   = torch.load(args.checkpoint, map_location=device)
    model.network.load_state_dict(ckpt["model"])
    model.network.eval()

    # Make sure no grads are tracked globally for safety (redundant with inference_mode, but harmless)
    torch.set_grad_enabled(False)

    # ---------- CPU proxy for env (forces value() returns to CPU) ----------
    class _CPUValueProxy:
        def __init__(self, wrapped):
            self._m = wrapped
        def value(self, *a, **kw):
            out = self._m.value(*a, **kw)
            def _to_cpu(x):
                return x.detach().to("cpu") if isinstance(x, torch.Tensor) else x
            if isinstance(out, (tuple, list)):
                return tuple(_to_cpu(x) for x in out)
            return _to_cpu(out)

    model_for_env = _CPUValueProxy(model)

    # Episodic buffer
    episodic_buffer = EpisodicBuffer(2e6, k)
    episodic_buffer.reset(2e6, k)
    episodic_buffer.batch_add(env.get_positions())

    # t=0 observation
    obs, vector = compute_obs_vector(env, k)

    # ---------- Recurrent state + message (on model device!) ----------
    message = torch.zeros((1, k, NetParameters.NET_SIZE), device=device)
    hidden  = (torch.zeros((k, NetParameters.NET_SIZE // 2), device=device),
               torch.zeros((k, NetParameters.NET_SIZE // 2), device=device))

    # ---------- Rollout (GPU-safe) ----------
    done = False
    t = 0
    positions = []
    for a, (r, c) in enumerate(env.get_positions()):
        positions.append((a, 0, int(r), int(c)))

    # Use inference_mode to prevent graph creation & tensor versioning
    with torch.inference_mode():
        while not done and t < args.max_steps:
            # Model inference stays on the chosen device
            actions, hidden, v_all, ps, message = model.final_evaluate(
                obs, vector, hidden, message, k, greedy=False
            )

            # Step env with CPU tensors; keep the GPU versions for the next model call
            (obs, vector, rewards, done, _next_valid, on_goals, _blockings, _valids,
             _num_collide, _leave_goals, _num_on_goal, _max_on_goal, _num_collide2,
             _action_status, _modify_actions) = env.joint_step(
                actions, t, model_for_env, v_all, hidden, ps,
                no_reward=False, message=message, episodic_buffer=episodic_buffer
            )

            # Bookkeeping
            t += 1
            for a, (r, c) in enumerate(env.get_positions()):
                positions.append((a, t, int(r), int(c)))

            # (Optional) nudge the CUDA cache occasionally if you still see spikes
            if device.type == "cuda" and (t % 64 == 0):
                torch.cuda.empty_cache()


    # ---------- Write CSV ----------
    args.outputPaths.parent.mkdir(parents=True, exist_ok=True)
    with open(args.outputPaths, "w", newline="") as f:
        w = csv.writer(f)
        for a, tt, r, c in positions:
            w.writerow([a, tt, c, r])

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"[scrimp_driver] ERROR: {e}", file=sys.stderr)
        sys.exit(2)
