"""Trains the wheel policy in ml/sim.py with a simple CEM/evolution loop.

No torch, no RL library: the policy is the same 8-32-24-2 MLP that runs on
the robot, and CEM searches its weights directly. Starting from the current
int8 header is usually much faster than starting from random weights.

    python ml/train_rl.py --generations 20
    python ml/train_rl.py --init random --generations 40
    python ml/train_rl.py --no-write

By default it only replaces include/nn_policy.h if the learned policy beats
the supervised one on held-out simulator mazes. Use --force to write anyway.
"""

from __future__ import annotations

import argparse
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sim import FRONT_RANGE_MM, MazeEnv, make_maze
from train_policy import LAYERS, forward, init_net, quantise, write_header

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_HEADER = os.path.join(HERE, "..", "include", "nn_policy.h")
NUMBER_RE = r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?"


def pack(Ws, bs):
    # unpack() expects W1, b1, W2, b2, ... in order.
    parts = []
    for w, b in zip(Ws, bs):
        parts.append(w.ravel())
        parts.append(b.ravel())
    return np.concatenate(parts).astype(np.float32)


def unpack(theta):
    Ws, bs = [], []
    k = 0
    for i in range(len(LAYERS) - 1):
        n_out, n_in = LAYERS[i + 1], LAYERS[i]
        Ws.append(np.asarray(theta[k:k + n_out * n_in], np.float32).reshape(n_out, n_in))
        k += n_out * n_in
        bs.append(np.asarray(theta[k:k + n_out], np.float32).reshape(-1, 1))
        k += n_out
    return Ws, bs


def load_header(path):
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()

    def array(name):
        m = re.search(
            rf"static const (?:int8_t|float)\s+{name}\s*\[\s*\d+\s*\]\s*=\s*\{{(.*?)\}}\s*;",
            text,
            re.S,
        )
        if not m:
            raise ValueError(f"{name} not found in {path}")
        return np.array([float(v) for v in re.findall(NUMBER_RE, m.group(1))], np.float32)

    def scalar(name):
        m = re.search(rf"static const float\s+{name}\s*=\s*({NUMBER_RE})f\s*;", text)
        if not m:
            raise ValueError(f"{name} not found in {path}")
        return float(m.group(1))

    Ws, bs = [], []
    for i in range(len(LAYERS) - 1):
        q = array(f"nn_w{i + 1}")
        scale = scalar(f"nn_w{i + 1}_scale")
        b = array(f"nn_b{i + 1}")
        if q.size != LAYERS[i + 1] * LAYERS[i]:
            raise ValueError(f"nn_w{i + 1} has {q.size} values, expected {LAYERS[i + 1] * LAYERS[i]}")
        if b.size != LAYERS[i + 1]:
            raise ValueError(f"nn_b{i + 1} has {b.size} values, expected {LAYERS[i + 1]}")
        Ws.append(q.reshape(LAYERS[i + 1], LAYERS[i]) * scale)
        bs.append(b.reshape(-1, 1))
    return Ws, bs


def policy_action(obs, Ws, bs):
    x = np.asarray(obs, np.float32).reshape(1, -1)
    return np.clip(forward(x, Ws, bs)[-1][0], -1.0, 1.0)


def evaluate(theta, seeds, max_steps, sensor_noise, mismatch):
    Ws, bs = unpack(theta)
    total = 0.0
    steps = collisions = 0
    path = 0.0
    min_front = FRONT_RANGE_MM
    for seed in seeds:
        env = MazeEnv(
            maze=make_maze("mix", seed),
            seed=seed,
            max_steps=max_steps,
            sensor_noise=sensor_noise,
            mismatch=mismatch,
        )
        obs = env.reset(seed)
        while not env.done:
            obs, reward, _, info = env.step(policy_action(obs, Ws, bs))
            total += reward
        steps += info["steps"]
        collisions += int(info["collision"])
        path += info["path"]
        min_front = min(min_front, info["min_front"])
    n = len(seeds)
    return {
        "score": total / n,
        "steps": steps / n,
        "collisions": collisions,
        "path": path / n,
        "min_front": min_front,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--generations", type=int, default=25)
    ap.add_argument("--population", type=int, default=24)
    ap.add_argument("--elite", type=int, default=6)
    ap.add_argument("--episodes", type=int, default=3, help="mazes per candidate")
    ap.add_argument("--eval-episodes", type=int, default=6)
    ap.add_argument("--max-steps", type=int, default=600)
    ap.add_argument("--sigma", type=float, default=0.06)
    ap.add_argument("--sigma-floor", type=float, default=0.008)
    ap.add_argument("--blend", type=float, default=0.25)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--init", choices=("bc", "random"), default="bc")
    ap.add_argument("--init-header", default=DEFAULT_HEADER)
    ap.add_argument("--out", default=DEFAULT_HEADER)
    ap.add_argument("--mismatch", type=float, default=0.0)
    ap.add_argument("--no-sensor-noise", action="store_true")
    ap.add_argument("--no-write", action="store_true")
    ap.add_argument("--force", action="store_true")
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    if args.init == "bc":
        try:
            Ws, bs = load_header(args.init_header)
            print(f"starting from {args.init_header}", flush=True)
        except Exception as e:
            print(f"could not load header ({e}), starting random", flush=True)
            Ws, bs = init_net(rng)
    else:
        Ws, bs = init_net(rng)

    mean = pack(Ws, bs)
    sigma = np.full(mean.size, args.sigma, np.float32)
    val_seeds = [args.seed + 10000 + i for i in range(args.eval_episodes)]
    noise = not args.no_sensor_noise

    base = evaluate(mean, val_seeds, args.max_steps, noise, args.mismatch)
    print(
        f"baseline: reward {base['score']:7.2f} steps {base['steps']:5.0f} "
        f"collisions {base['collisions']} path {base['path']:6.0f}mm",
        flush=True,
    )

    best_theta = mean.copy()
    best = base
    for gen in range(args.generations):
        train_seeds = [args.seed + gen * 100 + i for i in range(args.episodes)]
        candidates = mean[None, :] + sigma[None, :] * rng.standard_normal(
            (args.population, mean.size)
        ).astype(np.float32)
        scores = np.array(
            [
                evaluate(c, train_seeds, args.max_steps, noise, args.mismatch)["score"]
                for c in candidates
            ],
            np.float32,
        )

        elite = candidates[np.argsort(scores)[-args.elite:]]
        new_mean = elite.mean(axis=0)
        new_sigma = elite.std(axis=0)
        mean = (args.blend * mean + (1.0 - args.blend) * new_mean).astype(np.float32)
        sigma = np.maximum(
            args.blend * sigma + (1.0 - args.blend) * new_sigma,
            args.sigma_floor,
        ).astype(np.float32)

        val = evaluate(mean, val_seeds, args.max_steps, noise, args.mismatch)
        if val["score"] > best["score"]:
            best = val
            best_theta = mean.copy()

        print(
            f"gen {gen + 1:02d}  train mean {scores.mean():7.2f}  "
            f"best {scores.max():7.2f}  val {val['score']:7.2f}  "
            f"sigma {sigma.mean():.4f}",
            flush=True,
        )

    print(
        f"best: reward {best['score']:7.2f} steps {best['steps']:5.0f} "
        f"collisions {best['collisions']} path {best['path']:6.0f}mm",
        flush=True,
    )

    if args.no_write:
        print("not writing header (--no-write)", flush=True)
        return
    if best["score"] < base["score"] and not args.force:
        print("learned policy did not beat the supervised one; keeping header", flush=True)
        print("use --force if you want to write it anyway", flush=True)
        return

    best_Ws, best_bs = unpack(best_theta)
    qWs, scales = [], []
    for W in best_Ws:
        q, s = quantise(W)
        qWs.append(q)
        scales.append(s)

    q_theta = pack([q.astype(np.float32) * s for q, s in zip(qWs, scales)], best_bs)
    q_eval = evaluate(q_theta, val_seeds, args.max_steps, noise, args.mismatch)
    print(
        f"int8: reward {q_eval['score']:7.2f} steps {q_eval['steps']:5.0f} "
        f"collisions {q_eval['collisions']} path {q_eval['path']:6.0f}mm",
        flush=True,
    )

    write_header(
        qWs,
        scales,
        best_bs,
        best["score"],
        metric="sim reward",
        note=f"CEM {args.generations} gens, pop {args.population}, {args.episodes} mazes/candidate",
        out_path=args.out,
    )


if __name__ == "__main__":
    main()
