"""Trains the small steering net that runs on the robot.

Teacher is a hand written wall follower. We sample a bunch of sensor
states around it (with noise), fit a tiny MLP, then quantise the weights
to int8 and write include/nn_policy.h. Pure numpy, no torch needed.

    python ml/train_policy.py
"""

import math
import os

import numpy as np

OUT_HEADER = os.path.join(os.path.dirname(__file__), "..", "include", "nn_policy.h")
LAYERS = [8, 32, 24, 2]
SEED = 7


def clip(x, lo, hi):
    return max(lo, min(hi, x))


def teacher(front, left, right, dleft, dright, gz, prev):
    """Returns (left wheel, right wheel) in [-1, 1].

    turn > 0 means turn right (left wheel faster). Gyro gz is positive
    when the robot rotates left (CCW).
    """
    if front <= 0:
        front = 1200.0
    if left <= 0:
        left = 800.0
    if right <= 0:
        right = 800.0

    base = 0.60
    turn = 0.0

    # hug whichever wall we can see, target ~210 mm
    if left < 650:
        err = (left - 210.0) / 210.0
        turn -= 0.55 * clip(err, -1, 1)
    elif right < 650:
        err = (right - 210.0) / 210.0
        turn += 0.55 * clip(err, -1, 1)

    if left < 95:
        turn += 0.70
    if right < 95:
        turn -= 0.70

    # front obstacle: head for the open side
    if front < 520:
        urgency = (520.0 - front) / 420.0
        free = clip((right - left) / 350.0, -1, 1)
        turn += 1.10 * urgency * free
        base *= 1.0 - 0.80 * urgency
    if front < 240:
        base -= 0.55 * (240.0 - front) / 200.0

    # wall closing in fast
    turn += clip(-dleft / 80.0, -1, 1) * 0.18
    turn -= clip(-dright / 80.0, -1, 1) * 0.18

    # damp the rotation we already have
    turn += clip(gz / 90.0, -1, 1) * 0.10

    base += 0.08 * prev

    return clip(base + turn, -1, 1), clip(base - turn, -1, 1)


def features(front, left, right, dleft, dright, gz, prev):
    return [
        front / 1200.0,
        left / 800.0,
        right / 800.0,
        clip((right - left) / 800.0, -1, 1),
        dleft / 150.0,
        dright / 150.0,
        gz / 120.0,
        prev,
    ]


def make_dataset(rng, n):
    X = np.zeros((n, 8), np.float32)
    Y = np.zeros((n, 2), np.float32)
    W = np.zeros(n, np.float32)

    for i in range(n):
        tight = rng.random() < 0.40
        if tight:
            front = rng.uniform(90, 360)
            left = rng.uniform(60, 260)
            right = rng.uniform(60, 260)
        else:
            front = 1200.0 if rng.random() < 0.15 else rng.uniform(120, 900)
            left = 800.0 if rng.random() < 0.15 else rng.uniform(70, 700)
            right = 800.0 if rng.random() < 0.15 else rng.uniform(70, 700)
        dleft = clip(rng.normal(0, 45), -150, 150)
        dright = clip(rng.normal(0, 45), -150, 150)
        gz = clip(rng.normal(0, 30), -120, 120)
        prev = rng.uniform(0, 0.8)

        # sensor noise
        front_n = max(60.0, front + rng.normal(0, 8))
        left_n = max(40.0, left + rng.normal(0, 5))
        right_n = max(40.0, right + rng.normal(0, 5))

        l, r = teacher(front_n, left_n, right_n, dleft, dright, gz, prev)
        l += rng.normal(0, 0.015)
        r += rng.normal(0, 0.015)

        X[i] = features(front_n, left_n, right_n, dleft, dright, gz, prev)
        Y[i] = [clip(l, -1, 1), clip(r, -1, 1)]
        W[i] = 5.0 if (front_n < 320 or left_n < 180 or right_n < 180) else 1.0

    return X, Y, W


def init_net(rng):
    Ws, bs = [], []
    for i in range(len(LAYERS) - 1):
        fan_in = LAYERS[i]
        W = rng.normal(0, 1.0 / math.sqrt(fan_in), (LAYERS[i + 1], fan_in)).astype(np.float32)
        b = np.zeros((LAYERS[i + 1], 1), np.float32)
        Ws.append(W)
        bs.append(b)
    return Ws, bs


def forward(X, Ws, bs):
    acts = [X]
    a = X
    for W, b in zip(Ws, bs):
        a = np.tanh(a @ W.T + b.T)
        acts.append(a)
    return acts


def train(X, Y, W, rng, epochs=750, batch=256, lr=1.2e-3):
    Ws, bs = init_net(rng)
    mW = [np.zeros_like(w) for w in Ws]
    vW = [np.zeros_like(w) for w in Ws]
    mb = [np.zeros_like(b) for b in bs]
    vb = [np.zeros_like(b) for b in bs]
    b1, b2, eps = 0.9, 0.999, 1e-8
    step = 0

    for ep in range(epochs):
        idx = rng.permutation(len(X))
        total = 0.0
        for s in range(0, len(X), batch):
            xb = X[idx[s:s + batch]]
            yb = Y[idx[s:s + batch]]
            wb = W[idx[s:s + batch]].reshape(-1, 1)
            acts = forward(xb, Ws, bs)
            err = acts[-1] - yb
            total += float((err ** 2 * wb).sum() / wb.sum()) * len(xb)

            grad = 2.0 * err * wb / wb.sum()
            for l in range(len(Ws) - 1, -1, -1):
                gW = grad.T @ acts[l]
                gb = grad.sum(axis=0).reshape(-1, 1)
                grad = (grad @ Ws[l]) * (1.0 - acts[l] ** 2)

                step += 1
                mW[l] = b1 * mW[l] + (1 - b1) * gW
                vW[l] = b2 * vW[l] + (1 - b2) * (gW ** 2)
                mb[l] = b1 * mb[l] + (1 - b1) * gb
                vb[l] = b2 * vb[l] + (1 - b2) * (gb ** 2)
                mh = mW[l] / (1 - b1 ** step)
                vh = vW[l] / (1 - b2 ** step)
                Ws[l] -= lr * mh / (np.sqrt(vh) + eps)
                mh = mb[l] / (1 - b1 ** step)
                vh = vb[l] / (1 - b2 ** step)
                bs[l] -= lr * mh / (np.sqrt(vh) + eps)

        lr *= 0.995
        if ep % 60 == 0 or ep == epochs - 1:
            print(f"epoch {ep:3d}  train mse {total / len(X):.5f}")

    return Ws, bs


def quantise(W):
    scale = float(np.abs(W).max()) / 127.0
    if scale == 0:
        scale = 1.0
    q = np.clip(np.round(W / scale), -127, 127).astype(np.int8)
    return q, scale


def infer_q(x, qWs, scales, bs):
    # int8 weights, float activations (what the ESP32 build does)
    a = x.astype(np.float32)
    for qW, sw, b in zip(qWs, scales, bs):
        a = np.tanh((qW.astype(np.float32) * sw) @ a + b.ravel())
    return a


def fmt_arr(name, arr, ctype):
    flat = arr.ravel()
    lines = [f"static const {ctype} {name}[{flat.size}] = {{"]
    for i in range(0, flat.size, 16):
        chunk = ", ".join(str(int(v)) if ctype.startswith("int") else f"{v:.8e}f" for v in flat[i:i + 16])
        lines.append("  " + chunk + ",")
    lines.append("};")
    return "\n".join(lines)


def write_header(qWs, scales, bs, val_mse):
    out = [
        "// generated by ml/train_policy.py - do not edit by hand",
        "#pragma once",
        "#include <stdint.h>",
        "",
        f"#define NN_IN {LAYERS[0]}",
        f"#define NN_H1 {LAYERS[1]}",
        f"#define NN_H2 {LAYERS[2]}",
        f"#define NN_OUT {LAYERS[3]}",
        "",
        "void nnPolicy(const float *in, float *out);",
        "",
    ]
    names = ["nn_w1", "nn_w2", "nn_w3"]
    bnames = ["nn_b1", "nn_b2", "nn_b3"]
    snames = ["nn_w1_scale", "nn_w2_scale", "nn_w3_scale"]
    for i, (qW, sw, b) in enumerate(zip(qWs, scales, bs)):
        out.append(fmt_arr(names[i], qW, "int8_t"))
        out.append("")
        out.append(f"static const float {snames[i]} = {sw:.8e}f;")
        out.append(fmt_arr(bnames[i], b, "float"))
        out.append("")
    out.append(f"// float val mse: {val_mse:.6f}")
    out.append("")
    with open(OUT_HEADER, "w", encoding="utf-8") as f:
        f.write("\n".join(out))
    print(f"wrote {os.path.abspath(OUT_HEADER)}")


def main():
    rng = np.random.default_rng(SEED)
    X, Y, W = make_dataset(rng, 32000)
    perm = rng.permutation(len(X))
    nval = len(X) // 10
    val, train_idx = perm[:nval], perm[nval:]

    print(f"data {len(X)} samples, {LAYERS} net")
    Ws, bs = train(X[train_idx], Y[train_idx], W[train_idx], rng)

    pv = forward(X[val], Ws, bs)[-1]
    val_mse = float(((pv - Y[val]) ** 2).mean())
    mae = np.abs(pv - Y[val]).mean(axis=0)
    print(f"val mse {val_mse:.6f}  mae L {mae[0]:.4f} R {mae[1]:.4f}")

    qWs, scales = [], []
    for W in Ws:
        q, s = quantise(W)
        qWs.append(q)
        scales.append(s)
    qp = np.stack([infer_q(x, qWs, scales, bs) for x in X[val]])
    qmse = float(((qp - pv) ** 2).mean())
    print(f"int8 vs float mse {qmse:.6f}")

    for name, raw in [
        ("open", (1200, 700, 700, 0, 0, 0, 0.5)),
        ("left wall 120", (900, 120, 700, 0, 0, 0, 0.5)),
        ("front wall 180", (180, 400, 300, 0, 0, 0, 0.5)),
        ("boxed in", (130, 100, 110, 0, 0, 0, 0.5)),
    ]:
        x = np.array([features(*raw)], np.float32)
        f = forward(x, Ws, bs)[-1][0]
        q = infer_q(x[0], qWs, scales, bs)
        print(f"{name:16s} float {f[0]:+.2f} {f[1]:+.2f}   int8 {q[0]:+.2f} {q[1]:+.2f}")

    write_header(qWs, scales, bs, val_mse)


if __name__ == "__main__":
    main()
