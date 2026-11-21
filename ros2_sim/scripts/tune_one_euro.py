#!/usr/bin/env python3
"""Offline tuning of the One Euro parameters: replays the same synthetic finger signals as measure_jitter.py (30 Hz noisy
packets, 50 Hz controller ticks, same rate limiter) through the shaping code and prints jitter / lag per setting.
Pure numpy (no ROS, no Gazebo), so a grid of settings runs in seconds.

    python3 scripts/tune_one_euro.py            # grid (rate limit 10 curl/s) next to the old EMA setting
    python3 scripts/tune_one_euro.py --ema      # the old EMA 0.3 for reference
"""
import argparse
import itertools
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "src" / "hand_controller"))
from hand_controller.shaping import OneEuroFilter, Shaper  # noqa: E402

RATE, TICK, SIGMA, SECONDS = 30.0, 50.0, 0.03, 12.0


def simulate(signal, noise, euro=None, alpha=1.0, max_rate=4.0, seconds=SECONDS, start=0.5):
    """-> (t_ticks, command) of one finger, input = signal(t) + noise at RATE Hz, state initialised at `start`."""
    flt = OneEuroFilter(1, **euro, initial=start) if euro else None
    sh = Shaper(1, alpha=alpha, max_rate=max_rate, initial=start)
    held, nxt, ts, out = np.array([start]), 0.0, [], []
    for k in range(int(seconds * TICK)):
        t = k / TICK
        while t >= nxt:                                    # a frame arrives
            x = signal(nxt) + noise[int(nxt * RATE) % len(noise)]
            held = flt(np.array([np.clip(x, 0, 1)]), 1.0 / RATE) if flt else np.array([np.clip(x, 0, 1)])
            nxt += 1.0 / RATE
        ts.append(t)
        out.append(sh.step(held, 1.0 / TICK)[0])
    return np.array(ts), np.array(out)


def metrics(**kw):
    rng = np.random.default_rng(0)
    noise = rng.normal(0.0, SIGMA, int(SECONDS * RATE * 2))
    ts, c = simulate(lambda t: 0.5, noise, **kw)
    gain = c[ts > 2].std() / SIGMA
    lags, errs = {}, {}
    for f in (0.5, 1.0, 2.0):
        clean = lambda t, f=f: 0.5 + 0.35 * np.sin(2 * np.pi * f * t)
        ts, c = simulate(clean, noise, **kw)
        t = np.arange(2.0, SECONDS - 0.5, 0.005)
        ref = clean(t)
        ds = np.arange(0.0, 0.4, 0.005)
        e = [np.sqrt(np.mean((np.interp(t + d, ts, c) - ref) ** 2)) for d in ds]
        lags[f], errs[f] = ds[int(np.argmin(e))] * 1000, min(e)
    # step 0 -> 1: time to 50 % / 90 % of the command
    st, sc = simulate(lambda t: 0.0 if t < 1.0 else 1.0, np.zeros(1000), start=0.0, seconds=3.0, **kw)
    t50 = st[np.argmax(sc >= 0.5)] - 1.0
    t90 = st[np.argmax(sc >= 0.9)] - 1.0
    return dict(gain=gain, lag05=lags[0.5], lag1=lags[1.0], lag2=lags[2.0], err1=errs[1.0], err2=errs[2.0],
                t50=t50 * 1000, t90=t90 * 1000)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ema", action="store_true")
    a = ap.parse_args()
    rows = []
    if a.ema:
        rows.append(("ema 0.3 rate 4", metrics(alpha=0.3, max_rate=4.0)))
    else:
        rows.append(("ema 0.3 rate 4 (before)", metrics(alpha=0.3, max_rate=4.0)))
        for mc, beta, mr in itertools.product((0.3, 0.5, 0.8, 1.0, 1.5), (1.0, 2.0, 2.5, 3.0, 4.0), (10.0,)):
            rows.append((f"min_cutoff {mc} beta {beta} rate {mr}", metrics(euro=dict(min_cutoff=mc, beta=beta, d_cutoff=1.0), alpha=1.0, max_rate=mr)))
    print(f"{'setting':42s} {'noise_gain':>10s} {'lag@.5':>7s} {'lag@1':>6s} {'lag@2':>6s} {'err@1':>6s} {'err@2':>6s} {'t50ms':>6s} {'t90ms':>6s}")
    for name, m in rows:
        print(f"{name:42s} {m['gain']:10.3f} {m['lag05']:7.0f} {m['lag1']:6.0f} {m['lag2']:6.0f} {m['err1']:6.3f} {m['err2']:6.3f} {m['t50']:6.0f} {m['t90']:6.0f}")


if __name__ == "__main__":
    main()
