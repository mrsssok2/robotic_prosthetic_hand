#!/usr/bin/env python3
"""Print the markdown tables of docs/results from the raw JSON files (so the README numbers come from data, not memory).

    python3 scripts/summarize_results.py [docs/results]
"""
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else pathlib.Path(__file__).resolve().parents[1] / "docs" / "results")


def load(name):
    f = root / name
    return json.loads(f.read_text()) if f.exists() else None


def smoothing():
    jb, ja, lb, la = (load(n) for n in ("jitter_before.json", "jitter_after.json", "latency_before.json", "latency_after.json"))
    if not all((jb, ja, lb, la)):
        return
    print("### smoothing: EMA 0.3 + 4 curl/s  ->  One Euro + 10 curl/s\n")
    print("| metric | before (EMA) | after (One Euro) | change |\n|---|---|---|---|")

    def row(name, b, a, unit="", digits=1, lower_better=True):
        ch = 100 * (a - b) / b if b else 0.0
        print(f"| {name} | {b:.{digits}f}{unit} | {a:.{digits}f}{unit} | {ch:+.0f}% |")

    row("UDP -> /hand/state (median)", lb["bridge"]["median_ms"], la["bridge"]["median_ms"], " ms")
    row("UDP -> first joint command (median)", lb["command"]["median_ms"], la["command"]["median_ms"], " ms", 0)
    row("**UDP -> joint at 50% (median)**", lb["p50"]["median_ms"], la["p50"]["median_ms"], " ms", 0)
    row("UDP -> joint at 50% (p95)", lb["p50"]["p95_ms"], la["p50"]["p95_ms"], " ms", 0)
    row("UDP -> joint at 90% (median)", lb["p90"]["median_ms"], la["p90"]["median_ms"], " ms", 0)
    row(f"**jitter: std of the command, still hand + noise {jb['sigma']} (curl)**", jb["static"]["jitter_std"], ja["static"]["jitter_std"], "", 4)
    row("jitter: peak-to-peak (curl)", jb["static"]["peak_to_peak"], ja["static"]["peak_to_peak"], "", 3)
    for f in jb["tracking"]:
        row(f"tracking lag, {f} finger motion", jb["tracking"][f]["lag_ms"], ja["tracking"][f]["lag_ms"], " ms", 0)
    for f in jb["tracking"]:
        row(f"tracking amplitude kept, {f}", jb["tracking"][f]["gain"], ja["tracking"][f]["gain"], "", 2)
    print(f"\nlatency trials: before {lb['trials']}, after {la['trials']}\n")


def grasp():
    print("### grasp (5 trials per object)\n")
    for name in ("grasp_auto_only.json", "grasp.json"):
        rs = load(name)
        if not rs:
            continue
        print(f"{name}:\n\n| object | grip | success | first attempt | released | peak tip force (N) |\n|---|---|---|---|---|---|")
        for obj in dict.fromkeys(r["object"] for r in rs):
            sub = [r for r in rs if r["object"] == obj]
            grips = "/".join(sorted({r["grip"] for r in sub}))
            peak = max(max(r["peak_force_N"]) for r in sub)
            first = sum(r["success"] and r.get("attempts", 1) == 1 for r in sub)
            print(f"| {obj} | {grips} | {sum(r['success'] for r in sub)}/{len(sub)} | {first}/{len(sub)} | {sum(r['released'] for r in sub)}/{len(sub)} | {peak:.1f} |")
        print()


smoothing()
grasp()
