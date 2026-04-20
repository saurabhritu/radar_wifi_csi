#!/usr/bin/env python3
"""Analyze CSI session CSVs to understand score distributions by presence state."""

import csv
import os
import sys
from collections import defaultdict

SESSION_DIR = "/Users/saurabhritu/Code/radar_wifi_csi"
FILES = sorted([
    os.path.join(SESSION_DIR, f)
    for f in os.listdir(SESSION_DIR)
    if f.startswith("session") and f.endswith(".csv")
])

# Fields we care about
SCORE_FIELDS = [
    "occupancy_score", "micromotion_score", "motion_score", "stability_score",
    "occupancy_energy", "motion_avg_energy", "motion_energy",
    "occupancy_ref", "motion_ref",
    "motion_enter", "motion_clear",
    "micro_enter", "micro_clear",
    "presence_enter", "presence_clear",
]

# Additional fields for newer sessions
EXTRA_FIELDS = [
    "selected_shift_score", "selected_macro_score", "selected_micro_score",
    "hybrid_presence_state",
    "macro_frames", "micro_frames", "static_frames", "clear_frames",
    "presence_hold_frames",
]

def parse_row(header, row_parts):
    """Parse a CSV row into a dict."""
    d = {}
    for i, col in enumerate(header):
        if i < len(row_parts):
            d[col] = row_parts[i]
    return d

def safe_float(val):
    try:
        return float(val)
    except (ValueError, TypeError):
        return None

def analyze_file(filepath):
    """Analyze one session file."""
    basename = os.path.basename(filepath)
    
    # Stats per presence_state
    stats = defaultdict(lambda: defaultdict(list))
    total_rows = 0
    state_counts = defaultdict(int)
    annotations = defaultdict(int)
    threshold_set = set()
    
    with open(filepath, 'r') as f:
        # Read header
        header_line = f.readline().strip()
        if header_line.startswith('\xef\xbb\xbf'):
            header_line = header_line[3:]
        header_line = header_line.replace('\r', '')
        
        header = header_line.split(',')
        # Remove "CSI_DATA_HEADER" prefix
        if header[0] == "CSI_DATA_HEADER":
            header = header[1:]
        
        for line in f:
            line = line.strip().replace('\r', '')
            if not line or line.startswith("CSI_DATA_HEADER"):
                continue
            
            parts = line.split(',')
            # Remove "CSI_DATA" prefix
            if parts[0] == "CSI_DATA":
                parts = parts[1:]
            
            row = parse_row(header, parts)
            
            # Skip rows during calibration
            baseline = row.get("baseline_ready", "1")
            if baseline == "0":
                continue
            
            detection = row.get("detection_enabled", "1")
            if detection == "0":
                continue
            
            state = row.get("presence_state", "unknown")
            annotation = row.get("manual_annotation", "none")
            
            total_rows += 1
            state_counts[state] += 1
            if annotation != "none":
                annotations[annotation] += 1
            
            for field in SCORE_FIELDS + EXTRA_FIELDS:
                val = safe_float(row.get(field))
                if val is not None:
                    stats[state][field].append(val)
            
            # Collect threshold values
            me = safe_float(row.get("motion_enter"))
            if me is not None:
                threshold_set.add(f"motion_enter={me:.3f}")
    
    return basename, total_rows, state_counts, stats, annotations, threshold_set

def percentiles(data):
    """Return min, p25, median, p75, p95, max."""
    if not data:
        return (0, 0, 0, 0, 0, 0)
    s = sorted(data)
    n = len(s)
    return (
        s[0],
        s[int(n * 0.25)],
        s[int(n * 0.50)],
        s[int(n * 0.75)],
        s[int(n * 0.95)] if n > 20 else s[-1],
        s[-1],
    )

print("=" * 100)
print("CSI SESSION ANALYSIS — Score Distributions by Presence State")
print("=" * 100)

for filepath in FILES:
    basename, total, state_counts, stats, annotations, thresh = analyze_file(filepath)
    
    print(f"\n{'─' * 100}")
    print(f"📁 {basename} — {total} rows post-calibration")
    print(f"   States: {dict(state_counts)}")
    if annotations:
        print(f"   Annotations: {dict(annotations)}")
    if thresh:
        print(f"   Thresholds seen: {thresh}")
    
    # Only show detailed stats for files with substantial data
    if total < 50:
        print(f"   (Too few rows for detailed analysis)")
        continue
    
    key_fields = ["occupancy_score", "micromotion_score", "motion_score", "stability_score"]
    extra_key = ["selected_shift_score", "selected_macro_score", "selected_micro_score"]
    threshold_fields = ["motion_enter", "motion_clear", "micro_enter", "micro_clear", "presence_enter", "presence_clear"]
    
    # Show thresholds once
    for state in sorted(stats.keys()):
        if stats[state].get("motion_enter"):
            vals = stats[state]["motion_enter"]
            print(f"\n   Thresholds (from {state}): motion_enter={vals[0]:.3f}, ", end="")
            for tf in threshold_fields[1:]:
                v = stats[state].get(tf, [0])
                print(f"{tf}={v[0]:.3f}", end=", ")
            print()
            break
    
    for state in sorted(stats.keys()):
        n = state_counts.get(state, 0)
        if n < 5:
            continue
        
        print(f"\n   ┌── {state.upper()} ({n} rows, {n*100//total}%)")
        print(f"   │  {'Field':<25s} {'Min':>7s} {'P25':>7s} {'Med':>7s} {'P75':>7s} {'P95':>7s} {'Max':>7s}")
        print(f"   │  {'─'*25} {'─'*7} {'─'*7} {'─'*7} {'─'*7} {'─'*7} {'─'*7}")
        
        for field in key_fields:
            data = stats[state].get(field, [])
            if not data:
                continue
            p = percentiles(data)
            print(f"   │  {field:<25s} {p[0]:7.3f} {p[1]:7.3f} {p[2]:7.3f} {p[3]:7.3f} {p[4]:7.3f} {p[5]:7.3f}")
        
        for field in extra_key:
            data = stats[state].get(field, [])
            if not data:
                continue
            p = percentiles(data)
            print(f"   │  {field:<25s} {p[0]:7.3f} {p[1]:7.3f} {p[2]:7.3f} {p[3]:7.3f} {p[4]:7.3f} {p[5]:7.3f}")
        
        # Frame counters
        for field in ["macro_frames", "micro_frames", "static_frames", "clear_frames", "presence_hold_frames"]:
            data = stats[state].get(field, [])
            if not data:
                continue
            p = percentiles(data)
            print(f"   │  {field:<25s} {p[0]:7.0f} {p[1]:7.0f} {p[2]:7.0f} {p[3]:7.0f} {p[4]:7.0f} {p[5]:7.0f}")
        
        print(f"   └──")

# Cross-session summary of key score ranges
print(f"\n{'=' * 100}")
print("CROSS-SESSION SUMMARY — Score ranges across ALL sessions")
print("=" * 100)

all_stats = defaultdict(lambda: defaultdict(list))
all_state_counts = defaultdict(int)

for filepath in FILES:
    _, _, sc, stats, _, _ = analyze_file(filepath)
    for state, cnt in sc.items():
        all_state_counts[state] += cnt
    for state, fields in stats.items():
        for field, vals in fields.items():
            all_stats[state][field].extend(vals)

total_all = sum(all_state_counts.values())
print(f"\nTotal rows: {total_all}")
print(f"State distribution: {dict(all_state_counts)}")
if total_all > 0:
    for state in sorted(all_state_counts.keys()):
        pct = all_state_counts[state] * 100 // total_all
        print(f"  {state}: {all_state_counts[state]} ({pct}%)")

key_fields = ["occupancy_score", "micromotion_score", "motion_score", "stability_score"]

for state in sorted(all_stats.keys()):
    n = all_state_counts.get(state, 0)
    if n < 10:
        continue
    print(f"\n┌── {state.upper()} ({n} rows)")
    print(f"│  {'Field':<25s} {'Min':>7s} {'P25':>7s} {'Med':>7s} {'P75':>7s} {'P95':>7s} {'Max':>7s}")
    print(f"│  {'─'*25} {'─'*7} {'─'*7} {'─'*7} {'─'*7} {'─'*7} {'─'*7}")
    for field in key_fields:
        data = all_stats[state].get(field, [])
        if not data:
            continue
        p = percentiles(data)
        print(f"│  {field:<25s} {p[0]:7.3f} {p[1]:7.3f} {p[2]:7.3f} {p[3]:7.3f} {p[4]:7.3f} {p[5]:7.3f}")
    print(f"└──")

print(f"\n{'=' * 100}")
print("KEY INSIGHT: Compare motion_score ranges to motion_enter threshold")
print("=" * 100)
for state in ["no_presence", "presence_motion", "presence_micromotion", "presence_static"]:
    motion = all_stats.get(state, {}).get("motion_score", [])
    me = all_stats.get(state, {}).get("motion_enter", [])
    if motion:
        p = percentiles(motion)
        thresh_val = me[0] if me else "?"
        print(f"  {state:25s}: motion_score median={p[2]:.3f}, P95={p[4]:.3f}, max={p[5]:.3f} | threshold={thresh_val}")
