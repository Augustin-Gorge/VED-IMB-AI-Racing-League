import csv
import math
from pathlib import Path
import re
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import autopilot as ap


def make_controller(speed_ms=28.0):
    return SimpleNamespace(
        state=ap.STATE_STRAIGHT,
        turn_sign=0.0,
        phase_timer=0,
        phase_duration=1,
        prev_max_dist=120.0,
        approach_entry_pos=0.0,
        target_pos_filtered=0.0,
        last_speed_ms=speed_ms,
        last_track_pos=0.0,
        off_track_timer=0,
        recovery_state="NONE",
        exit_arc_m=20.0,
        yaw_f=0.0,
        prev_steer=0.0,
    )


def parse_corkscrew_segments(xml_path=None):
    if xml_path is None:
        xml_path = Path(__file__).resolve().parent / "sources-xml" / "corkscrew.xml"

    xml_text = Path(xml_path).read_text(encoding="utf-8")
    xml_text = re.sub(r"<!DOCTYPE[^>]*\[[\s\S]*?\]>", "", xml_text, flags=re.MULTILINE)
    xml_text = xml_text.replace("&default-surfaces;", "")
    root = ET.fromstring(xml_text)
    segment_parent = root.find("./section[@name='Main Track']/section[@name='Track Segments']")
    if segment_parent is None:
        raise ValueError("Track Segments section not found in corkscrew.xml")

    segments = []
    for seg in list(segment_parent):
        if seg.tag != "section":
            continue
        name = seg.attrib.get("name", "")
        if not name.startswith("s"):
            continue

        typ = _att(seg, "type", "str")
        if typ == "str":
            length = max(1.0, _att_float(seg, "lg", 20.0))
            radius = 1e9
            end_radius = radius
            sign = 0.0
        else:
            arc_deg = abs(_att_float(seg, "arc", 10.0))
            radius = max(8.0, _att_float(seg, "radius", 80.0))
            end_radius = max(8.0, _att_float(seg, "end radius", radius))
            radius_avg = 0.5 * (radius + end_radius)
            length = max(2.0, math.radians(arc_deg) * radius_avg)
            sign = 1.0 if typ == "lft" else -1.0

        segments.append(
            {
                "name": name,
                "type": typ,
                "length_m": float(length),
                "radius_m": float(radius),
                "end_radius_m": float(end_radius),
                "sign": float(sign),
            }
        )

    return segments


def _att(section, name, default):
    for node in section.findall("attstr"):
        if node.attrib.get("name") == name:
            return node.attrib.get("val", default)
    return default


def _att_float(section, name, default):
    for node in section.findall("attnum"):
        if node.attrib.get("name") == name:
            try:
                return float(node.attrib.get("val", default))
            except (TypeError, ValueError):
                return float(default)
    return float(default)


def build_corkscrew_samples(speed_ms=30.0, fps=ap.TORCS_FPS):
    dt = 1.0 / max(1.0, fps)
    samples = []
    segments = parse_corkscrew_segments()

    for seg in segments:
        seg_steps = max(4, int(seg["length_m"] / max(0.5, speed_ms * dt)))
        for i in range(seg_steps):
            p = i / max(1, seg_steps - 1)
            if seg["type"] == "str":
                alpha_deg = 0.0
                max_dist = 130.0
                turn_radius = 500.0
            else:
                r = seg["radius_m"] + (seg["end_radius_m"] - seg["radius_m"]) * p
                k = 1.0 / max(8.0, r)

                # Approximate entry->apex->opening visibility on each curved segment.
                apex_factor = 1.0 - abs(2.0 * p - 1.0)
                min_dist = max(24.0, min(85.0, 0.85 * r))
                entry_dist = max(min_dist + 16.0, min(130.0, min_dist + 48.0))
                max_dist = min_dist + (entry_dist - min_dist) * (1.0 - apex_factor)

                sin_arg = max(-0.95, min(0.95, k * max_dist * 0.5))
                alpha_abs = math.degrees(math.asin(abs(sin_arg)))
                alpha_deg = seg["sign"] * alpha_abs
                turn_radius = max(15.0, r)

            samples.append(
                {
                    "segment": seg["name"],
                    "alpha_deg": alpha_deg,
                    "max_dist": max_dist,
                    "turn_radius": turn_radius,
                }
            )

    return samples


def scenario_profile(step, total_steps):
    # Synthetic corner profile: straight -> approach zone -> apex -> opening.
    x = step / max(1, total_steps - 1)
    if x < 0.16:
        return 0.0, 120.0
    if x < 0.40:
        # Turn appears and horizon shortens.
        alpha = (x - 0.16) / 0.24 * 20.0
        max_dist = 120.0 - (x - 0.16) / 0.24 * 70.0
        return alpha, max_dist
    if x < 0.68:
        # Around apex: high curvature, shorter lookahead.
        alpha = 20.0 - (x - 0.40) / 0.28 * 10.0
        max_dist = 50.0 - (x - 0.40) / 0.28 * 22.0
        return alpha, max_dist
    # Exit opening.
    alpha = max(0.0, 10.0 - (x - 0.68) / 0.32 * 10.0)
    max_dist = 28.0 + (x - 0.68) / 0.32 * 102.0
    return alpha, max_dist


def run_simulation(name, speed_ms=28.0, steps=180):
    synthetic_samples = []
    for i in range(steps):
        alpha_deg, max_dist = scenario_profile(i, steps)
        alpha_rad = alpha_deg * math.pi / 180.0
        turn_radius = max(
            15.0,
            abs(max_dist / (2.0 * math.sin(abs(alpha_rad)))) if abs(alpha_rad) > 0.05 else 500.0,
        )
        synthetic_samples.append({"segment": "synthetic", "alpha_deg": alpha_deg, "max_dist": max_dist, "turn_radius": turn_radius})
    return run_profile(name=name, speed_ms=speed_ms, samples=synthetic_samples)


def run_profile(name, speed_ms, samples):
    c = make_controller(speed_ms=speed_ms)
    rows = []

    for i, sample in enumerate(samples):
        alpha_deg = sample["alpha_deg"]
        max_dist = sample["max_dist"]
        alpha_rad = alpha_deg * math.pi / 180.0
        curvature = 2.0 * ap.math.sin(abs(alpha_rad)) / max(max_dist, 1.0)
        turn_radius = sample["turn_radius"]

        ap.update_state(c, alpha_deg, max_dist, speed_ms, turn_radius)
        raw_target = ap.get_target_lateral(c)
        smooth = (
            ap.SMOOTH_APPROACH
            if c.state == ap.STATE_APPROACH
            else ap.SMOOTH_STRAIGHT
            if c.state == ap.STATE_STRAIGHT
            else ap.SMOOTH_CORNER
        )
        c.target_pos_filtered = smooth * raw_target + (1.0 - smooth) * c.target_pos_filtered

        distances = [max_dist] * 19
        mu_eff, _, a_brake = ap.surface_physics(c.last_track_pos)
        target_speed = ap.compute_target_speed(distances, max_dist, curvature, mu_eff, a_brake)

        rows.append(
            {
                "step": i,
                "alpha_deg": round(alpha_deg, 3),
                "max_dist": round(max_dist, 3),
                "segment": sample["segment"],
                "state": c.state,
                "phase_timer": c.phase_timer,
                "phase_duration": c.phase_duration,
                "target_pos": round(c.target_pos_filtered, 4),
                "target_speed": round(target_speed, 2),
            }
        )

    approach_steps = sum(1 for r in rows if r["state"] == ap.STATE_APPROACH)
    turn_in_steps = sum(1 for r in rows if r["state"] == ap.STATE_TURN_IN)
    exit_steps = sum(1 for r in rows if r["state"] == ap.STATE_EXIT)

    print(f"Scenario: {name}")
    print(f"  steps={len(samples)}, speed_ms={speed_ms:.1f}")
    print(f"  APPROACH={approach_steps}, TURN_IN={turn_in_steps}, EXIT={exit_steps}")
    print(f"  final target_pos={rows[-1]['target_pos']}, final state={rows[-1]['state']}")

    return rows


def run_corkscrew_simulation(speed_ms=30.0):
    samples = build_corkscrew_samples(speed_ms=speed_ms)
    return run_profile(name="corkscrew", speed_ms=speed_ms, samples=samples)


def save_rows(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    all_rows = []
    all_rows.extend(run_simulation("medium_speed", speed_ms=24.0, steps=260))
    all_rows.extend(run_simulation("high_speed", speed_ms=34.0, steps=260))
    all_rows.extend(run_corkscrew_simulation(speed_ms=28.0))

    out = Path(__file__).resolve().parent / "report" / "sim_state_trace.csv"
    save_rows(out, all_rows)
    print(f"Saved simulation trace to: {out}")
