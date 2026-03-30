import argparse
from dataclasses import dataclass
from typing import Dict, List

import numpy as np
import pandas as pd


@dataclass
class RunMetrics:
    lap_time_s: float
    offtrack_pct: float
    avg_speed_kmh: float
    max_abs_trackpos: float
    approach_abs_trackpos_mean: float
    turn_in_abs_trackpos_mean: float
    turn_in_abs_trackpos_p90: float
    turn_in_entry_max_dist_mean: float
    apex_abs_trackpos_mean: float
    approach_to_turnin_count: int
    turnin_to_exit_count: int

    def to_dict(self) -> Dict[str, float]:
        return {
            "lap_time_s": self.lap_time_s,
            "offtrack_pct": self.offtrack_pct,
            "avg_speed_kmh": self.avg_speed_kmh,
            "max_abs_trackpos": self.max_abs_trackpos,
            "approach_abs_trackpos_mean": self.approach_abs_trackpos_mean,
            "turn_in_abs_trackpos_mean": self.turn_in_abs_trackpos_mean,
            "turn_in_abs_trackpos_p90": self.turn_in_abs_trackpos_p90,
            "turn_in_entry_max_dist_mean": self.turn_in_entry_max_dist_mean,
            "apex_abs_trackpos_mean": self.apex_abs_trackpos_mean,
            "approach_to_turnin_count": float(self.approach_to_turnin_count),
            "turnin_to_exit_count": float(self.turnin_to_exit_count),
        }


def score_run(metrics: RunMetrics) -> float:
    # Lower is better. Lap time dominates; off-track is heavily penalized.
    score = metrics.lap_time_s
    score += 8.0 * metrics.offtrack_pct
    score += 2.0 * max(0.0, metrics.max_abs_trackpos - 1.0) * 100.0

    # Mild secondary terms to break ties between similarly safe runs.
    score += 0.03 * max(0.0, 130.0 - metrics.avg_speed_kmh)
    score += 0.40 * max(0.0, 0.40 - metrics.turn_in_abs_trackpos_mean) * 100.0
    score += 0.25 * max(0.0, 0.55 - metrics.apex_abs_trackpos_mean) * 100.0
    score += 0.20 * max(0.0, 0.0 - (metrics.turnin_to_exit_count - metrics.approach_to_turnin_count))
    return float(score)


def compare_scores(base: RunMetrics, candidate: RunMetrics) -> Dict[str, float]:
    base_score = score_run(base)
    cand_score = score_run(candidate)
    delta = cand_score - base_score
    return {
        "base_score": base_score,
        "candidate_score": cand_score,
        "delta": delta,
        "candidate_better": 1.0 if delta < 0.0 else 0.0,
    }


def load_telemetry(csv_path: str) -> pd.DataFrame:
    df = pd.read_csv(csv_path, sep=";", low_memory=False)
    df.columns = df.columns.str.strip()
    if "state" not in df.columns:
        raise ValueError("Missing required column: state")
    for col in ("lap_time", "speed_x", "track_pos", "max_dist"):
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    return df


def _safe_mean(series: pd.Series) -> float:
    if series is None or len(series) == 0:
        return 0.0
    return float(series.mean())


def _safe_p90_abs(series: pd.Series) -> float:
    if series is None or len(series) == 0:
        return 0.0
    return float(np.percentile(np.abs(series.values), 90))


def _state_transition_series(df: pd.DataFrame) -> pd.Series:
    return df["state"].shift(1).fillna(df["state"]) + "->" + df["state"]


def _turnin_entry_distances(df: pd.DataFrame) -> List[float]:
    tr = _state_transition_series(df)
    entries = df[tr == "APPROACH->TURN_IN"]
    if "max_dist" not in entries.columns:
        return []
    vals = pd.to_numeric(entries["max_dist"], errors="coerce").dropna()
    return [float(v) for v in vals.values]


def _apex_abs_positions(df: pd.DataFrame) -> List[float]:
    if "speed_x" not in df.columns or "track_pos" not in df.columns:
        return []
    ti_idx = df.index[df["state"] == "TURN_IN"].tolist()
    if not ti_idx:
        return []

    segments = []
    start = prev = ti_idx[0]
    for i in ti_idx[1:]:
        if i == prev + 1:
            prev = i
            continue
        segments.append((start, prev))
        start = prev = i
    segments.append((start, prev))

    apexes = []
    for a, b in segments:
        seg = df.loc[a:b]
        seg = seg.dropna(subset=["speed_x", "track_pos"])
        if len(seg) == 0:
            continue
        apex_row = seg.loc[seg["speed_x"].idxmin()]
        apexes.append(abs(float(apex_row["track_pos"])))
    return apexes


def compute_metrics(df: pd.DataFrame) -> RunMetrics:
    offtrack = (df["track_pos"].abs() > 1.0) if "track_pos" in df.columns else pd.Series([False] * len(df))
    app = df[df["state"] == "APPROACH"]
    ti = df[df["state"] == "TURN_IN"]

    tr = _state_transition_series(df)
    entry_dists = _turnin_entry_distances(df)
    apex_abs = _apex_abs_positions(df)

    return RunMetrics(
        lap_time_s=float(pd.to_numeric(df.get("lap_time", pd.Series([0.0])), errors="coerce").max()),
        offtrack_pct=float(offtrack.mean() * 100.0),
        avg_speed_kmh=float(pd.to_numeric(df.get("speed_x", pd.Series([0.0])), errors="coerce").mean()),
        max_abs_trackpos=float(pd.to_numeric(df.get("track_pos", pd.Series([0.0])), errors="coerce").abs().max()),
        approach_abs_trackpos_mean=_safe_mean(pd.to_numeric(app.get("track_pos", pd.Series(dtype=float)), errors="coerce").abs()),
        turn_in_abs_trackpos_mean=_safe_mean(pd.to_numeric(ti.get("track_pos", pd.Series(dtype=float)), errors="coerce").abs()),
        turn_in_abs_trackpos_p90=_safe_p90_abs(pd.to_numeric(ti.get("track_pos", pd.Series(dtype=float)), errors="coerce")),
        turn_in_entry_max_dist_mean=float(np.mean(entry_dists)) if entry_dists else 0.0,
        apex_abs_trackpos_mean=float(np.mean(apex_abs)) if apex_abs else 0.0,
        approach_to_turnin_count=int((tr == "APPROACH->TURN_IN").sum()),
        turnin_to_exit_count=int((tr == "TURN_IN->EXIT").sum()),
    )


def compare_metrics(base: RunMetrics, candidate: RunMetrics) -> pd.DataFrame:
    rows = []
    for key, base_val in base.to_dict().items():
        cand_val = candidate.to_dict()[key]
        diff = cand_val - base_val
        rows.append({"metric": key, "base": base_val, "candidate": cand_val, "delta": diff})
    return pd.DataFrame(rows)


def _print_summary(label: str, m: RunMetrics) -> None:
    print(f"[{label}] lap={m.lap_time_s:.3f}s, avg_speed={m.avg_speed_kmh:.2f} km/h, offtrack={m.offtrack_pct:.2f}%")
    print(
        f"[{label}] line: app|pos|={m.approach_abs_trackpos_mean:.3f}, "
        f"ti|pos|={m.turn_in_abs_trackpos_mean:.3f}, ti_p90={m.turn_in_abs_trackpos_p90:.3f}, "
        f"apex|pos|={m.apex_abs_trackpos_mean:.3f}"
    )
    print(
        f"[{label}] transitions: APP->TI={m.approach_to_turnin_count}, "
        f"TI->EXIT={m.turnin_to_exit_count}, TI_entry_dist={m.turn_in_entry_max_dist_mean:.2f}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare two telemetry CSV runs.")
    parser.add_argument("--base", required=True, help="Baseline telemetry csv path")
    parser.add_argument("--candidate", required=True, help="Candidate telemetry csv path")
    args = parser.parse_args()

    base_df = load_telemetry(args.base)
    cand_df = load_telemetry(args.candidate)
    base = compute_metrics(base_df)
    cand = compute_metrics(cand_df)

    _print_summary("BASE", base)
    _print_summary("CAND", cand)

    score_cmp = compare_scores(base, cand)
    print(
        "\n[SCORE] "
        f"base={score_cmp['base_score']:.3f}, "
        f"candidate={score_cmp['candidate_score']:.3f}, "
        f"delta={score_cmp['delta']:+.3f} "
        f"({'candidate better' if score_cmp['candidate_better'] > 0 else 'candidate worse or equal'})"
    )

    comp = compare_metrics(base, cand)
    print("\n[DELTA] candidate - base")
    print(comp.to_string(index=False))


if __name__ == "__main__":
    main()
