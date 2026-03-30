import argparse
import csv
import subprocess
from datetime import datetime
from pathlib import Path

import telemetry_tools as tt
from snapshot_run import snapshot_run


def _run_cmd(cmd: str, label: str) -> None:
    print(f"[{label}] {cmd}")
    subprocess.run(cmd, shell=True, check=True)


def _write_comparison(
    out_dir: Path,
    base_name: str,
    cand_name: str,
    comp_rows: list[dict],
    score_cmp: dict,
) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    out_csv = out_dir / f"compare-{stamp}.csv"

    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["metric", "base", "candidate", "delta"])
        writer.writeheader()
        writer.writerows(comp_rows)

    out_txt = out_dir / f"compare-{stamp}.txt"
    with open(out_txt, "w", encoding="utf-8") as f:
        f.write(f"base: {base_name}\n")
        f.write(f"candidate: {cand_name}\n\n")
        f.write(
            "score: "
            f"base={score_cmp['base_score']:.6f} "
            f"candidate={score_cmp['candidate_score']:.6f} "
            f"delta={score_cmp['delta']:+.6f}\n\n"
        )
        for row in comp_rows:
            f.write(
                f"{row['metric']}: base={row['base']:.6f} candidate={row['candidate']:.6f} delta={row['delta']:+.6f}\n"
            )

    print(f"[COMPARE] CSV: {out_csv}")
    print(f"[COMPARE] TXT: {out_txt}")
    return out_csv


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Run one-shot benchmark pipeline: snapshot baseline, run autopilot, regenerate report, "
            "snapshot candidate, and compare telemetry metrics."
        )
    )
    parser.add_argument("--report-dir", default="report", help="Report directory containing telemetry.csv")
    parser.add_argument("--base-label", default="baseline", help="Label used for baseline snapshot")
    parser.add_argument("--candidate-label", default="candidate", help="Label used for candidate snapshot")
    parser.add_argument(
        "--run-cmd",
        default="python autopilot.py",
        help="Command used to execute one driving run",
    )
    parser.add_argument(
        "--report-cmd",
        default="python report.py report/telemetry.csv",
        help="Command used to regenerate reports after run",
    )
    parser.add_argument(
        "--skip-run",
        action="store_true",
        help="Skip driving run command (useful if run already done manually)",
    )
    parser.add_argument(
        "--skip-report",
        action="store_true",
        help="Skip report regeneration command",
    )
    args = parser.parse_args()

    report_dir = Path(args.report_dir).resolve()
    print(f"[PIPELINE] report_dir={report_dir}")

    baseline_dir = snapshot_run(report_dir, args.base_label)
    print(f"[PIPELINE] baseline snapshot: {baseline_dir}")

    if not args.skip_run:
        _run_cmd(args.run_cmd, "RUN")

    if not args.skip_report:
        _run_cmd(args.report_cmd, "REPORT")

    candidate_dir = snapshot_run(report_dir, args.candidate_label)
    print(f"[PIPELINE] candidate snapshot: {candidate_dir}")

    base_df = tt.load_telemetry(str(baseline_dir / "telemetry.csv"))
    cand_df = tt.load_telemetry(str(candidate_dir / "telemetry.csv"))
    base_metrics = tt.compute_metrics(base_df)
    cand_metrics = tt.compute_metrics(cand_df)

    comp = tt.compare_metrics(base_metrics, cand_metrics)
    comp_rows = comp.to_dict("records")
    score_cmp = tt.compare_scores(base_metrics, cand_metrics)

    print("[PIPELINE] comparison summary")
    print(
        "  SCORE: "
        f"base={score_cmp['base_score']:.3f} "
        f"candidate={score_cmp['candidate_score']:.3f} "
        f"delta={score_cmp['delta']:+.3f} "
        f"({'candidate better' if score_cmp['candidate_better'] > 0 else 'candidate worse or equal'})"
    )
    for _, row in comp.iterrows():
        print(
            f"  {row['metric']}: base={row['base']:.6f} candidate={row['candidate']:.6f} delta={row['delta']:+.6f}"
        )

    _write_comparison(
        report_dir / "history" / "comparisons",
        baseline_dir.name,
        candidate_dir.name,
        comp_rows,
        score_cmp,
    )


if __name__ == "__main__":
    main()
