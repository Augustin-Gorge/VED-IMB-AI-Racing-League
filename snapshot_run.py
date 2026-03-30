import argparse
import json
import shutil
from datetime import datetime
from pathlib import Path

import telemetry_tools as tt


def snapshot_run(report_dir: Path, label: str = "") -> Path:
    telemetry = report_dir / "telemetry.csv"
    report_md = report_dir / "report.md"
    report_html = report_dir / "report.html"

    if not telemetry.exists():
        raise FileNotFoundError(f"Missing telemetry file: {telemetry}")

    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    suffix = f"-{label}" if label else ""
    out_dir = report_dir / "history" / f"run-{stamp}{suffix}"
    out_dir.mkdir(parents=True, exist_ok=False)

    copied = []
    for src in (telemetry, report_md, report_html):
        if src.exists():
            dst = out_dir / src.name
            shutil.copy2(src, dst)
            copied.append(dst.name)

    metrics = None
    try:
        df = tt.load_telemetry(str(telemetry))
        metrics = tt.compute_metrics(df).to_dict()
    except Exception as exc:
        metrics = {"error": f"Failed to compute metrics: {exc}"}

    meta = {
        "timestamp": stamp,
        "label": label,
        "source_report_dir": str(report_dir),
        "copied_files": copied,
        "metrics": metrics,
    }

    with open(out_dir / "snapshot.json", "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)

    return out_dir


def main() -> None:
    parser = argparse.ArgumentParser(description="Create a timestamped snapshot of current run artifacts.")
    parser.add_argument("--report-dir", default="report", help="Directory containing telemetry/report artifacts")
    parser.add_argument("--label", default="", help="Optional short label (e.g., attack-a1)")
    args = parser.parse_args()

    report_dir = Path(args.report_dir).resolve()
    out_dir = snapshot_run(report_dir, args.label.strip())
    print(f"Snapshot created: {out_dir}")


if __name__ == "__main__":
    main()
