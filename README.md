# IBM Racing League - TORCS Autopilot | Vinci Eco Drive

## About
Vinci Eco Drive is a student association from Pole Leonard de Vinci in Courbevoie, France.

This project runs an autonomous driving agent in TORCS and generates a race report after each run.

## Requirements
- Windows, or macOS with Wine
- Python 3.8+
- Python packages: `pandas`, `numpy`, `matplotlib`
- TORCS: https://ibm.biz/TORCSdownloadzip

## Start

### 1) Configure race
- Start `wtorcs.exe`.
- Go to: `Race` > `Quick Race` > `Configure Race`.
- Choose your track.
- Driver must be `scr_server 1`.

### 2) Run the autopilot
- Launch : > `New Race`
- Execute: `python autopilot.py`.

When connected, the car is controlled automatically.

## After Each Run
Files are generated in `report/`:
- `telemetry.csv`: full telemetry log
- `report.md`: text summary
- `report.html`: visual report

## Optional Comparison Workflow
- Save snapshots of runs:
	- Execute: `python snapshot_run.py --report-dir report --label baseline`.

- Run one-shot baseline vs candidate pipeline:
	- Execute: `python run_compare_pipeline.py --report-dir report --base-label baseline --candidate-label candidate`.