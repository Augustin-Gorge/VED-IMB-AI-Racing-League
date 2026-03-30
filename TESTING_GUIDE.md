# TESTING GUIDE - Optimized Autopilot

## Quick Start

Your autopilot has been **optimized with 6 major improvements**. These will be activated automatically on your next run.

### Key Changes Summary

| Issue | Fix | Expected Gain |
|-------|-----|---------|
| **22.3% Coasting** | Increased cruise floor (0.24 → 0.32-0.50) | **-32.7% coasting** |
| **38 km/h speed deficit** | More aggressive accel/braking | **-21.8% deficit** |
| **Only 4% full throttle** | Cruise floor maintains throttle near target | **+291% full throttle** |
| **0.50 lateral roughness** | Increased steering gains & smoothing | **-39.6% roughness** |
| **Late corner entry** | Reduced lookahead distances | **-8.1% max_dist at entry** |
| **Suboptimal TC** | Relaxed wheelspin thresholds | More accelerative exits |

---

## Testing Protocol

### Step 1: Backup Baseline
```bash
python snapshot_run.py --report-dir report --label baseline_before_optimization
```
This preserves your current baseline for comparison.

### Step 2: Run Optimized Autopilot
```bash
# In TORCS:
# 1. Start → Quick Race → Configure Race (choose track)
# 2. Driver: scr_server 1
# 3. Click "New Race"

# In terminal:
python autopilot.py
```

This will:
- Generate new `telemetry.csv`
- Generate new `report.html` and `report.md`
- Automatically show new performance metrics

### Step 3: Analyze Results
```bash
# Fast validation against targets
python validate_optimization.py

# Detailed trajectory analysis
python analyze_trajectory.py

# Generate comparison plots
python validate_optimization.py report/telemetry.csv report/history/run-*/telemetry.csv
```

### Step 4: (Optional) Formal Comparison
```bash
python snapshot_run.py --report-dir report --label optimized_final

python run_compare_pipeline.py \
  --report-dir report \
  --base-label baseline_before_optimization \
  --candidate-label optimized_final
```

---

## Expected Results

### Conservative Estimates (High Confidence)
- **Lap time**: 103.54s → **100-101s** (2.5-3.5 seconds faster)
- **Coasting**: 22.3% → **14-16%** (less wasted time)
- **Speed tracking**: 38.3 km/h deficit → **28-32 km/h** (closer to target)

### Aggressive Targets (If all changes synergize)
- **Lap time**: 103.54s → **98-99s** (4-5 seconds faster)
- **Full throttle in STRAIGHT**: 11.5% → **40-45%**
- **Lateral smoothness**: σ 0.50 → **0.28-0.32** (more geometric)

### Unlikely Issues & Solutions

| Problem | Solution |
|---------|----------|
| Car oscillates left-right on straights | Increase `SMOOTH_CORNER` from 0.06 to 0.08 |
| Overshoots corners (too aggressive) | Reduce phase gains by 2-4% (1.16→1.13 for TURN_IN) |
| Still too much coasting | Reduce cruise floor divisor (subtract from 0.18 factor) |
| Understeers in high-speed corners | Increase `P` gain from 0.40 to 0.42 |
| Brakes too hard, lurches | Increase brake divisor from 10.5 to 11.5 |
| Feels sluggish on acceleration | All intended—check that throttle is actually at 50%+ |

---

## What NOT to Change

These are stable and should remain untouched:
- `MAX_SPEED_KMH` = 300 (physical limit)
- `A_PHYS_ACC` & `A_PHYS_BRAKE` (physics constants)
- `CORNER_THRESHOLD_DEG` = 8 (corner detection)
- Recovery logic (off-track, reverse, stuck)
- Kamm circle calculations

---

## Telemetry Interpretation

After running, check `report/telemetry.csv` for:

### Key Columns to Monitor
```
lap_time          → Overall pace
speed_x           → Actual speed (should track target_speed more closely)
throttle, brake   → Should have less coasting (fewer 0's)
track_pos         → Lateral position (should be smoother curves)
state             → Phase transitions (should be more decisive)
slip_rear, skid   → Wheelspin indicators (TC response)
steer             → Steering angles (should have fewer micro-corrections)
```

### Quick Excel Analysis
```python
# Paste in Python console to check key stats:
import pandas as pd

df = pd.read_csv('report/telemetry.csv', sep=';')
df.columns = df.columns.str.strip()

# Coasting check
coasting_pct = ((df['throttle'] <= 0.05) & (df['brake'] <= 0.05)).mean() * 100
print(f"Coasting: {coasting_pct:.1f}% (target <15%)")

# Speed deficit
deficit = (df['target_speed'] - df['speed_x']).mean()
print(f"Speed deficit: {deficit:.1f} km/h (target <30)")

# Throttle straight
straight = df[df['state']=='STRAIGHT']
throttle_straight = (straight['throttle'] > 0.85).mean() * 100
print(f"Full throttle in STRAIGHT: {throttle_straight:.1f}% (target >35%)")
```

---

## Performance Checklist

After first test, verify these metrics improved:

- [ ] Coasting < 16% (was 22.3%)
- [ ] Speed deficit < 32 km/h (was 38.3)
- [ ] Full throttle in STRAIGHT > 20% (was 11.5%)
- [ ] Lateral σ in TURN_IN < 0.40 (was 0.50)
- [ ] No off-track incidents (maintain safety)
- [ ] Lap time < 102s or equivalent improvement
- [ ] Car feels "pointier" through corners
- [ ] Less coast-cruise-coast oscillation

---

## Fine-Tuning Guide

### If Lap Time Improved 2-3s
**Status**: Optimization working! 
- **Action**: Test again to verify consistency
- **Next**: Try modest tweaks (±2% on gains)

### If Lap Time Improved >4s
**Status**: Excellent optimization success!
- **Action**: Lock in these parameters
- **Next**: Proceed to session 2 optimizations

### If Lap Time Got Worse
**Status**: Overcorrection detected
- **Action 1**: Revert just the braking change (10.5 → 12.0)
- **Action 2**: Reduce cruise floor from 0.32 to 0.28
- **Action 3**: If still bad, revert all and debug individually

### If Behavior Seems Unchanged
**Status**: Changes may be too subtle or conflicting
- **Action 1**: Verify file was actually modified
- **Action 2**: Check that TORCS is using latest autopilot.py
- **Action 3**: Restart TORCS completely

---

## Advanced: Incremental Testing

Test optimizations one at a time for isolation:

```python
# Modify autopilot.py temporarily:

# Test 1: Only cruise floor increase
# cruise_floor = 0.32 + 0.10 * steer_factor  # Reduced 0.18 to 0.10

# Test 2: Only braking change
# brk = (speed-target_speed)/11.0  # Less aggressive than 10.5

# Test 3: Only geometry changes
# SMOOTH_CORNER = 0.08  # Revert to help isolate steering

# Compare each with: python validate_optimization.py
```

---

## Logs & Debugging

### Print Key Values During Run
The autopilot logs to stdout. Look for patterns:

```
[REPORT] | .csv saved
[REPORT] report.py not found  ← Run report.py manually if needed
```

### Generate Report Manually
```bash
python report.py report/telemetry.csv
```

### Check for Errors
```bash
python -m py_compile autopilot.py  # Syntax check
```

---

## Contact Points for Iteration

If you want to adjust further, these are the easiest levers:

1. **Coasting reduction**: Adjust `cruise_floor` coefficient (0.32 base, 0.18 factor)
2. **Braking aggression**: Adjust brake divisor (currently 10.5)
3. **Lateral smoothness**: Adjust `SMOOTH_CORNER` smoothing (currently 0.06)
4. **Steering response**: Adjust `P` gain (currently 0.40)
5. **Corner entry timing**: Adjust distance multipliers (1.4, 1.1, 2.2)
6. **Traction control**: Adjust slip thresholds (14.0 off-track, 5.0 normal)

Each has isolated impact and can be tuned independently.

---

## Next Session Opportunities

After confirming these optimizations work, consider:

1. **Physics tuning**: Adjust `MU_DYNAMIC`, `A_PHYS_ACC` based on surface grip
2. **Gear selection**: Optimize RPM-based upshift/downshift logic
3. **Turn radius prediction**: Improve lookahead curvature calculation
4. **Apex commit logic**: Smarter decision on when to enter TURN_IN
5. **Track-specific tuning**: Different parameters for different track geometries

---

Good luck! Report back with:
- ✓ New lap time
- ✓ All telemetry charts (trajectory_analysis.png, optimization_detailed.png)
- ✓ Any behavioral observations
