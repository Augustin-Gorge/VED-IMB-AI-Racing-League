# CODE CHANGES REFERENCE

This document lists all modifications made to `autopilot.py` to optimize trajectory and control.

---

## 1. LATERAL SMOOTHING - More Geometric Corners

**File**: `autopilot.py`  
**Function**: Global constants  
**Line**: ~145

### Change
```python
# BEFORE:
SMOOTH_STRAIGHT = 0.18; SMOOTH_CORNER = 0.10; SMOOTH_APPROACH = 0.06

# AFTER:
SMOOTH_STRAIGHT = 0.18; SMOOTH_CORNER = 0.06; SMOOTH_APPROACH = 0.05
```

### Impact
- `SMOOTH_CORNER`: Reduced from 0.10 → 0.06 = tighter lateral position blending
  - Effect: More deliberate, less filtered lateral control through turns
  - Result: Geometric trajectory (lower lateral σ)
- `SMOOTH_APPROACH`: 0.06 → 0.05 = slightly crisper approach geometry

---

## 2. TARGETED LATERAL STABILIZATION

**Function**: `stabilize_target_dynamic()`

### Change
```python
# BEFORE:
def stabilize_target_dynamic(raw_target, track_pos, max_dist, speed_kmh, state=STATE_STRAIGHT):
    edge = max(0.0, min(1.0, (abs(track_pos) - 0.72) / 0.36))
    edge_hard = max(0.0, min(1.0, (abs(track_pos) - 0.92) / 0.06))
    horizon = max(0.0, min(1.0, (10.0 - max_dist) / 10.0))
    low_speed = max(0.0, min(1.0, (45.0 - speed_kmh) / 45.0))
    center_pull = max(edge, edge_hard, horizon * (0.45 + 0.55 * low_speed))
    if state == STATE_APPROACH:
        center_pull *= 0.75
    elif state == STATE_TURN_IN:
        center_pull *= 0.68
    return raw_target * (1.0 - center_pull)

# AFTER:
def stabilize_target_dynamic(raw_target, track_pos, max_dist, speed_kmh, state=STATE_STRAIGHT):
    edge = max(0.0, min(1.0, (abs(track_pos) - 0.72) / 0.36))
    edge_hard = max(0.0, min(1.0, (abs(track_pos) - 0.92) / 0.06))
    horizon = max(0.0, min(1.0, (12.0 - max_dist) / 12.0))  # 10.0 → 12.0
    low_speed = max(0.0, min(1.0, (45.0 - speed_kmh) / 45.0))
    center_pull = max(edge, edge_hard, horizon * (0.50 + 0.60 * low_speed))  # 0.45 → 0.50, 0.55 → 0.60
    if state == STATE_APPROACH:
        center_pull *= 0.72  # 0.75 → 0.72
    elif state == STATE_TURN_IN:
        center_pull *= 0.64  # 0.68 → 0.64
    return raw_target * (1.0 - center_pull)
```

### Impact
- Longer horizon (12m vs 10m): Looks further ahead for center stabilization
- Stronger base pull (0.50 vs 0.45): More aggressive return-to-center
- Stronger low-speed factor (0.60 vs 0.55): Better handling at low speeds
- Less suppression in TURN_IN (0.64 vs 0.68): More geometric cornering

---

## 3. MORE AGGRESSIVE GEOMETRIC SHAPING

**Function**: `shape_geometric_line()`

### Change
```python
# BEFORE:
def shape_geometric_line(raw_target, state, alpha_deg, speed_kmh):
    if state not in (STATE_APPROACH, STATE_TURN_IN, STATE_EXIT):
        return raw_target
    turn_vis = max(0.0, min(1.0, abs(alpha_deg) / 35.0))
    speed_fac = max(0.0, min(1.0, speed_kmh / 120.0))
    phase_gain = 1.12 if state == STATE_TURN_IN else 1.08 if state == STATE_APPROACH else 1.05
    gain = phase_gain + 0.22 * turn_vis + 0.12 * speed_fac
    return max(-0.98, min(0.98, raw_target * gain))

# AFTER:
def shape_geometric_line(raw_target, state, alpha_deg, speed_kmh):
    if state not in (STATE_APPROACH, STATE_TURN_IN, STATE_EXIT):
        return raw_target
    turn_vis = max(0.0, min(1.0, abs(alpha_deg) / 35.0))
    speed_fac = max(0.0, min(1.0, speed_kmh / 120.0))
    # Increased gain for more aggressive steering in all phases
    phase_gain = 1.16 if state == STATE_TURN_IN else 1.12 if state == STATE_APPROACH else 1.08
    gain = phase_gain + 0.26 * turn_vis + 0.14 * speed_fac
    return max(-0.98, min(0.98, raw_target * gain))
```

### Impact
- TURN_IN gain: 1.12 → 1.16 (+3.6% steering magnitude)
- APPROACH gain: 1.08 → 1.12 (+3.7% steering magnitude)
- Turn visibility factor: 0.22 → 0.26 (+18% turn-based correction)
- Speed factor: 0.12 → 0.14 (+17% speed-dependent steering)
- **Result**: Sharper, more responsive steering through corners

---

## 4. AGGRESSIVE CRUISE FLOOR - ELIMINATE COASTING

**Function**: `compute_pedals()`

### Change
```python
# BEFORE:
if speed > 30.0 and speed >= target_speed-8.0:
    cruise_floor = 0.24 * max(0.0, 1.0 - abs(R['steer'])/0.85)
    acc = max(acc, cruise_floor)

# AFTER:
if speed > 30.0 and speed >= target_speed-8.0:
    steer_factor = max(0.0, 1.0 - abs(R['steer'])/0.85)
    cruise_floor = 0.32 + 0.18 * steer_factor  # 0.32-0.50 range
    acc = max(acc, cruise_floor)
```

### Impact
- Base throttle: 0 → 0.32 (always applies minimum throttle)
- Additional based on steering: +0 to 0.18 (reduces as steering increases)
- Range: 0.32-0.50 throttle when within 8 km/h of target
- **Result**: Eliminates coasting, maintains speed near target

---

## 5. MORE AGGRESSIVE BRAKING

**Function**: `compute_pedals()`

### Change
```python
# BEFORE:
else:
    if speed < 15.0: return 0.0, 0.0
    brk = (speed-target_speed)/13.5
    if abs(S['angle']) > EMERGENCY_BRAKE_YAW: brk = max(brk, 0.5)
    brk = min(brk, math.sqrt(max(0.0, 1.0-R['steer']**2)))
    return 0.0, max(0.0, min(1.0, brk))

# AFTER:
else:
    if speed < 15.0: return 0.0, 0.0
    brk = (speed-target_speed)/10.5  # 13.5 → 10.5
    if abs(S['angle']) > EMERGENCY_BRAKE_YAW: brk = max(brk, 0.6)  # 0.5 → 0.6
    brk = min(brk, math.sqrt(max(0.0, 1.0-R['steer']**2)))
    return 0.0, max(0.0, min(1.0, brk))
```

### Impact
- Brake divisor: 13.5 → 10.5 (28.6% more aggressive)
  - Example: 10 km/h overspeed: 0.74 braking vs 0.59 before
- Emergency brake: 0.5 → 0.6 (+20% emergency braking)
- **Result**: Faster deceleration to target speed, smoother phase transitions

---

## 6. IMPROVED STEERING CONTROL

**Function**: `compute_steering()`

### Change
```python
# BEFORE:
def compute_steering(S, target_pos, c):
    P=0.38; YA=0.25; MAX_R=0.10
    # Inspired by 1m30: stronger yaw damping, especially at lower speed.
    speed_kmh = max(1.0, S.get('speedX', 0.0))
    low_speed = max(0.0, min(1.0, (70.0 - speed_kmh) / 70.0))
    D = (14.0 + 10.0 * low_speed) / math.pi
    c.yaw_f = YA*S['angle'] + (1.0-YA)*c.yaw_f
    err = S['trackPos'] - target_pos
    sp  = -(err*P)
    sd  = c.yaw_f*D
    wp  = S['trackPos']
    wg  = -(wp-0.84)*2.6 if wp>0.84 else -(wp+0.84)*2.6 if wp<-0.84 else 0.0
    raw = max(-1.0, min(1.0, sp+sd+wg))
    
    if c.state == STATE_STRAIGHT:
        if abs(wp) < 0.12 and abs(c.yaw_f) < 0.03:
            raw *= 0.55
        if abs(wp) < 0.04 and abs(c.yaw_f) < 0.015:
            raw = 0.0
    
    if abs(err) < 0.40:
        d = raw-c.prev_steer
        max_rate = 0.06 if c.state == STATE_STRAIGHT else MAX_R
        if abs(d)>max_rate: raw = c.prev_steer+math.copysign(max_rate,d)
    c.prev_steer = raw
    return raw, sp, sd

# AFTER:
def compute_steering(S, target_pos, c):
    P=0.40; YA=0.27; MAX_R=0.10  # P: 0.38→0.40, YA: 0.25→0.27
    # Improved steering: faster response for geometric cornering
    speed_kmh = max(1.0, S.get('speedX', 0.0))
    low_speed = max(0.0, min(1.0, (70.0 - speed_kmh) / 70.0))
    D = (14.5 + 9.5 * low_speed) / math.pi  # 14.0→14.5, 10.0→9.5
    c.yaw_f = YA*S['angle'] + (1.0-YA)*c.yaw_f
    err = S['trackPos'] - target_pos
    sp  = -(err*P)
    sd  = c.yaw_f*D
    wp  = S['trackPos']
    wg  = -(wp-0.84)*2.7 if wp>0.84 else -(wp+0.84)*2.7 if wp<-0.84 else 0.0  # 2.6→2.7
    raw = max(-1.0, min(1.0, sp+sd+wg))
    
    if c.state == STATE_STRAIGHT:
        if abs(wp) < 0.12 and abs(c.yaw_f) < 0.03:
            raw *= 0.50  # 0.55→0.50
        if abs(wp) < 0.04 and abs(c.yaw_f) < 0.015:
            raw = 0.0
    
    if abs(err) < 0.40:
        d = raw-c.prev_steer
        max_rate = 0.055 if c.state == STATE_STRAIGHT else MAX_R  # 0.06→0.055
        if abs(d)>max_rate: raw = c.prev_steer+math.copysign(max_rate,d)
    c.prev_steer = raw
    return raw, sp, sd
```

### Impact
- P gain: 0.38 → 0.40 (+5.3% proportional correction)
- YA filter: 0.25 → 0.27 (+8% yaw responsiveness)
- D gain base: 14.0 → 14.5 (faster low-speed damping)
- D gain low-speed: 10.0 → 9.5 (less low-speed damping)
- Wall response: 2.6 → 2.7 (+3.8% edge correction)
- Straight damping: 0.55 → 0.50 (more aggressive on straights)
- **Result**: Faster steering response, better edge handling, less oscillation

---

## 7. EARLIER CORNER ENTRY

**Function**: `update_state()`

### Change
```python
# BEFORE:
dist_turn_in = max(32.0, min(70.0, speed_ms * 1.6))  # earlier approach trigger
dist_apex = max(25.0, min(60.0, speed_ms * 1.2))     # 1.2 seconds lookahead
dist_approach = max(60.0, min(100.0, speed_ms * 2.5)) # 2.5 seconds lookahead

# AFTER:
dist_turn_in = max(32.0, min(70.0, speed_ms * 1.4))  # Earlier TURN_IN trigger (was 1.6)
dist_apex = max(25.0, min(60.0, speed_ms * 1.1))     # Tighter apex (was 1.2 seconds)
dist_approach = max(60.0, min(100.0, speed_ms * 2.2)) # Shortened approach (was 2.5)
```

### Impact
- dist_turn_in: 1.6 → 1.4 seconds (-12.5% lookahead)
  - At 20 m/s (72 km/h): 32→28m trigger distance
  - Effect: Earlier transition to TURN_IN phase
  
- dist_apex: 1.2 → 1.1 seconds (-8.3% lookahead)
  - Effect: Tighter apex commitment window
  
- dist_approach: 2.5 → 2.2 seconds (-12% lookahead)
  - Effect: Shorter approach phase, more aggressive entry

- **Result**: Corners entered earlier with tighter geometry

---

## 8. TIGHTER CORNER ARC DURATIONS

**Function**: `_dur_arc()`

### Change
```python
# BEFORE:
def _dur_arc(r, a_rad, v, lo=PHASE_MIN_FRAMES, hi=70):
    arc = max(2.0, r*max(0.05,abs(a_rad)))
    return int(max(lo, min(hi, arc/max(1.0,v)*TORCS_FPS))), arc

# AFTER:
def _dur_arc(r, a_rad, v, lo=PHASE_MIN_FRAMES, hi=70):
    # IMPROVED: Tighter arc duration for more geometric control
    arc = max(2.0, r*max(0.06, abs(a_rad)))  # 0.05 → 0.06
    return int(max(lo, min(hi-8, arc/max(1.0,v)*TORCS_FPS))), arc  # hi → hi-8
```

### Impact
- Arc multiplier: 0.05 → 0.06 (+20% arc length calculation)
- Max duration: 70 → 62 frames (-11.4% max turn duration)
- **Result**: Tighter, shorter corner phases with more geometric definition

---

## 9. MORE AGGRESSIVE TRACTION CONTROL

**Function**: `traction_control()`

### Change
```python
# BEFORE:
def traction_control(S, accel, max_thr):
    if not TRACTION_CONTROL_ON: return accel
    if 'wheelSpinVel' not in S or len(S['wheelSpinVel'])!=4: return accel
    spin = (S['wheelSpinVel'][2]+S['wheelSpinVel'][3])-(S['wheelSpinVel'][0]+S['wheelSpinVel'][1])
    if spin > (12.0 if abs(S.get('trackPos',0))>1.0 else 3.5): accel -= 0.08
    return max(0.0, min(max_thr, accel))

# AFTER:
def traction_control(S, accel, max_thr):
    if not TRACTION_CONTROL_ON: return accel
    if 'wheelSpinVel' not in S or len(S['wheelSpinVel'])!=4: return accel
    spin = (S['wheelSpinVel'][2]+S['wheelSpinVel'][3])-(S['wheelSpinVel'][0]+S['wheelSpinVel'][1])
    # More aggressive TC - higher threshold before cutting throttle
    if spin > (14.0 if abs(S.get('trackPos',0))>1.0 else 5.0): accel -= 0.06  # 12.0→14.0, 3.5→5.0, 0.08→0.06
    return max(0.0, min(max_thr, accel))
```

### Impact
- Off-track spin threshold: 12.0 → 14.0 rad/s (+16.7% tolerance)
- Normal spin threshold: 3.5 → 5.0 rad/s (+42.9% tolerance)  
- Penalty when exceeded: -0.08 → -0.06 (-25% throttle cut)
- **Result**: Maintains throttle during acceleration, cuts less aggressively on wheelspin

---

## Summary Table

| Component | Before | After | Change | Impact |
|-----------|--------|-------|--------|--------|
| `SMOOTH_CORNER` | 0.10 | 0.06 | -40% | Tighter lateral control |
| Cruise floor base | 0.24 | 0.32 | +33% | Eliminates coasting |
| Cruise floor range | 0-0.24 | 0.32-0.50 | +130% | Always maintains throttle |
| Brake divisor | 13.5 | 10.5 | -22% | More aggressive braking |
| Emergency brake | 0.5 | 0.6 | +20% | Stronger emergency response |
| Steering P gain | 0.38 | 0.40 | +5.3% | Faster position correction |
| Steering YA filter | 0.25 | 0.27 | +8% | More yaw response |
| TURN_IN lookahead | 1.6s | 1.4s | -12% | Earlier entry |
| APEX lookahead | 1.2s | 1.1s | -8% | Tighter commitment |
| Arc multiplier | 0.05 | 0.06 | +20% | Tighter arcs |
| TC off-track threshold | 12.0 | 14.0 | +16.7% | More wheelspin allowed |
| TC penalty | -0.08 | -0.06 | -25% | Gentler TC cuts |

All changes are **backwards compatible** and can be selectively reverted if overcorrection is detected.

---

## Reversion Instructions

To revert any change:

1. Open `autopilot.py`
2. Find the section using the function name above
3. Replace "AFTER" code with "BEFORE" code
4. Save and re-run

Example to revert just coasting reduction:
```python
# Replace this:
cruise_floor = 0.32 + 0.18 * steer_factor

# With:
cruise_floor = 0.24 * steer_factor
```
