import math
import os
from torcs_base import Client

# ═══════════════════════════════════════════════════════════════════════════════
#  GLOBAL SETTINGS
# ═══════════════════════════════════════════════════════════════════════════════

MAX_SPEED_KMH       = 300    # Absolute speed cap (km/h)
EMERGENCY_BRAKE_YAW = 0.9    # Yaw angle (rad) that triggers emergency braking
TRACTION_CONTROL_ON = True   # Enable/disable wheel-spin limiter

# 19 range-finder angles (degrees) — fixed TORCS sensor layout
SENSOR_ANGLES = [-90, -75, -60, -45, -30, -20, -15, -10, -5,
                  0, 5, 10, 15, 20, 30, 45, 60, 75, 90]

# ═══════════════════════════════════════════════════════════════════════════════
#  STATE MACHINE — DRIVING PHASES
# ═══════════════════════════════════════════════════════════════════════════════
#
#  Racing line target: Outside → Apex → Outside
#
#  STRAIGHT  : straight track, car centred
#  APPROACH  : corner detected ahead, drift progressively toward the OUTSIDE
#  TURN_IN   : dive toward the APEX (inside kerb)
#  APEX      : clipping point, minimum speed
#  EXIT      : track-out progressively toward the OUTSIDE
#
#  TORCS convention (do NOT invert):
#    trackPos > 0  → LEFT side of track
#    trackPos < 0  → RIGHT side of track
#    alpha_deg > 0 → LEFT corner  (outside = right, target trackPos < 0)
#    alpha_deg < 0 → RIGHT corner (outside = left,  target trackPos > 0)

STATE_STRAIGHT = "STRAIGHT"
STATE_APPROACH = "APPROACH"
STATE_TURN_IN  = "TURN_IN"
STATE_APEX     = "APEX"
STATE_EXIT     = "EXIT"

# ── Transition thresholds ─────────────────────────────────────────────────────
CORNER_THRESHOLD_DEG = 8.0   # Min open-corridor angle (°) to detect a corner
DIST_APPROACH        = 78.0  # Sensor range (m) → start outside drift
DIST_TURN_IN         = 62.0  # Sensor range (m) → begin diving to apex
DIST_APEX            = 44.0  # Sensor range (m) → apex zone
PHASE_MIN_FRAMES     = 22    # Min frames per state before any transition (anti-oscillation)

# ── Lateral target positions (trackPos units) ─────────────────────────────────
AMP_OUTSIDE = 0.78   # Outside drift amplitude on entry
AMP_APEX    = 0.83   # Apex depth (deep inside kerb)
AMP_EXIT    = 0.68   # Track-out amplitude on exit

# ── Exponential smoothing on the lateral target (anti-oscillation) ────────────
# alpha → 0 : slow, buttery convergence / alpha → 1 : instant step
SMOOTH_STRAIGHT = 0.06   # Gentle on straights
SMOOTH_CORNER   = 0.10   # Slightly quicker in corners to follow the line


# ═══════════════════════════════════════════════════════════════════════════════
#  PHYSICS — TARGET SPEED
# ═══════════════════════════════════════════════════════════════════════════════

def compute_target_speed(max_dist, turn_radius):
    """
    Derive the maximum safe cornering speed from first principles, then
    back-calculate the allowable approach speed given the available braking
    distance.

    Corner equilibrium with aerodynamic downforce:
        m * v² / R = mu * (m * g + C_df * v²)
        → v² = (mu * m * g) / (m/R − mu * C_df)

    Approach speed (kinematics):
        v_approach² = v_apex² + 2 * a_brake * d_braking
    """
    m    = 600.0   # Vehicle mass (kg)
    mu   = 1.6     # Grip coefficient (slicks, dry tarmac)
    g    = 9.81    # Gravity (m/s²)
    C_df = 1.1     # Aero downforce coefficient ≈ 0.5 * rho * S * Cl

    denom = (m / turn_radius) - (mu * C_df)

    if denom > 0.05:
        v_apex_ms = math.sqrt((mu * m * g) / denom)
    else:
        # Aero alone prevents sliding at any realistic speed
        v_apex_ms = MAX_SPEED_KMH / 3.6

    v_apex_ms *= 0.90   # 10 % safety margin at the clipping point

    # Maximum deceleration: mu * g * 0.88 ≈ 13.8 m/s² (engine braking + aero included)
    a_brake   = mu * g * 0.88
    # Available braking distance (10 m safety buffer = late braking)
    d_braking = max(0.0, max_dist - 10.0)

    v_approach_ms  = math.sqrt(v_apex_ms**2 + 2.0 * a_brake * d_braking)
    v_approach_kmh = v_approach_ms * 3.6

    return min(MAX_SPEED_KMH, v_approach_kmh)


# ═══════════════════════════════════════════════════════════════════════════════
#  RACING LINE — LATERAL TARGET
# ═══════════════════════════════════════════════════════════════════════════════

def get_target_lateral(state, turn_sign, phase_timer):
    """
    Return the desired trackPos for the current driving state.

    turn_sign : +1 for left corner, −1 for right corner.
    phase_timer: frames elapsed in the current state (drives interpolation).

    Outside → Apex → Outside geometry:
      APPROACH  : drift to outside  → −turn_sign * AMP_OUTSIDE
      TURN_IN   : blend outside → apex over 35 frames
      APEX      : hold clipping point → +turn_sign * AMP_APEX
      EXIT      : blend apex → outside over 55 frames
      STRAIGHT  : track centre → 0.0
    """
    if state == STATE_STRAIGHT:
        return 0.0

    if state == STATE_APPROACH:
        t = min(1.0, phase_timer / 50.0)
        return -turn_sign * AMP_OUTSIDE * t

    if state == STATE_TURN_IN:
        t         = min(1.0, phase_timer / 35.0)
        pos_out   = -turn_sign * AMP_OUTSIDE
        pos_apex  =  turn_sign * AMP_APEX
        return pos_out + (pos_apex - pos_out) * t

    if state == STATE_APEX:
        return turn_sign * AMP_APEX

    if state == STATE_EXIT:
        t         = min(1.0, phase_timer / 55.0)
        pos_apex  =  turn_sign * AMP_APEX
        pos_out   = -turn_sign * AMP_EXIT
        return pos_apex + (pos_out - pos_apex) * t

    return 0.0


# ═══════════════════════════════════════════════════════════════════════════════
#  STEERING — PD CONTROLLER
# ═══════════════════════════════════════════════════════════════════════════════

def compute_steering(S, target_pos_filtered):
    """
    PD controller tracking the smoothed lateral target.

    Terms:
      steer_p    : proportional correction on lateral position error
      steer_d    : derivative correction on yaw angle (alignment damping)
      wall_guard : emergency repulsion if the car approaches the physical edge

    ⚠  TORCS sign convention — do NOT invert:
         steer_d = S['angle'] * D_GAIN
       Reversing this sign causes the car to steer away from the track.
    """
    P_GAIN = 0.38             # Position error gain (slightly soft to limit oscillation)
    D_GAIN = 25.0 / math.pi  # Yaw angle gain (TORCS convention, sign preserved)

    pos_error = S['trackPos'] - target_pos_filtered
    steer_p   = -(pos_error * P_GAIN)

    # ⚠  Sign preserved — TORCS convention
    steer_d   = S['angle'] * D_GAIN

    wall_guard = 0.0
    if S['trackPos'] > 0.87:
        wall_guard = -(S['trackPos'] - 0.87) * 2.5
    elif S['trackPos'] < -0.87:
        wall_guard = -(S['trackPos'] + 0.87) * 2.5

    steer = steer_p + steer_d + wall_guard
    return max(-1.0, min(1.0, steer)), steer_p, steer_d


# ═══════════════════════════════════════════════════════════════════════════════
#  THROTTLE
# ═══════════════════════════════════════════════════════════════════════════════

def compute_throttle(S, R, target_speed):
    """
    Direct (non-incremental) throttle for maximum responsiveness.

    Logic:
      - Below target speed : throttle = 0.50 + 0.50 * relative_gap
        (floor at 0.50 avoids timid acceleration)
      - Above target speed : bleed off proportionally to the overshoot
      - Traction circle cap: throttle ≤ 1.0 − |steer| * 1.4
        (preserves lateral grip when turning)
      - Below 10 km/h      : full throttle (launch / recovery)
      - RPM limit          : cut throttle above 18 500 rpm
    """
    speed = S['speedX']

    if speed < target_speed:
        gap   = (target_speed - speed) / max(1.0, target_speed)
        accel = 0.50 + 0.50 * gap
    else:
        overshoot = (speed - target_speed) / max(1.0, target_speed)
        accel = max(0.0, R['accel'] - 0.35 * overshoot)

    # Traction circle — cap by steering angle
    accel = min(accel, 1.0 - abs(R['steer']) * 1.4)

    if speed < 10:
        accel = 1.0              # Full power at launch or near-standstill

    if S['rpm'] > 18500:
        accel = 0.0              # Rev limiter

    return max(0.0, min(1.0, accel))


# ═══════════════════════════════════════════════════════════════════════════════
#  BRAKING
# ═══════════════════════════════════════════════════════════════════════════════

def compute_brake(S, target_speed):
    """
    Proportional brake pressure based on the speed overshoot.

    Divisor of 15 (vs 20 previously) → more aggressive deceleration
    for the same speed gap.
    Emergency brake if yaw angle exceeds threshold (spin detected).
    """
    speed = S['speedX']

    if speed < 15.0:
        return 0.0              # No braking at very low speed (avoid stalling)

    if speed > target_speed:
        pressure = (speed - target_speed) / 15.0
        return max(0.0, min(1.0, pressure))

    return 0.3 if abs(S['angle']) > EMERGENCY_BRAKE_YAW else 0.0


# ═══════════════════════════════════════════════════════════════════════════════
#  GEARBOX
# ═══════════════════════════════════════════════════════════════════════════════

def compute_gear(S):
    """Static RPM-threshold gear shifting."""
    gear = S['gear']
    rpm  = S['rpm']

    if gear < 1:
        return 1

    if rpm > 18500 and gear < 7:
        gear += 1
    elif rpm < 9000 and gear > 1:
        gear -= 1

    return gear


# ═══════════════════════════════════════════════════════════════════════════════
#  TRACTION CONTROL
# ═══════════════════════════════════════════════════════════════════════════════

def traction_control(S, accel):
    """
    Compare rear driven-wheel spin to front free-rolling speed.
    Cut throttle when slip exceeds threshold.
    Threshold is wider off-track to allow extraction from gravel/grass.
    """
    if TRACTION_CONTROL_ON:
        spin      = ((S['wheelSpinVel'][2] + S['wheelSpinVel'][3])
                     - (S['wheelSpinVel'][0] + S['wheelSpinVel'][1]))
        threshold = 10.0 if abs(S.get('trackPos', 0)) > 1.0 else 2.0
        if spin > threshold:
            accel -= 0.1
    return max(0.0, accel)


# ═══════════════════════════════════════════════════════════════════════════════
#  STATE MACHINE TRANSITIONS
# ═══════════════════════════════════════════════════════════════════════════════

def update_state(c, alpha_deg, max_dist):
    """
    Advance the driving-phase state machine based on sensor readings.

    Transition rules:
      STRAIGHT → APPROACH  : significant corner angle detected ahead
      APPROACH → TURN_IN   : corner close enough (max_dist dropping)
      APPROACH → STRAIGHT  : corner vanished (false positive)
      TURN_IN  → APEX      : distance at minimum OR distance rebounding
      APEX     → EXIT      : track opening back up
      EXIT     → STRAIGHT  : track straight and clear
      EXIT     → APPROACH  : immediate next corner detected

    c.phase_timer resets to 0 on every transition.
    c.turn_sign is locked at corner detection and held until EXIT ends.
    c.prev_max_dist enables apex detection via distance rebound.
    """
    prev_state = c.state
    timer      = c.phase_timer

    if c.state == STATE_STRAIGHT:
        if abs(alpha_deg) > CORNER_THRESHOLD_DEG and max_dist > DIST_TURN_IN:
            c.state      = STATE_APPROACH
            c.turn_sign  = 1.0 if alpha_deg > 0 else -1.0
            c.phase_timer = 0

    elif c.state == STATE_APPROACH:
        if timer > PHASE_MIN_FRAMES:
            if max_dist < DIST_TURN_IN:
                c.state      = STATE_TURN_IN
                c.phase_timer = 0
            elif abs(alpha_deg) < CORNER_THRESHOLD_DEG / 2.0:
                c.state      = STATE_STRAIGHT
                c.phase_timer = 0

    elif c.state == STATE_TURN_IN:
        if timer > PHASE_MIN_FRAMES:
            if max_dist < DIST_APEX:
                c.state      = STATE_APEX
                c.phase_timer = 0
            elif max_dist > c.prev_max_dist + 6.0 and timer > 25:
                # Distance rebounding without reaching DIST_APEX → apex passed
                c.state      = STATE_APEX
                c.phase_timer = 0

    elif c.state == STATE_APEX:
        if timer > PHASE_MIN_FRAMES:
            if max_dist > DIST_TURN_IN or abs(alpha_deg) < CORNER_THRESHOLD_DEG:
                c.state      = STATE_EXIT
                c.phase_timer = 0

    elif c.state == STATE_EXIT:
        if timer > PHASE_MIN_FRAMES:
            if abs(alpha_deg) < CORNER_THRESHOLD_DEG / 2.0 and max_dist > DIST_APPROACH:
                c.state      = STATE_STRAIGHT
                c.phase_timer = 0
            elif abs(alpha_deg) > CORNER_THRESHOLD_DEG and timer > 30:
                c.state      = STATE_APPROACH
                c.turn_sign  = 1.0 if alpha_deg > 0 else -1.0
                c.phase_timer = 0

    # ── Terminal output on state change ──────────────────────────────────────
    if c.state != prev_state:
        side = "LEFT" if c.turn_sign > 0 else "RIGHT"
        print(
            f"[STATE] {prev_state:9s} → {c.state:9s}"
            f" | corner={side}"
            f" | alpha={alpha_deg:+.1f}°"
            f" | dist={max_dist:.1f}m"
            f" | step={c.step}"
        )

    c.phase_timer  += 1
    c.prev_max_dist = max_dist


# ═══════════════════════════════════════════════════════════════════════════════
#  TELEMETRY — CSV EXPORT
# ═══════════════════════════════════════════════════════════════════════════════

CSV_HEADER = (
    "step;speed;target_speed;track_pos;target_pos;"
    "yaw;steer;throttle;brake;"
    "slip;skid;gear;rpm;"
    "steer_p;steer_d;"
    "wheelspin;understeer;state\n"
)

def save_telemetry(c):
    """
    Write the telemetry buffer to a CSV file.

    If a crash was detected, save a window of 300 frames before and 100 frames
    after the incident so the data captures the run-up to the off.
    Otherwise save the last 400 frames (recent clean lap data).
    """
    if c.crash_step != -1:
        start = max(0, c.crash_step - 300)
        end   = min(len(c.telemetry_buffer), c.crash_step + 100)
        rows  = c.telemetry_buffer[start:end]
        print(f"[TELEMETRY] Crash at step {c.crash_step} — saving window [{start}:{end}]")
    else:
        rows = c.telemetry_buffer[-400:]
        print(f"[TELEMETRY] Clean session — saving last {len(rows)} rows")

    try:
        with open(c.telemetry_path, "w") as f:
            f.write(CSV_HEADER)
            for row in rows:
                f.write(row['line'])
        print(f"[TELEMETRY] Saved → {c.telemetry_path}")
    except Exception as e:
        print(f"[TELEMETRY] Write error: {e}")


# ═══════════════════════════════════════════════════════════════════════════════
#  MAIN DRIVING LOOP — called every game tick
# ═══════════════════════════════════════════════════════════════════════════════

def drive(c):
    """
    Orchestrator called once per simulator tick (~50 Hz).
    Reads sensors → updates state machine → computes controls → logs telemetry.
    """
    S, R = c.S.d, c.R.d

    # ── Track analysis ────────────────────────────────────────────────────────
    distances = S['track']
    max_dist  = max(distances)

    # Open-corridor direction: average sensor angle where range ≥ 90 % of max
    open_angles = [SENSOR_ANGLES[i] for i, d in enumerate(distances)
                   if d >= max_dist * 0.90]
    alpha_deg = sum(open_angles) / len(open_angles) if open_angles else 0.0
    alpha_rad = alpha_deg * math.pi / 180.0

    # Turn radius from the geometric chord formula
    if abs(alpha_rad) > 0.05:
        turn_radius = abs(max_dist / (2.0 * math.sin(alpha_rad)))
    else:
        turn_radius = 1000.0        # Straight → very large radius
    turn_radius = max(15.0, turn_radius)

    # ── State machine ─────────────────────────────────────────────────────────
    update_state(c, alpha_deg, max_dist)

    # ── Lateral target with exponential smoothing ─────────────────────────────
    raw_target   = get_target_lateral(c.state, c.turn_sign, c.phase_timer)
    smooth_alpha = SMOOTH_STRAIGHT if c.state == STATE_STRAIGHT else SMOOTH_CORNER
    c.target_pos_filtered = (smooth_alpha * raw_target
                             + (1.0 - smooth_alpha) * c.target_pos_filtered)

    # ── Control outputs ───────────────────────────────────────────────────────
    target_speed = compute_target_speed(max_dist, turn_radius)

    R['steer'], c.steer_p, c.steer_d = compute_steering(S, c.target_pos_filtered)
    R['accel']                        = compute_throttle(S, R, target_speed)
    R['brake']                        = compute_brake(S, target_speed)
    R['accel']                        = traction_control(S, R['accel'])
    R['gear']                         = compute_gear(S)

    # ── Recovery: reverse + relaunch ─────────────────────────────────────────
    if abs(S.get('speedX', 0)) < 2.0 and S.get('rpm', 0) > 3000 and c.step > 100:
        c.stuck_timer += 1
    else:
        if c.stuck_timer > 0:
            c.stuck_timer = 0

    if c.stuck_timer > 150:
        c.reverse_timer  = 90
        c.relaunch_timer = 0
        c.stuck_timer    = -300    # 300-frame immunity to prevent re-trigger
        print(f"[STUCK] Reverse triggered at step={c.step}")

    if c.reverse_timer > 0:
        R['gear']  = -1
        R['accel'] = 1.0
        R['brake'] = 0.0
        R['steer'] = -S.get('angle', 0)   # Counter-steer to realign
        c.reverse_timer -= 1
        if c.reverse_timer < 70 and abs(S.get('angle', 0)) < 0.15:
            c.reverse_timer = 0            # Early exit if already aligned
        if c.reverse_timer == 0:
            c.relaunch_timer = 80
            print(f"[RELAUNCH] Full-throttle relaunch at step={c.step}")

    elif c.relaunch_timer > 0:
        R['gear']  = 1
        R['accel'] = 1.0
        R['brake'] = 0.0
        c.relaunch_timer -= 1

    # ── Telemetry ─────────────────────────────────────────────────────────────
    slip = 0.0
    skid = 0.0
    if 'wheelSpinVel' in S and len(S['wheelSpinVel']) == 4:
        slip = ((S['wheelSpinVel'][2] + S['wheelSpinVel'][3])
                - (S['wheelSpinVel'][0] + S['wheelSpinVel'][1]))
        if S['wheelSpinVel'][0] != 0:
            skid = 0.5555555555 * S['speedX'] / S['wheelSpinVel'][0] - 0.66124

    wheelspin  = 1 if abs(slip) > 5.0 or abs(skid) > 0.5 else 0
    understeer = 1 if (abs(R['steer']) + R['accel'] + R['brake']) < 0.35 \
                      and S.get('speedX', 0) > 30 else 0

    log_line = (
        f"{c.step};"
        f"{S.get('speedX', 0):.2f};{target_speed:.2f};"
        f"{S.get('trackPos', 0):.4f};{c.target_pos_filtered:.4f};"
        f"{S.get('angle', 0):.4f};"
        f"{R['steer']:.4f};{R['accel']:.4f};{R['brake']:.4f};"
        f"{slip:.2f};{skid:.4f};{R['gear']};{S.get('rpm', 0):.0f};"
        f"{c.steer_p:.4f};{c.steer_d:.4f};"
        f"{wheelspin};{understeer};{c.state}\n"
    )

    c.telemetry_buffer.append({'line': log_line})

    if abs(S.get('trackPos', 0)) > 1.0 and c.crash_step == -1:
        c.crash_step = c.step

    c.step += 1


# ═══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    C = Client(p=3001)

    # ── Output paths ──────────────────────────────────────────────────────────
    C.telemetry_dir  = r"C:\Users\augus\SynologyDrive\Documents\ESILV\Ved\torcs"
    C.telemetry_path = os.path.join(C.telemetry_dir, "telemetry.csv")

    # ── Telemetry buffer & session counters ───────────────────────────────────
    C.telemetry_buffer = []
    C.crash_step       = -1
    C.step             = 0
    C.stuck_timer      = 0
    C.reverse_timer    = 0
    C.relaunch_timer   = 0

    # ── State machine ─────────────────────────────────────────────────────────
    C.state            = STATE_STRAIGHT
    C.turn_sign        = 0.0      # +1 left / −1 right / 0 undecided
    C.phase_timer      = 0
    C.prev_max_dist    = 200.0

    # ── Lateral target filter ─────────────────────────────────────────────────
    C.target_pos_filtered = 0.0   # Starts at track centre

    # ── Steering decomposition (telemetry) ────────────────────────────────────
    C.steer_p = 0.0
    C.steer_d = 0.0

    print("=" * 60)
    print("  TORCS Autopilot — Racing Line: Outside → Apex → Outside")
    print("  States: STRAIGHT | APPROACH | TURN_IN | APEX | EXIT")
    print("=" * 60)

    # ── Ensure output directory exists ────────────────────────────────────────
    if not os.path.exists(C.telemetry_dir):
        try:
            os.makedirs(C.telemetry_dir)
        except Exception as e:
            print(f"[ERROR] Could not create telemetry directory: {e}")

    # ── Main loop ─────────────────────────────────────────────────────────────
    for _ in range(C.maxSteps, 0, -1):
        C.get_servers_input()
        if not C.so:
            break
        drive(C)
        C.respond_to_server()

    # ── Session end ───────────────────────────────────────────────────────────
    if C.so:
        C.shutdown()

    save_telemetry(C)