import math
import os
import socket
import sys
import getopt
import time

# ═══════════════════════════════════════════════════════════════════════════════
#  TORCS CLIENT
# ═══════════════════════════════════════════════════════════════════════════════

DATA_SIZE = 2**17

def _destringify(s):
    if not s: return s
    if type(s) is str:
        try: return float(s)
        except ValueError: return s
    return _destringify(s[0]) if len(s) < 2 else [_destringify(i) for i in s]

class ServerState:
    def __init__(self): self.d = {}
    def parse_server_str(self, s):
        for item in s.strip()[:-1].strip().lstrip('(').rstrip(')').split(')('):
            w = item.split(' ')
            self.d[w[0]] = _destringify(w[1:])

class DriverAction:
    def __init__(self):
        self.d = {'accel':0.2,'brake':0,'clutch':0,'gear':1,'steer':0,
                  'focus':[-90,-45,0,45,90],'meta':0}
    def _clip(self):
        for k in ('steer','brake','accel','clutch'):
            lo, hi = (-1.0,1.0) if k=='steer' else (0.0,1.0)
            self.d[k] = max(lo, min(hi, self.d[k]))
        if self.d['gear'] not in range(-1,8): self.d['gear'] = 0
    def __repr__(self):
        self._clip()
        return ''.join(f"({k} {' '.join(str(x) for x in v) if type(v) is list else '%.3f'%v})"
                       for k,v in self.d.items())

class Client:
    def __init__(self, H=None, p=None, i=None):
        self.host='localhost'; self.port=3001; self.sid='SCR'
        self.maxSteps=100000; self.debug=False
        self._parse_cmd()
        if H: self.host=H
        if p: self.port=p
        if i: self.sid=i
        self.S=ServerState(); self.R=DriverAction(); self.so=None
        self._connect()

    def _parse_cmd(self):
        try: opts,_=getopt.getopt(sys.argv[1:],'H:p:i:m:dh',['host=','port=','id=','steps=','debug'])
        except: return
        for k,v in opts:
            if k in('-H','--host'): self.host=v
            elif k in('-p','--port'): self.port=int(v)
            elif k in('-i','--id'): self.sid=v
            elif k in('-m','--steps'): self.maxSteps=int(v)
            elif k in('-d','--debug'): self.debug=True

    def _connect(self):
        try: self.so=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        except: print('Socket error'); sys.exit(-1)
        self.so.settimeout(1); n=5
        while True:
            try: self.so.sendto(f"{self.sid}(init -45 -19 -12 -7 -4 -2.5 -1.7 -1 -.5 0 .5 1 1.7 2.5 4 7 12 19 45)".encode(),(self.host,self.port))
            except: sys.exit(-1)
            try:
                data,_=self.so.recvfrom(DATA_SIZE)
                if '***identified***' in data.decode(): print(f"Connected port {self.port}"); break
            except:
                print(f"Waiting {self.port}... ({n})"); n-=1
                if n<0: os.system('pkill torcs'); time.sleep(1); os.system('torcs -nofuel -nodamage -nolaptime &'); time.sleep(1); n=5

    def get_servers_input(self):
        if not self.so: return
        while True:
            try: data,_=self.so.recvfrom(DATA_SIZE); data=data.decode()
            except: continue
            if '***identified***' in data: continue
            elif '***shutdown***' in data or '***restart***' in data: self.shutdown(); return
            elif data: self.S.parse_server_str(data); break

    def respond_to_server(self):
        if not self.so: return
        try: self.so.sendto(repr(self.R).encode(),(self.host,self.port))
        except Exception as e: print(f"Send error: {e}"); sys.exit(-1)

    def shutdown(self):
        if not self.so: return
        self.so.close(); self.so=None


# ═══════════════════════════════════════════════════════════════════════════════
#  PHYSICAL CONSTANTS — car1-ow1.xml + corkscrew.xml
# ═══════════════════════════════════════════════════════════════════════════════

MAX_SPEED_KMH       = 300.0
EMERGENCY_BRAKE_YAW = 0.9
TRACTION_CONTROL_ON = True
TORCS_FPS           = 50.0

SENSOR_ANGLES = [-90,-75,-60,-45,-30,-20,-15,-10,-5,0,5,10,15,20,30,45,60,75,90]

CAR_MASS    = 600.0
MU_TIRE     = 1.6      
MU_DYNAMIC  = MU_TIRE * 0.70  
SURF_GRIP   = 1.1      
REV_LIMITER = 18700    
G           = 9.81
FRONT_AREA  = 2.0; RHO_AIR=1.225; FRONT_CLIFT=0.2; REAR_CLIFT=0.7

MU_EFFECTIVE = MU_TIRE * SURF_GRIP   
C_DF = 0.5 * RHO_AIR * FRONT_AREA * (FRONT_CLIFT + REAR_CLIFT)  

A_PHYS_BRAKE = MU_DYNAMIC * G * 0.88   
A_PHYS_ACC   = MU_DYNAMIC * G * 0.55   

CORKSCREW_DIST_MAX_DELTA = 3.0  

def surface_physics(pos):
    a  = abs(pos)
    sf = 1.10 if a<=1.00 else 1.05 if a<=1.167 else 0.90 if a<=2.50 else 0.60
    f  = sf/1.10
    return MU_TIRE*sf, f, A_PHYS_BRAKE*f

def surface_label(pos):
    a = abs(pos)
    return "roadA" if a<=1.00 else "curb" if a<=1.167 else "dirtA" if a<=2.50 else "sand"


# ═══════════════════════════════════════════════════════════════════════════════
#  MACHINE À ÉTATS  STRAIGHT → APPROACH → TURN_IN → EXIT → STRAIGHT
# ═══════════════════════════════════════════════════════════════════════════════

STATE_STRAIGHT = "STRAIGHT"
STATE_APPROACH = "APPROACH"
STATE_TURN_IN  = "TURN_IN"
STATE_EXIT     = "EXIT"
STATE_RECOVERY = "RECOVERY"

CORNER_THRESHOLD_DEG = 8.0
PHASE_MIN_FRAMES = 22

AMP_OUTSIDE = 0.60; AMP_APEX = 0.80; AMP_EXIT = 0.65 
SMOOTH_STRAIGHT = 0.18; SMOOTH_CORNER = 0.10; SMOOTH_APPROACH = 0.06

def circ_ease_out(t):
    t = max(0.0, min(1.0, t))
    return math.sqrt(max(0.0, 2.0*t - t*t))

def circ_ease_in(t):
    t = max(0.0, min(1.0, t))
    return 1.0 - math.sqrt(max(0.0, 1.0 - t*t))

def _dur_approach(max_dist, v, turn_in_threshold):
    return int(max(PHASE_MIN_FRAMES, min(100, max(5.0,max_dist-turn_in_threshold)/max(1.0,v)*TORCS_FPS)))

def _dur_arc(r, a_rad, v, lo=PHASE_MIN_FRAMES, hi=70):
    arc = max(2.0, r*max(0.05,abs(a_rad)))
    return int(max(lo, min(hi, arc/max(1.0,v)*TORCS_FPS))), arc

def update_state(c, alpha_deg, max_dist, speed_ms, turn_radius):
    a_rad = alpha_deg * math.pi / 180.0
    prev  = c.state
    timer = c.phase_timer

    if getattr(c, 'recovery_state', "NONE") != "NONE":
        c.state = STATE_RECOVERY
        c.phase_timer += 1
        c.prev_max_dist = max_dist
        return
        
    if c.state == STATE_RECOVERY:
        c.state = STATE_STRAIGHT
        c.phase_timer = 0
        c.phase_duration = 1

    if abs(c.last_speed_ms) < 3.0/3.6:
        c.phase_timer += 1; c.prev_max_dist = max_dist; return

    # Dynamic distance thresholds based on speed and curvature
    dist_turn_in = max(40.0, min(80.0, speed_ms * 2.0))  # 2 seconds lookahead
    dist_apex = max(25.0, min(60.0, speed_ms * 1.2))     # 1.2 seconds lookahead
    dist_approach = max(60.0, min(100.0, speed_ms * 2.5)) # 2.5 seconds lookahead

    if c.state == STATE_STRAIGHT:
        if abs(alpha_deg) > CORNER_THRESHOLD_DEG and max_dist > dist_turn_in:
            c.state = STATE_APPROACH; c.turn_sign = 1.0 if alpha_deg>0 else -1.0
            c.phase_timer = 0; c.phase_duration = _dur_approach(max_dist, speed_ms, dist_turn_in)
            c.approach_entry_pos = c.target_pos_filtered

    elif c.state == STATE_APPROACH:
        if timer > PHASE_MIN_FRAMES:
            if max_dist < dist_turn_in:
                c.state = STATE_TURN_IN; c.phase_timer = 0
                c.phase_duration, c.exit_arc_m = _dur_arc(turn_radius, a_rad, speed_ms, hi=65)
            elif abs(alpha_deg) < CORNER_THRESHOLD_DEG/2.0 and abs(c.last_track_pos) < 0.90:
                c.state = STATE_STRAIGHT; c.phase_timer = 0; c.phase_duration = 1

    elif c.state == STATE_TURN_IN:
        if timer > PHASE_MIN_FRAMES:
            if max_dist < dist_apex or (max_dist > c.prev_max_dist+6.0 and timer > 25):
                c.state = STATE_EXIT; c.phase_timer = 0
                arc = getattr(c,'exit_arc_m',20.0)
                c.phase_duration = int(max(PHASE_MIN_FRAMES, min(90, arc/max(1.0,speed_ms)*TORCS_FPS)))

    elif c.state == STATE_EXIT:
        if timer > PHASE_MIN_FRAMES:
            force = c.off_track_timer > 80
            if (abs(alpha_deg) < CORNER_THRESHOLD_DEG/2.0 and max_dist > dist_approach) or force:
                c.state = STATE_STRAIGHT; c.phase_timer = 0; c.phase_duration = 1
            elif abs(alpha_deg) > CORNER_THRESHOLD_DEG and timer > 30 and not force:
                c.state = STATE_APPROACH; c.turn_sign = 1.0 if alpha_deg>0 else -1.0
                c.phase_timer = 0; c.phase_duration = _dur_approach(max_dist, speed_ms, dist_turn_in)
                c.approach_entry_pos = c.target_pos_filtered

    c.phase_timer += 1; c.prev_max_dist = max_dist

def get_target_lateral(c):
    t = min(1.0, c.phase_timer/max(1,c.phase_duration)); s = c.turn_sign
    if c.state in (STATE_STRAIGHT, STATE_RECOVERY): return 0.0
    if c.state == STATE_APPROACH: return c.approach_entry_pos + (-s*AMP_OUTSIDE-c.approach_entry_pos)*t
    if c.state == STATE_TURN_IN:  return -s*AMP_OUTSIDE + (s*AMP_APEX-(-s*AMP_OUTSIDE))*circ_ease_out(t)
    if c.state == STATE_EXIT:     return s*AMP_APEX + (-s*AMP_EXIT-s*AMP_APEX)*circ_ease_out(t)
    return 0.0


# ═══════════════════════════════════════════════════════════════════════════════
#  TARGET SPEED
# ═══════════════════════════════════════════════════════════════════════════════

def _v_apex_from_k(k, mu_eff):
    if k < 1e-4: return MAX_SPEED_KMH/3.6
    d = CAR_MASS*k - mu_eff*C_DF
    return math.sqrt((mu_eff*CAR_MASS*G)/d) if d>0.05 else MAX_SPEED_KMH/3.6

def compute_target_speed(distances, max_dist, curvature, mu_eff, a_brake):
    d_b = max(0.0, max_dist-10.0)
    
    def tgt_from_k(k):
        return min(MAX_SPEED_KMH, math.sqrt(max(0.0,_v_apex_from_k(k,mu_eff)**2*0.81 + 2.0*a_brake*d_b))*3.6)
        
    target = tgt_from_k(curvature)
    for i in range(1, 18):
        a_r = abs(SENSOR_ANGLES[i])*math.pi/180.0
        d   = distances[i] if i<len(distances) else 200.0
        if d < 1.0 or a_r < 0.01: continue
        target = min(target, tgt_from_k(2.0*math.sin(a_r)/d))
    return target


# ═══════════════════════════════════════════════════════════════════════════════
#  DIRECTION
# ═══════════════════════════════════════════════════════════════════════════════

def compute_steering(S, target_pos, c):
    P=0.38; D=10.0/math.pi; YA=0.25; MAX_R=0.10; DC=0.20
    c.yaw_f = YA*S['angle'] + (1.0-YA)*c.yaw_f
    err = S['trackPos'] - target_pos
    sp  = -(err*P)
    sd  = max(-DC, min(DC, c.yaw_f*D))
    wp  = S['trackPos']
    wg  = -(wp-0.87)*2.5 if wp>0.87 else -(wp+0.87)*2.5 if wp<-0.87 else 0.0
    raw = max(-1.0, min(1.0, sp+sd+wg))

    # Dampen micro-corrections on straights to avoid left-right oscillation.
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


# ═══════════════════════════════════════════════════════════════════════════════
#  PÉDALES
# ═══════════════════════════════════════════════════════════════════════════════

def compute_pedals(S, R, target_speed, max_dist, max_thr, a_brake):
    speed = S['speedX']
    if speed <= target_speed:
        v, vt  = speed/3.6, target_speed/3.6
        d      = max(1.0, max_dist-5.0)
        raw    = max(0.0, (vt**2-v**2)/(2.0*d*A_PHYS_ACC))
        gap    = max(0.0, (vt-v)/max(1.0,vt))
        acc    = max(raw, 0.50+0.50*gap) if v<vt else raw
        acc    = min(acc, 1.0-abs(R['steer'])*1.4)
        acc    = min(acc, max_thr)
        if speed < 10.0:           acc = max_thr
        if S['rpm'] > REV_LIMITER: acc = 0.0
        return max(0.0, min(1.0, acc)), 0.0
    else:
        if speed < 15.0: return 0.0, 0.0
        brk = (speed-target_speed)/10.0
        if abs(S['angle']) > EMERGENCY_BRAKE_YAW: brk = max(brk, 0.5)
        brk = min(brk, math.sqrt(max(0.0, 1.0-R['steer']**2)))
        return 0.0, max(0.0, min(1.0, brk))


# ═══════════════════════════════════════════════════════════════════════════════
#  BOÎTE + TRACTION CONTROL
# ═══════════════════════════════════════════════════════════════════════════════

def compute_gear(S):
    g, rpm = S['gear'], S['rpm']
    if g < 1: return 1
    spin = 0.0
    if 'wheelSpinVel' in S and len(S['wheelSpinVel'])==4:
        spin = (S['wheelSpinVel'][2]+S['wheelSpinVel'][3])-(S['wheelSpinVel'][0]+S['wheelSpinVel'][1])
    if rpm>REV_LIMITER and g<7 and spin<5.0: return g+1
    if rpm<9000 and g>1: return g-1
    return g

def traction_control(S, accel, max_thr):
    if not TRACTION_CONTROL_ON: return accel
    if 'wheelSpinVel' not in S or len(S['wheelSpinVel'])!=4: return accel
    spin = (S['wheelSpinVel'][2]+S['wheelSpinVel'][3])-(S['wheelSpinVel'][0]+S['wheelSpinVel'][1])
    if spin > (10.0 if abs(S.get('trackPos',0))>1.0 else 2.0): accel -= 0.1
    return max(0.0, min(max_thr, accel))


# ═══════════════════════════════════════════════════════════════════════════════
#  RÉCUPÉRATION (MACHINE A ETATS STRICTE)
# ═══════════════════════════════════════════════════════════════════════════════

def update_recovery(c, S, R, track_pos, speed_x):
    _ang = S.get('angle', 0)
    
    is_wrong_way = math.cos(_ang) < 0.0

    if getattr(c, 'recovery_state', "NONE") == "NONE":
        if is_wrong_way:
            c.stuck_timer += 15
        elif abs(speed_x) < 3.0 and c.step > 100:
            c.stuck_timer += 1
        else:
            c.stuck_timer = 0

        if c.stuck_timer > 30:
            c.recovery_state = "BRAKE"
            c.stuck_timer = 0
            c.recovery_steer_dir = 1.0 if track_pos < 0 else -1.0
            return True

    if c.recovery_state != "NONE":
        
        if c.recovery_state == "BRAKE":
            R['gear'] = 1; R['brake'] = 1.0; R['accel'] = 0.0; R['steer'] = 0.0
            if abs(speed_x) < 1.0: 
                c.recovery_state = "J_TURN"
                c.recovery_timer = 120 
            return True
            
        elif c.recovery_state == "J_TURN":
            R['gear'] = -1
            R['brake'] = 0.0
            R['steer'] = c.recovery_steer_dir
            R['accel'] = 1.0 if speed_x > -25.0 else 0.0
            
            c.recovery_timer -= 1
            is_aligned = math.cos(_ang) > 0.5 
            
            # Anti-stuck rear wall: if unable to reverse (speed stalls near zero) after 40 frames
            if c.recovery_timer < 80 and speed_x > -2.0:
                c.recovery_state = "FWD_TURN"
                c.recovery_timer = 100 # Go forward to unstick the rear
            elif is_aligned or c.recovery_timer <= 0:
                c.recovery_state = "WAIT_FWD"
            return True

        elif c.recovery_state == "FWD_TURN":
            R['gear'] = 1
            R['brake'] = 0.0
            R['steer'] = -c.recovery_steer_dir # Steer opposite to continue the same rotation
            R['accel'] = 0.6 if speed_x < 25.0 else 0.0
            
            c.recovery_timer -= 1
            is_aligned = math.cos(_ang) > 0.5 
            
            # Anti-stuck front wall: if unable to move forward after 40 frames
            if c.recovery_timer < 60 and speed_x < 2.0:
                c.recovery_state = "J_TURN"
                c.recovery_timer = 120 # Go back in reverse
            elif is_aligned or c.recovery_timer <= 0:
                c.recovery_state = "WAIT_FWD"
            return True
            
        elif c.recovery_state == "WAIT_FWD":
            R['gear'] = 1; R['brake'] = 1.0; R['accel'] = 0.0; R['steer'] = 0.0
            if abs(speed_x) < 1.0:
                if math.cos(_ang) > 0.0:
                    c.recovery_state = "NONE" 
                else:
                    c.recovery_state = "J_TURN"
                    c.recovery_timer = 80
                    c.recovery_steer_dir = 1.0 if track_pos < 0 else -1.0
            return True

    # Récupération normale en avançant (hors-piste)
    if abs(track_pos) > 1.0 and c.off_track_timer > 10 and not is_wrong_way:
        abs_speed = abs(speed_x)
        
        target_angle = max(-0.8, min(0.8, -track_pos * 0.4))
        steer_error = _ang - target_angle
        
        while steer_error > math.pi: steer_error -= 2*math.pi
        while steer_error < -math.pi: steer_error += 2*math.pi
        
        if abs_speed > 30.0:
            R['gear']  = 1
            R['brake'] = min(1.0, (abs_speed - 20.0) / 10.0) 
            R['accel'] = 0.0
            R['steer'] = max(-1.0, min(1.0, -steer_error * 1.5))
        else:
            R['gear']  = 1
            R['brake'] = 0.0
            if abs_speed < 15.0:
                R['accel'] = 0.5 
                R['steer'] = max(-0.5, min(0.5, -steer_error * 1.5)) 
            else:
                _dist_to_track = max(0.0, abs(track_pos) - 1.0)
                R['accel'] = min(0.8, _dist_to_track * 0.5 + 0.2)
                R['steer'] = max(-1.0, min(1.0, -steer_error * 2.0))
        return True

    return False

# ═══════════════════════════════════════════════════════════════════════════════
#  TELEMETRY
# ═══════════════════════════════════════════════════════════════════════════════

CSV_HEADER = (
    "step;lap_time;dist_raced;dist_from_start;race_pos;"
    "speed_x;speed_y;speed_z;speed_3d;accel_x;accel_y;accel_z;"
    "target_speed;target_speed_raw;"
    "track_pos;target_pos;raw_target;pos_error;lat_progress;"
    "yaw;yaw_filt;steer;steer_p;steer_d;steer_rate;"
    "throttle;brake;gear;rpm;"
    "wspin_FL;wspin_FR;wspin_RL;wspin_RR;slip_rear;skid;"
    "angle;pitch;roll;z;"
    "max_dist_raw;max_dist;alpha_deg;curvature;turn_radius;"
    "track_s0;track_s1;track_s2;track_s3;track_s4;track_s5;track_s6;"
    "track_s7;track_s8;track_s9;track_s10;track_s11;track_s12;track_s13;"
    "track_s14;track_s15;track_s16;track_s17;track_s18;"
    "surface;mu_eff;a_brake;max_thr;"
    "state;phase_dur;t_phase;turn_sign;"
    "stuck_timer;recovery_state;recovery_timer;"
    "wheelspin;lockup;understeer;off_track;damage;fuel\n"
)

def build_telemetry_row(c, S, R, **kw):
    spd=S.get('speedX',0); sy=S.get('speedY',0); sz=S.get('speedZ',0)
    ws=S.get('wheelSpinVel',[0]*4)
    if len(ws)<4: ws=[0]*4
    FL,FR,RL,RR=ws
    slip=(RL+RR)-(FL+FR); skid=(0.5555555555*spd/FL-0.66124) if FL>0.1 else 0.0
    st=R['steer']; sr=st-c.prev_log_steer; c.prev_log_steer=st
    pos_err=kw['track_pos']-c.target_pos_filtered
    lat=(-c.turn_sign*kw['track_pos'])/AMP_OUTSIDE if c.turn_sign else 0.0
    ts=[f"{kw['distances'][i]:.1f}" if i<len(kw['distances']) else "0.0" for i in range(19)]
    mu_eff,max_thr,a_brake=surface_physics(kw['track_pos'])
    return (
        f"{c.step};{S.get('curLapTime',0):.3f};{S.get('distRaced',0):.1f};"
        f"{S.get('distFromStart',0):.1f};{S.get('racePos',0)};"
        f"{spd:.2f};{sy:.2f};{sz:.2f};{math.sqrt(spd**2+sy**2+sz**2):.2f};"
        f"{S.get('accelX',0):.4f};{S.get('accelY',0):.4f};{S.get('accelZ',0):.4f};"
        f"{kw['target_speed']:.2f};{kw['target_speed_raw']:.2f};"
        f"{kw['track_pos']:.4f};{c.target_pos_filtered:.4f};{kw['raw_target']:.4f};"
        f"{pos_err:.4f};{lat:.3f};"
        f"{S.get('angle',0):.4f};{c.yaw_f:.4f};"
        f"{st:.4f};{c.steer_p:.4f};{c.steer_d:.4f};{sr:.4f};"
        f"{R['accel']:.4f};{R['brake']:.4f};{R['gear']};{S.get('rpm',0):.0f};"
        f"{FL:.2f};{FR:.2f};{RL:.2f};{RR:.2f};{slip:.2f};{skid:.4f};"
        f"{S.get('angle',0):.4f};{S.get('pitch',0):.4f};{S.get('roll',0):.4f};{S.get('z',0):.3f};"
        f"{kw['max_dist_raw']:.1f};{kw['max_dist']:.1f};{kw['alpha_deg']:.2f};"
        f"{kw['curvature']:.5f};{kw['turn_radius']:.1f};"
        f"{';'.join(ts)};"
        f"{surface_label(kw['track_pos'])};{mu_eff:.3f};{a_brake:.2f};{max_thr:.3f};"
        f"{c.state};{c.phase_duration};{kw['t_phase']:.3f};{c.turn_sign:.0f};"
        f"{c.stuck_timer};{c.recovery_state};{c.recovery_timer};"
        f"{1 if abs(slip)>5 or abs(skid)>0.5 else 0};"
        f"{1 if FL<5 and spd>30 else 0};"
        f"{1 if (abs(st)+R['accel']+R['brake'])<0.35 and spd>30 else 0};"
        f"{1 if abs(kw['track_pos'])>1.0 else 0};"
        f"{S.get('damage',0):.0f};{S.get('fuel',0):.2f}\n"
    )

def save_telemetry(c):
    save_path = c.telemetry_path
    print(f"Path to reports : {save_path}")
    try:
        with open(save_path, "w", encoding="utf-8") as f:
            f.write(CSV_HEADER)
            for row in c.telemetry_buffer: 
                f.write(row)
        print("[REPORT] | .csv saved")
    except Exception as e:
        print(f"[REPORT] | .csv error: {e}")


# ═══════════════════════════════════════════════════════════════════════════════
#  DRIVE — centralisateur pur
# ═══════════════════════════════════════════════════════════════════════════════

def drive(c):
    S, R = c.S.d, c.R.d

    distances    = S['track']
    max_dist_raw = max(distances)
    if max_dist_raw <= 0: max_dist_raw = c.prev_max_dist
    max_dist  = min(max_dist_raw, c.prev_max_dist + CORKSCREW_DIST_MAX_DELTA)
    open_a    = [SENSOR_ANGLES[i] for i,d in enumerate(distances) if d>=max_dist*0.90]
    alpha_deg = sum(open_a)/len(open_a) if open_a else 0.0
    alpha_rad = alpha_deg * math.pi / 180.0
    curvature = 2.0*math.sin(abs(alpha_rad))/max(max_dist,1.0)
    turn_radius = max(15.0, abs(max_dist/(2.0*math.sin(abs(alpha_rad)))) if abs(alpha_rad)>0.05 else 500.0)

    track_pos = S.get('trackPos', 0)
    speed_kmh = S.get('speedX', 1.0)
    mu_eff, max_thr, a_brake = surface_physics(track_pos)
    c.last_speed_ms = speed_kmh/3.6; c.last_track_pos = track_pos
    c.off_track_timer = c.off_track_timer+1 if abs(track_pos)>1.0 else 0

    update_state(c, alpha_deg, max_dist, speed_kmh/3.6, turn_radius)
    t_phase    = min(1.0, c.phase_timer/max(1,c.phase_duration))
    raw_target = get_target_lateral(c)
    smooth = SMOOTH_APPROACH if c.state==STATE_APPROACH else SMOOTH_STRAIGHT if c.state==STATE_STRAIGHT else SMOOTH_CORNER
    c.target_pos_filtered = smooth*raw_target + (1.0-smooth)*c.target_pos_filtered

    target_speed_raw = compute_target_speed(distances, max_dist, curvature, mu_eff, a_brake)
    
    if c.state == STATE_APPROACH:
        c.committed_apex_speed = min(getattr(c, 'committed_apex_speed', MAX_SPEED_KMH), target_speed_raw)
        target_speed = c.committed_apex_speed
    elif c.state == STATE_TURN_IN:
        target_speed = min(target_speed_raw, getattr(c, 'committed_apex_speed', MAX_SPEED_KMH))
    elif c.state == STATE_RECOVERY:
        target_speed = 20.0 # Arbitrary speed for local telemetry in recovery mode
    else:
        if abs(track_pos) >= 0.95:
            target_speed = min(target_speed_raw, getattr(c, 'committed_apex_speed', MAX_SPEED_KMH))
        else:
            c.committed_apex_speed = MAX_SPEED_KMH
            target_speed = target_speed_raw

    if abs(track_pos)>1.0 and curvature<1e-4 and speed_kmh>50.0:
        target_speed = max(40.0, speed_kmh*0.70)

    R['steer'], c.steer_p, c.steer_d = compute_steering(S, c.target_pos_filtered, c)
    R['accel'], R['brake']            = compute_pedals(S, R, target_speed, max_dist, max_thr, a_brake)
    R['accel']                        = traction_control(S, R['accel'], max_thr)
    R['gear']                         = compute_gear(S)

    update_recovery(c, S, R, track_pos, speed_kmh)

    c.telemetry_buffer.append(build_telemetry_row(
        c, S, R, distances=distances, track_pos=track_pos,
        target_speed=target_speed, target_speed_raw=target_speed_raw,
        raw_target=raw_target, t_phase=t_phase, alpha_deg=alpha_deg,
        curvature=curvature, turn_radius=turn_radius,
        max_dist=max_dist, max_dist_raw=max_dist_raw,
    ))
    if abs(track_pos)>1.0 and c.crash_step==-1: c.crash_step=c.step
    c.step += 1


# ═══════════════════════════════════════════════════════════════════════════════
#  POINT D'ENTRÉE
# ═══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":

    print(f"-----TORCS Autopilot | VED-----")

    C = Client(p=3001)

    _here   = os.path.dirname(os.path.abspath(__file__))
    
    _report_dir = os.path.join(_here, "report")
    os.makedirs(_report_dir, exist_ok=True)
    C.telemetry_path = os.path.join(_report_dir, "telemetry.csv")

    C.telemetry_buffer=[]; C.crash_step=-1; C.step=0
    C.stuck_timer=0; C.recovery_state="NONE"; C.recovery_timer=0; C.recovery_steer_dir=0.0
    C.state=STATE_STRAIGHT; C.turn_sign=0.0
    C.phase_timer=0; C.phase_duration=1; C.exit_arc_m=0.0
    C.prev_max_dist=200.0; C.approach_entry_pos=0.0; C.off_track_timer=0
    C.target_pos_filtered=0.0
    C.yaw_f=0.0; C.prev_steer=0.0; C.prev_log_steer=0.0; C.steer_p=0.0; C.steer_d=0.0
    C.last_speed_ms=1.0; C.last_track_pos=0.0

    print("Driving...")

    for _ in range(C.maxSteps, 0, -1):
        C.get_servers_input()
        if not C.so: break
        drive(C)
        C.respond_to_server()
    
    save_telemetry(C)
    
    _report = os.path.join(_here, 'report.py')
    if os.path.exists(_report):
        import subprocess
        subprocess.run([sys.executable, _report, C.telemetry_path], timeout=120)
    else:
        print(f"[REPORT] report.py not found at {_report}")