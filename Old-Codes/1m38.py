import math
import os
from torcs_base import Client

# Vitesse maximale absolue
TARGET_SPEED = 300
# Angle au-delà duquel un coup de frein d'urgence est donné
BRAKE_THRESHOLD = 0.9
ENABLE_TRACTION_CONTROL = True

# Angles balayés par les 19 télémètres virtuels du véhicule pour détecter le bord de la piste carrossable
SENSOR_ANGLES = [-90, -75, -60, -45, -30, -20, -15, -10, -5, 0, 5, 10, 15, 20, 30, 45, 60, 75, 90]

def calculate_target_speed(S):
    """
    Calcule le rayon de courbure géométrique du virage à venir.
    Déduit la vitesse limite physique de passage en courbe avec l'adhérence mu.
    """
    distances = S['track']
    max_dist = max(distances)
    
    angles_ouverts = [SENSOR_ANGLES[i] for i, dist in enumerate(distances) if dist >= max_dist * 0.9]
    alpha_deg = sum(angles_ouverts) / len(angles_ouverts) if angles_ouverts else 0.0
    alpha = alpha_deg * math.pi / 180.0
    
    if abs(alpha) > 0.05:
        R = abs(max_dist / (2.0 * math.sin(alpha)))
    else:
        R = 1000.0
    R = max(15.0, R)
    
    mu = 1.6 
    g = 9.81
    
    v_apex_ms = math.sqrt(mu * g * R)
    
    if R > 60.0:
        v_apex_ms *= 1.15
        
    # Coefficient de sécurité augmenté pour un passage en courbe plus rapide
    coef_securite = 0.90
    v_apex_ms *= coef_securite
    
    dec_max = 9.0 
    dist_freinage = max(0.0, max_dist - 20.0) 
    
    v_safe_ms = math.sqrt(v_apex_ms**2 + 2.0 * dec_max * dist_freinage)
    v_safe_kmh = v_safe_ms * 3.6
    
    return min(TARGET_SPEED, v_safe_kmh)

def calculate_steering(S):
    """
    Recherche le point de corde géométrique.
    Déporte la voiture à l'opposé du virage en approche puis plonge à l'intérieur.
    Utilise un contrôleur PD Proportionnel-Dérivé pour la stabilité avec bords fluides.
    """
    distances = S['track']
    max_dist = max(distances)
    
    angles_ouverts = [SENSOR_ANGLES[i] for i, dist in enumerate(distances) if dist >= max_dist * 0.9]
    alpha_deg = sum(angles_ouverts) / len(angles_ouverts) if angles_ouverts else 0.0
    
    dist_factor = (max_dist - 65.0) / 35.0  
    dist_factor = max(-1.0, min(1.0, dist_factor))
    
    alpha_factor = 0.0
    if abs(alpha_deg) > 2.0:
        sign = 1.0 if alpha_deg > 0 else -1.0
        alpha_factor = sign * min(1.0, (abs(alpha_deg) - 2.0) / 10.0)
        
    # On amplifie l'entrée mais on bloque la cible avant les vibreurs
    target_pos = -0.6 * alpha_factor * dist_factor
    target_pos = max(-0.8, min(0.8, target_pos))
    
    erreur_pos = S['trackPos'] - target_pos
    P_gain = 0.4    
    D_gain = 25.0 / math.pi 
    
    steer_bords = -(erreur_pos * P_gain)
    steer_angle = S['angle'] * D_gain
    steer_anticipation = (alpha_deg / 180.0) * 0.8
    
    # Répulsion douce et continue si la voiture s'approche de l'herbe
    marge_bord = 0.0
    if S['trackPos'] > 0.85:
        marge_bord = -(S['trackPos'] - 0.85) * 2.0
    elif S['trackPos'] < -0.85:
        marge_bord = -(S['trackPos'] + 0.85) * 2.0
        
    steer = steer_anticipation + steer_bords + steer_angle + marge_bord
    
    return max(-1.0, min(1.0, steer)), steer_anticipation, steer_bords, steer_angle

def calculate_throttle(S, R):
    """
    Accélération continue avec bridage dynamique lié à l'angle du volant.
    """
    vitesse_cible = calculate_target_speed(S)
    vitesse_courante = S['speedX']
    
    if vitesse_courante < vitesse_cible:
        marge = (vitesse_cible - vitesse_courante) / max(1.0, vitesse_cible)
        accel = R['accel'] + (0.2 * marge)
    else:
        accel = R['accel'] - 0.2
        
    steer_abs = abs(R['steer'])
    accel = min(accel, 1.0 - (steer_abs * 1.5))
    
    if S['speedX'] < 10:
        accel += 1.0 / (S['speedX'] + 0.1)
        
    if S['rpm'] > 18500:
        accel = 0.0
        
    return max(0.0, min(1.0, accel))

def apply_brakes(S):
    """
    Anticipation lissée du freinage proportionnelle à l'écart de vitesse.
    """
    vitesse_cible = calculate_target_speed(S)
    vitesse_courante = S['speedX']
    
    if vitesse_courante < 15.0:
        return 0.0
    
    if vitesse_courante > vitesse_cible:
        brake_force = (vitesse_courante - vitesse_cible) / 20.0
        return max(0.0, min(1.0, brake_force))
        
    return 0.3 if abs(S['angle']) > BRAKE_THRESHOLD else 0.0

def shift_gears(S):
    """
    Algorithme de changement de vitesse basique.
    Monte un rapport quand le régime moteur dépasse 18500 et le descend sous 9000.
    """
    gear = S['gear']
    rpm = S['rpm']

    if gear < 1:
        return 1

    if rpm > 18500 and gear < 7:
        gear += 1
    elif rpm < 9000 and gear > 1:
        gear -= 1

    return gear

def traction_control(S, accel):
    """
    Compare la vitesse de rotation des roues arrière motrices aux roues avant directrices.
    Baisse l'accélération demandée si le patinage dépasse une certaine limite pour retrouver l'adhérence.
    """
    if ENABLE_TRACTION_CONTROL:
        spin = (S['wheelSpinVel'][2] + S['wheelSpinVel'][3]) - (S['wheelSpinVel'][0] + S['wheelSpinVel'][1])
        if spin > 2.0:
            accel -= 0.1
    return max(0.0, accel)

def drive_modular(c):
    """
    Fonction chef d'orchestre appelée à chaque image du jeu.
    Appelle les algorithmes de pilotage, affecte les commandes, calcule le dérapage et enregistre la télémétrie.
    """
    S, R = c.S.d, c.R.d
    
    R['steer'], c.steer_anticipation, c.steer_bords, c.steer_angle = calculate_steering(S)
    R['accel'] = calculate_throttle(S, R)
    R['brake'] = apply_brakes(S)
    R['accel'] = traction_control(S, R['accel'])
    R['gear'] = shift_gears(S)
    
    vitesse_cible = calculate_target_speed(S)
    slip = 0.0
    skid = 0.0
    
    if 'wheelSpinVel' in S and len(S['wheelSpinVel']) == 4:
        slip = (S['wheelSpinVel'][2] + S['wheelSpinVel'][3]) - (S['wheelSpinVel'][0] + S['wheelSpinVel'][1])
        if S['wheelSpinVel'][0] != 0:
            skid = 0.5555555555 * S['speedX'] / S['wheelSpinVel'][0] - 0.66124
            
    derapage = 1 if abs(slip) > 5.0 or abs(skid) > 0.5 else 0
    utilisation_friction = abs(R['steer']) + R['accel'] + R['brake']
    friction_sous_utilisee = 1 if utilisation_friction < 0.35 and S.get('speedX', 0) > 30 else 0
            
    log_line = f"{c.step_count};{S.get('speedX', 0):.2f};{vitesse_cible:.2f};{S.get('trackPos', 0):.4f};{S.get('angle', 0):.4f};{R['steer']:.4f};{R['accel']:.4f};{R['brake']:.4f};{slip:.2f};{skid:.4f};{R['gear']};{S.get('rpm', 0):.0f};{c.steer_anticipation:.4f};{c.steer_bords:.4f};{c.steer_angle:.4f};{derapage};{friction_sous_utilisee}\n"
    
    c.telemetry_buffer.append({
        'line': log_line,
        'speed': S.get('speedX', 0),
        'angle': S.get('angle', 0)
    })
    
    if abs(S.get('trackPos', 0)) > 1.0 and c.crash_step == -1:
        c.crash_step = c.step_count
        
    c.step_count += 1
    return

if __name__ == "__main__":
    C = Client(p=3001)
    
    C.telemetry_dir = r"C:\Users\augus\SynologyDrive\Documents\ESILV\Ved\torcs"
    C.telemetry_path = os.path.join(C.telemetry_dir, "telemetry.csv")
    C.telemetry_buffer = []
    C.crash_step = -1
    C.step_count = 0
    
    if not os.path.exists(C.telemetry_dir):
        try:
            os.makedirs(C.telemetry_dir)
        except Exception as e:
            print(f"Erreur de creation de dossier : {e}")
            
    for step in range(C.maxSteps, 0, -1):
        C.get_servers_input()
        
        if not C.so:
            break
            
        drive_modular(C)
        C.respond_to_server()
        
    if C.so:
        C.shutdown()
        