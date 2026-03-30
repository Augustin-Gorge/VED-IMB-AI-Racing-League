import sys, os, math, base64, io, warnings
from datetime import datetime
warnings.filterwarnings('ignore')

import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.gridspec as gridspec
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
import matplotlib.cm as cm

# ── Config ────────────────────────────────────────────────────────────────────
plt.rcParams.update({
    'figure.facecolor': '#0f0f0f', 'axes.facecolor': '#1a1a1a',
    'axes.edgecolor': '#333', 'axes.labelcolor': '#ccc',
    'text.color': '#ccc', 'xtick.color': '#888', 'ytick.color': '#888',
    'grid.color': '#2a2a2a', 'grid.linewidth': 0.5,
    'font.family': 'monospace', 'font.size': 9,
    'lines.linewidth': 1.2, 'axes.titlesize': 10, 'axes.titleweight': 'bold',
})

STATE_COLORS = {
    'STRAIGHT': '#4a9eff', 'APPROACH': '#ffcc00',
    'TURN_IN':  '#ff6b35', 'EXIT':     '#7bed9f',
}
RED = '#ff4444'; GRN = '#7bed9f'; YEL = '#ffcc00'; BLU = '#4a9eff'; GRY = '#555'


# ═══════════════════════════════════════════════════════════════════════════════
#  CHARGEMENT
# ═══════════════════════════════════════════════════════════════════════════════

def load(path):
    df = pd.read_csv(path, sep=';', low_memory=False)
    df.columns = df.columns.str.strip()
    for c in df.columns:
        if c not in ('state', 'surface'):
            df[c] = pd.to_numeric(df[c], errors='coerce')
    df['off_track'] = df['track_pos'].abs() > 1.0
    if 'dist_from_start' in df.columns:
        df['dist'] = df['dist_from_start'].ffill()
    else:
        df['dist'] = df['step'] * 0.5   # fallback
    for _c in ['steer','brake','throttle','slip_rear','skid','wspin_FL','wspin_FR','wspin_RL','wspin_RR']:
        if _c not in df.columns: df[_c] = 0.0
    df['kamm'] = np.sqrt(df['steer']**2 + df['brake']**2)
    df['kamm_over'] = df['kamm'] > 1.0
    df['lat_err'] = (df['track_pos'] - df['target_pos']).abs()
    return df


# ═══════════════════════════════════════════════════════════════════════════════
#  UTILITAIRES
# ═══════════════════════════════════════════════════════════════════════════════

def fig_to_b64(fig):
    buf = io.BytesIO()
    fig.savefig(buf, format='png', dpi=130, bbox_inches='tight', facecolor=fig.get_facecolor())
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode()

def state_segments(df, ax, alpha=0.10):
    """Bandes de fond colorées par état."""
    if 'state' not in df.columns: return
    prev_s, prev_d = None, df['dist'].iloc[0]
    for _, row in df.iterrows():
        if row['state'] != prev_s:
            if prev_s in STATE_COLORS:
                ax.axvspan(prev_d, row['dist'], alpha=alpha,
                           color=STATE_COLORS[prev_s], linewidth=0)
            prev_s, prev_d = row['state'], row['dist']

def off_track_spans(df, ax, alpha=0.15):
    """Zones hors piste en rouge."""
    in_ot = False
    for _, row in df.iterrows():
        if row['off_track'] and not in_ot:
            ot_start = row['dist']; in_ot = True
        elif not row['off_track'] and in_ot:
            ax.axvspan(ot_start, row['dist'], alpha=alpha, color=RED, linewidth=0)
            in_ot = False
    if in_ot:
        ax.axvspan(ot_start, df['dist'].iloc[-1], alpha=alpha, color=RED, linewidth=0)

def colored_line(ax, x, y, c, cmap='RdYlGn', vmin=None, vmax=None, lw=1.5):
    """Ligne colorée selon une valeur continue."""
    pts = np.array([x, y]).T.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, cmap=cmap, norm=Normalize(vmin or c.min(), vmax or c.max()))
    lc.set_array(c); lc.set_linewidth(lw)
    ax.add_collection(lc)
    ax.autoscale()
    return lc

def stat_box(ax, stats_dict, title=''):
    ax.axis('off')
    if title: ax.set_title(title, color=YEL, pad=4)
    y = 0.95; dy = 0.95 / max(len(stats_dict), 1)
    for k, v in stats_dict.items():
        color = GRN if isinstance(v, str) and '✓' in v else (RED if isinstance(v, str) and '✗' in v else '#ccc')
        ax.text(0.02, y, k, transform=ax.transAxes, fontsize=8, color='#888', va='top')
        ax.text(0.98, y, str(v), transform=ax.transAxes, fontsize=8, color=color, va='top', ha='right', fontweight='bold')
        y -= dy


# ═══════════════════════════════════════════════════════════════════════════════
#  PANELS
# ═══════════════════════════════════════════════════════════════════════════════

def panel_speed_trace(df):
    """Speed trace F1 — vitesse + cible, colorée par état + zones hors piste."""
    fig, ax = plt.subplots(figsize=(14, 3.5))
    state_segments(df, ax, alpha=0.08)
    off_track_spans(df, ax)
    ax.plot(df['dist'], df['target_speed'], color=GRY, lw=0.8, ls='--', label='Target')
    lc = colored_line(ax, df['dist'].values, df['speed_x'].values,
                      df['speed_x'].values, cmap='plasma', vmin=0, vmax=300)
    plt.colorbar(lc, ax=ax, label='km/h', fraction=0.015, pad=0.01)
    ax.set_ylabel('Speed (km/h)'); ax.set_xlabel('Distance (m)')
    ax.set_title('SPEED TRACE')
    ax.legend(loc='upper left', fontsize=7)
    ax.grid(True, axis='y')
    _legend_states(ax)
    return fig

def panel_pedals(df):
    """Throttle / Brake / Gear / Steer — style F1 multi-trace."""
    fig, axes = plt.subplots(4, 1, figsize=(14, 7), sharex=True)
    fig.subplots_adjust(hspace=0.05)
    panels = [
        ('throttle', GRN,  'Throttle', (0, 1)),
        ('brake',    RED,  'Brake',    (0, 1)),
        ('gear',     YEL,  'Gear',     (0, 8)),
        ('steer',    BLU,  'Steer',    (-1, 1)),
    ]
    for ax, (col, clr, lbl, ylim) in zip(axes, panels):
        state_segments(df, ax, alpha=0.06)
        off_track_spans(df, ax)
        ax.plot(df['dist'], df[col], color=clr, lw=1.0)
        if col in ('throttle', 'brake'):
            ax.fill_between(df['dist'], 0, df[col], color=clr, alpha=0.3)
        ax.set_ylabel(lbl, fontsize=8); ax.set_ylim(*ylim); ax.grid(True, axis='y')
        if col == 'steer': ax.axhline(0, color=GRY, lw=0.5)
    axes[-1].set_xlabel('Distance (m)')
    fig.suptitle('PEDALS & CONTROLS', y=1.01)
    return fig

def panel_kamm_circle(df):
    """Cercle de Kamm — scatter steer vs brake, coloré par vitesse."""
    fig, axes = plt.subplots(1, 2, figsize=(10, 5))

    # Scatter
    ax = axes[0]
    sc = ax.scatter(df['steer'], df['brake'], c=df['speed_x'],
                    cmap='plasma', s=1, alpha=0.4, vmin=0, vmax=300)
    theta = np.linspace(0, math.pi/2, 100)
    ax.plot(np.cos(theta), np.sin(theta), color=RED, lw=1.5, ls='--', label='Kamm limit')
    ax.set_xlim(-0.1, 1.1); ax.set_ylim(-0.1, 1.1)
    ax.set_xlabel('|Steer|'); ax.set_ylabel('Brake')
    ax.set_title('KAMM CIRCLE (brake vs steer)')
    ax.legend(fontsize=7)
    ax.set_aspect('equal')
    plt.colorbar(sc, ax=ax, label='Speed (km/h)', fraction=0.046)

    # Histogram violations
    ax2 = axes[1]
    over = df[df['kamm_over']]
    ok   = df[~df['kamm_over']]
    ax2.bar(['On-track\nOK', 'On-track\nOver', 'Off-track\nOK', 'Off-track\nOver'],
            [len(ok[~ok['off_track']]), len(over[~over['off_track']]),
             len(ok[ok['off_track']]), len(over[over['off_track']])],
            color=[GRN, RED, BLU, YEL], alpha=0.8)
    ax2.set_title('KAMM VIOLATIONS')
    ax2.set_ylabel('Frames')
    pct = 100*len(over)/max(len(df),1)
    ax2.text(0.5, 0.95, f'{pct:.1f}% frames > Kamm limit',
             transform=ax2.transAxes, ha='center', color=RED if pct>5 else GRN, fontsize=9)
    fig.tight_layout()
    return fig

def panel_lateral(df):
    """Trajectoire latérale vs cible — erreur de ligne."""
    fig, axes = plt.subplots(3, 1, figsize=(14, 7), sharex=True)
    fig.subplots_adjust(hspace=0.05)

    ax = axes[0]
    state_segments(df, ax, alpha=0.08); off_track_spans(df, ax)
    ax.plot(df['dist'], df['track_pos'],    color=BLU, lw=1.0, label='Actual')
    ax.plot(df['dist'], df['target_pos'],   color=YEL, lw=0.8, ls='--', label='Target')
    ax.fill_between(df['dist'], -1, 1, alpha=0.04, color=GRN)   # track bounds
    ax.axhline( 1, color=RED, lw=0.5, ls=':')
    ax.axhline(-1, color=RED, lw=0.5, ls=':')
    ax.set_ylabel('trackPos'); ax.legend(fontsize=7); ax.grid(True, axis='y')
    ax.set_title('LATERAL POSITION & RACING LINE')

    ax2 = axes[1]
    state_segments(df, ax2, alpha=0.06)
    ax2.fill_between(df['dist'], 0, df['lat_err'], color=RED, alpha=0.5, label='|error|')
    ax2.set_ylabel('Pos error'); ax2.grid(True, axis='y'); ax2.legend(fontsize=7)

    ax3 = axes[2]
    state_segments(df, ax3, alpha=0.06)
    ax3.plot(df['dist'], df['steer_p'], color=BLU, lw=0.8, label='P (pos)')
    ax3.plot(df['dist'], df['steer_d'], color=YEL, lw=0.8, label='D (yaw)')
    ax3.axhline(0, color=GRY, lw=0.4)
    ax3.set_ylabel('Steer decomp'); ax3.set_xlabel('Distance (m)')
    ax3.legend(fontsize=7); ax3.grid(True, axis='y')
    return fig

def panel_longitudinal_g(df):
    """G longitudinal et latéral — cercle d'adhérence temporel."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 4.5))

    ax = axes[0]
    state_segments(df, ax, alpha=0.08); off_track_spans(df, ax)
    if 'accel_x' in df.columns:
        ax.plot(df['dist'], df['accel_x'], color=GRN, lw=0.8, label='Long G (accelX)')
    if 'accel_y' in df.columns:
        ax.plot(df['dist'], df['accel_y'], color=RED, lw=0.8, label='Lat G (accelY)')
    ax.axhline(0, color=GRY, lw=0.4)
    ax.set_ylabel('Acceleration (m/s²)'); ax.set_xlabel('Distance (m)')
    ax.set_title('G-FORCES vs DISTANCE'); ax.legend(fontsize=7); ax.grid(True, axis='y')

    ax2 = axes[1]
    if 'accel_x' in df.columns and 'accel_y' in df.columns:
        sc = ax2.scatter(df['accel_y'], df['accel_x'], c=df['speed_x'],
                         cmap='plasma', s=1, alpha=0.3, vmin=0, vmax=300)
        lim = max(df['accel_x'].abs().max(), df['accel_y'].abs().max()) * 1.1
        theta = np.linspace(0, 2*math.pi, 300)
        for r in [5, 10, 15]:
            ax2.plot(r*np.cos(theta), r*np.sin(theta), color=GRY, lw=0.4, ls=':')
        ax2.set_xlim(-lim, lim); ax2.set_ylim(-lim, lim)
        ax2.set_xlabel('Lat G (m/s²)'); ax2.set_ylabel('Long G (m/s²)')
        ax2.set_title('G-G DIAGRAM (traction circle)')
        ax2.set_aspect('equal')
        plt.colorbar(sc, ax=ax2, label='Speed (km/h)', fraction=0.046)
    fig.tight_layout()
    return fig

def panel_braking_points(df):
    """Analyse des points de freinage — vitesse vs target, delta."""
    fig, axes = plt.subplots(2, 1, figsize=(14, 5), sharex=True)
    fig.subplots_adjust(hspace=0.05)

    ax = axes[0]
    state_segments(df, ax, alpha=0.08); off_track_spans(df, ax)
    delta = df['speed_x'] - df['target_speed']
    ax.fill_between(df['dist'], 0, delta.clip(lower=0), color=RED, alpha=0.5, label='Over target')
    ax.fill_between(df['dist'], delta.clip(upper=0), 0, color=GRN, alpha=0.4, label='Under target')
    ax.axhline(0, color=GRY, lw=0.5)
    ax.set_ylabel('Speed − Target (km/h)'); ax.legend(fontsize=7); ax.grid(True, axis='y')
    ax.set_title('BRAKING EFFICIENCY — Speed delta vs target')

    ax2 = axes[1]
    state_segments(df, ax2, alpha=0.06)
    ax2.plot(df['dist'], df['max_dist'], color=BLU, lw=0.8, label='max_dist')
    ax2.plot(df['dist'], df['max_dist_raw'], color=GRY, lw=0.5, ls=':', label='max_dist_raw')
    ax2.axhline(62, color=YEL, lw=0.5, ls='--', label='DIST_TURN_IN')
    ax2.axhline(44, color=RED, lw=0.5, ls='--', label='DIST_APEX')
    ax2.set_ylabel('Sensor distance (m)'); ax2.set_xlabel('Distance (m)')
    ax2.legend(fontsize=7, ncol=4); ax2.grid(True, axis='y')
    return fig

def panel_rpm_gear(df):
    """RPM + vitesses par rapport — style histogramme F1."""
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.5))

    ax = axes[0]
    state_segments(df, ax, alpha=0.08)
    lc = colored_line(ax, df['dist'].values, df['rpm'].values,
                      df['gear'].values, cmap='tab10', vmin=1, vmax=7, lw=1.5)
    ax.axhline(18700, color=RED, lw=0.8, ls='--', label='Rev limiter')
    ax.set_ylabel('RPM'); ax.set_xlabel('Distance (m)')
    ax.set_title('RPM (colored by gear)'); ax.legend(fontsize=7)
    for g in range(1, 8):
        ax.text(df['dist'].iloc[-1]*1.01, 18700*g/7.5, f'G{g}', fontsize=7, color='#666')

    ax2 = axes[1]
    df['gear_int'] = pd.to_numeric(df['gear'],errors='coerce').round().astype('Int64')
    gear_max_speeds = df.groupby('gear_int')['speed_x'].max().reindex(range(1,8))
    bars = ax2.bar(range(1,8), gear_max_speeds.values,
                   color=[cm.tab10(i/7) for i in range(7)], alpha=0.85)
    ax2.set_xlabel('Gear'); ax2.set_ylabel('Max speed (km/h)')
    ax2.set_title('MAX SPEED PER GEAR')
    for bar, val in zip(bars, gear_max_speeds.values):
        if not np.isnan(val):
            ax2.text(bar.get_x()+bar.get_width()/2, val+2, f'{val:.0f}', ha='center', fontsize=8)
    fig.tight_layout()
    return fig

def panel_recovery_timeline(df):
    """Timeline des événements de récupération."""
    fig, axes = plt.subplots(3, 1, figsize=(14, 5), sharex=True)
    fig.subplots_adjust(hspace=0.05)

    t = df['dist']

    ax = axes[0]
    ax.fill_between(t, 0, df['off_track'].astype(int), color=RED, alpha=0.6, step='post', label='Off-track')
    ax.set_ylabel('Off-track'); ax.set_yticks([0,1]); ax.legend(fontsize=7)
    ax.set_title('RECOVERY TIMELINE')

    ax2 = axes[1]
    rev = (df['reverse_timer'] > 0) if 'reverse_timer' in df.columns else (df['recovery_state'] == 2) if 'recovery_state' in df.columns else pd.Series([False]*len(df))
    rel = (df['relaunch_timer'] > 0) if 'relaunch_timer' in df.columns else (df['recovery_state'] == 4) if 'recovery_state' in df.columns else pd.Series([False]*len(df))
    ax2.fill_between(t, 0, rev.astype(int), color=YEL, alpha=0.7, step='post', label='Reverse')
    ax2.fill_between(t, 0, rel.astype(int), color=GRN, alpha=0.7, step='post', label='Relaunch')
    ax2.set_ylabel('Recovery'); ax2.legend(fontsize=7)

    ax3 = axes[2]
    ax3.plot(t, df['stuck_timer'].clip(lower=0), color=BLU, lw=1.0, label='stuck_timer')
    ax3.axhline(150, color=RED, lw=0.5, ls='--', label='Trigger (150)')
    ax3.set_ylabel('Stuck timer'); ax3.set_xlabel('Distance (m)')
    ax3.legend(fontsize=7); ax3.grid(True, axis='y')
    return fig

def panel_wheelspin(df):
    """Wheelspin / traction — glissement roues."""
    fig, axes = plt.subplots(2, 1, figsize=(14, 5), sharex=True)
    fig.subplots_adjust(hspace=0.05)

    ax = axes[0]
    state_segments(df, ax, alpha=0.06); off_track_spans(df, ax)
    ax.plot(df['dist'], df['slip_rear'], color=RED, lw=0.8, label='slip_rear')
    ax.axhline(5, color=YEL, lw=0.5, ls='--', label='TC threshold (5)')
    ax.axhline(-5, color=YEL, lw=0.5, ls='--')
    ax.set_ylabel('Wheel slip (rad/s)'); ax.legend(fontsize=7); ax.grid(True, axis='y')
    ax.set_title('WHEELSPIN & TRACTION CONTROL')

    ax2 = axes[1]
    state_segments(df, ax2, alpha=0.06)
    ax2.plot(df['dist'], df['wspin_FL'], color=BLU, lw=0.7, label='FL')
    ax2.plot(df['dist'], df['wspin_FR'], color=GRN, lw=0.7, label='FR')
    ax2.plot(df['dist'], df['wspin_RL'], color=RED, lw=0.7, label='RL')
    ax2.plot(df['dist'], df['wspin_RR'], color=YEL, lw=0.7, label='RR')
    ax2.set_ylabel('Wheel spin (rad/s)'); ax2.set_xlabel('Distance (m)')
    ax2.legend(fontsize=7, ncol=4); ax2.grid(True, axis='y')
    return fig

def panel_curvature_sensor(df):
    """Courbure détectée par les capteurs — visualisation des 19 capteurs."""
    fig, axes = plt.subplots(2, 1, figsize=(14, 5), sharex=True)
    fig.subplots_adjust(hspace=0.05)

    ax = axes[0]
    sensor_cols = [c for c in df.columns if c.startswith('track_s') and c[7:].isdigit()]
    sensor_cols.sort(key=lambda c: int(c[7:]))
    if sensor_cols:
        vals = df[sensor_cols].values
        im = ax.imshow(vals.T, aspect='auto', origin='lower',
                       extent=[df['dist'].min(), df['dist'].max(), -90, 90],
                       cmap='viridis_r', vmin=0, vmax=100, interpolation='nearest')
        plt.colorbar(im, ax=ax, label='Dist (m)', fraction=0.015)
        ax.set_ylabel('Sensor angle (°)')
        ax.set_title('SENSOR HEATMAP (19 range-finders, dark = obstacle close)')

    ax2 = axes[1]
    state_segments(df, ax2, alpha=0.08); off_track_spans(df, ax2)
    ax2.plot(df['dist'], df['curvature']*1000, color=RED, lw=0.8, label='k×1000')
    ax2.plot(df['dist'], df['alpha_deg']/10, color=BLU, lw=0.8, label='alpha/10 (°)')
    ax2.axhline(0, color=GRY, lw=0.4)
    ax2.set_ylabel('Curvature / Alpha'); ax2.set_xlabel('Distance (m)')
    ax2.legend(fontsize=7); ax2.grid(True, axis='y')
    return fig

def panel_circuit_map(df):
    """
    Reconstruction schématique du circuit depuis distFromStart + trackPos.
    Approximation : projette trackPos perpendiculairement à l'axe de progression.
    """
    fig, axes = plt.subplots(1, 2, figsize=(13, 6))

    # ── Carte 2D ──────────────────────────────────────────────────────────────
    ax = axes[0]
    d   = df['dist_from_start'].values if 'dist_from_start' in df.columns else df['dist'].values
    pos = df['track_pos'].values
    spd = df['speed_x'].values

    # Reconstruction de la position absolue approximative
    # On suppose que le circuit est à peu près linéaire ; trackPos est la déviation latérale
    # La longueur du circuit Corkscrew ≈ 3600m
    TRACK_LEN = d.max() if d.max() > 100 else 3600.0
    angle_prog = d / TRACK_LEN * 2 * math.pi   # angle de progression sur le circuit

    # Position cartésienne : projection sur un ovale grossier
    R_BASE = 400
    x_center = (R_BASE + pos * 6) * np.cos(angle_prog)
    y_center = (R_BASE + pos * 6) * np.sin(angle_prog) * 0.6   # aplatissement

    sc = ax.scatter(x_center, y_center, c=spd, cmap='RdYlGn', s=1.5,
                    alpha=0.5, vmin=0, vmax=300)
    plt.colorbar(sc, ax=ax, label='Speed (km/h)', fraction=0.046)
    ax.set_aspect('equal'); ax.set_title('CIRCUIT MAP (trajectory colored by speed)')
    ax.set_xlabel('X approx (m)'); ax.set_ylabel('Y approx (m)')
    # Marquer zones off-track
    ot_mask = df['off_track'].values
    ax.scatter(x_center[ot_mask], y_center[ot_mask], c=RED, s=3, alpha=0.6, zorder=5, label='Off-track')
    ax.legend(fontsize=7)

    # ── trackPos distribution ─────────────────────────────────────────────────
    ax2 = axes[1]
    for st, clr in STATE_COLORS.items():
        sub = df[df['state']==st]['track_pos'] if 'state' in df.columns else pd.Series()
        if len(sub):
            ax2.hist(sub, bins=80, color=clr, alpha=0.5, label=st, density=True)
    ax2.axvline(-1, color=RED, lw=1, ls='--'); ax2.axvline(1, color=RED, lw=1, ls='--')
    ax2.axvline( 0, color=GRY, lw=0.5)
    ax2.set_xlabel('trackPos'); ax2.set_ylabel('Density')
    ax2.set_title('LATERAL POSITION DISTRIBUTION by state')
    ax2.legend(fontsize=7)
    fig.tight_layout()
    return fig


# ═══════════════════════════════════════════════════════════════════════════════
#  DIAGNOSTICS TEXTE
# ═══════════════════════════════════════════════════════════════════════════════

def compute_diagnostics(df):
    n = len(df)
    ot = df['off_track']
    over = df['kamm_over']
    
    rev  = (df['reverse_timer'] > 0) if 'reverse_timer' in df.columns else (df['recovery_state'] == 2) if 'recovery_state' in df.columns else pd.Series([False]*n)
    rel  = (df['relaunch_timer'] > 0) if 'relaunch_timer' in df.columns else (df['recovery_state'] == 4) if 'recovery_state' in df.columns else pd.Series([False]*n)

    first_ot = df[ot]['step'].iloc[0] if ot.any() else None
    first_ot_state = df[ot]['state'].iloc[0] if ot.any() else '—'
    first_ot_speed = df[ot]['speed_x'].iloc[0] if ot.any() else 0
    max_pos = df['track_pos'].abs().max()
    lap_t = df['lap_time'].max()
    dist  = df['dist_raced'].max() if 'dist_raced' in df.columns else 0

    # Vitesse par état
    spd_by_state = {st: df[df['state']==st]['speed_x'].mean() if 'state' in df.columns else 0
                    for st in ['STRAIGHT','APPROACH','TURN_IN','EXIT']}

    # Freinage: delta moyen speed-target en APPROACH/TURN_IN
    brake_states = df[df['state'].isin(['APPROACH','TURN_IN'])] if 'state' in df.columns else df
    over_speed_mean = (brake_states['speed_x'] - brake_states['target_speed']).clip(lower=0).mean()
    brake_eff = 100*(1 - over_speed_mean/max(brake_states['speed_x'].mean(),1))

    n_reverse = len(rev[rev].index.tolist())

    diag = {
        'session': {
            'Frames totaux': n,
            'Lap time': f"{lap_t:.2f}s",
            'Distance parcourue': f"{dist:.0f}m",
            'Frames hors piste': f"{ot.sum()} ({100*ot.mean():.1f}%)",
            'Premier OT': f"step {first_ot} ({first_ot_state}, {first_ot_speed:.0f} km/h)" if first_ot else "✓ Aucun",
            'trackPos max': f"{max_pos:.3f}" + (' ✗' if max_pos>2 else ' ✓'),
            'Violations Kamm': f"{over.sum()} ({100*over.mean():.1f}%)" + (' ✗' if over.mean()>0.05 else ' ✓'),
            'Frames reverse': f"{rev.sum()}",
            'Frames relaunch': f"{rel.sum()}",
        },
        'performance': {
            'Vitesse max': f"{df['speed_x'].max():.1f} km/h",
            'Vitesse moy': f"{df['speed_x'].mean():.1f} km/h",
            'Vit moy STRAIGHT': f"{spd_by_state['STRAIGHT']:.1f} km/h",
            'Vit moy TURN_IN': f"{spd_by_state['TURN_IN']:.1f} km/h",
            'Over-speed moy (APPROACH+TI)': f"{over_speed_mean:.1f} km/h" + (' ✗' if over_speed_mean>10 else ' ✓'),
            'Efficacité freinage': f"{brake_eff:.1f}%" + (' ✓' if brake_eff>85 else ' ✗'),
            'Slip max (wheelspin)': f"{df['slip_rear'].abs().max():.1f} rad/s",
        },
        'issues': _find_issues(df),
    }
    return diag

def _find_issues(df):
    """Détecte automatiquement les problèmes récurrents."""
    issues = []
    ot = df['off_track']
    over = df['kamm_over']

    if ot.mean() > 0.10:
        issues.append(('CRITIQUE', f"Hors piste {100*ot.mean():.0f}% du temps"))
    if over.mean() > 0.05:
        issues.append(('PHYSIQUE', f"Cercle de Kamm violé {100*over.mean():.0f}% du temps → grip perdu"))

    # Vitesse à l'entrée de TURN_IN
    ti = df[df['state']=='TURN_IN']['speed_x'] if 'state' in df.columns else pd.Series()
    if len(ti) and ti.mean() > 180:
        issues.append(('TRAJECTOIRE', f"Entrée TURN_IN trop rapide : {ti.mean():.0f} km/h moy (cible <150)"))

    # Over-speed en APPROACH
    if 'state' in df.columns:
        app = df[df['state']=='APPROACH']
        over_app = (app['speed_x'] - app['target_speed']).clip(lower=0)
        if over_app.mean() > 15:
            issues.append(('FREIN', f"Vitesse > cible de {over_app.mean():.0f} km/h en APPROACH → freinage insuffisant"))

    # Steer saturé hors piste
    ot_steer = df[df['off_track']]['steer'].abs() if ot.any() else pd.Series()
    if len(ot_steer) and ot_steer.mean() > 0.8:
        issues.append(('RECOVERY', f"Steer saturé ({ot_steer.mean():.2f}) hors piste → machine à états bloquée"))

    # Reverse inefficace (pos empire pendant reverse)
    rev_frames = df[df['reverse_timer']>0] if 'reverse_timer' in df.columns else df[df['recovery_state'] == 2] if 'recovery_state' in df.columns else pd.DataFrame()
    if len(rev_frames) > 10:
        # Amélioration = |trackPos| diminue pendant le reverse
        pos_trend = rev_frames['track_pos'].abs().diff().mean()
        if pos_trend > 0.005:
            issues.append(('REVERSE', f"Reverse inefficace : |pos| empire (+{pos_trend:.4f}/fr)"))
        elif pos_trend < -0.005:
            pass  # bon : la voiture revient vers la piste
        else:
            issues.append(('REVERSE', f"Reverse neutre — vérifier l\'angle de sortie"))

    # Vitesse latérale à l'entrée d'APPROACH
    if 'state' in df.columns:
        df2 = df.copy()
        df2['lat_vel'] = df2['track_pos'].diff() * 50 * 6  # m/s
        entries = []
        for i in range(1, len(df2)):
            if df2.iloc[i]['state']=='APPROACH' and df2.iloc[i-1]['state']!='APPROACH':
                entries.append(df2.iloc[i]['lat_vel'])
        if entries:
            avg_lat = np.mean([abs(v) for v in entries])
            if avg_lat > 5.0:
                issues.append(('TRAJECTOIRE', f"Vitesse latérale à l\'entrée APPROACH: {avg_lat:.1f} m/s → momentum EXIT trop élevé"))

    # Angle au relaunch
    rl = df[df['relaunch_timer']>0] if 'relaunch_timer' in df.columns else df[df['recovery_state'] == 4] if 'recovery_state' in df.columns else pd.DataFrame()
    if len(rl) > 5:
        avg_angle_deg = abs(np.degrees(rl['angle'].mean())) if 'angle' in rl else 0
        if avg_angle_deg > 45:
            issues.append(('RECOVERY', f"Relaunch avec angle {avg_angle_deg:.0f}° → voiture roule de travers"))

    if not issues:
        issues.append(('OK', 'Aucun problème critique détecté'))
    return issues


def panel_lateral_velocity(df):
    """Vitesse latérale trackPos — diagnostic momentum EXIT→APPROACH."""
    fig, axes = plt.subplots(2, 1, figsize=(14, 5), sharex=True)
    fig.subplots_adjust(hspace=0.05)

    df2 = df.copy()
    df2['lat_vel'] = df2['track_pos'].diff() * 50 * 6  # m/s

    ax = axes[0]
    state_segments(df2, ax, alpha=0.08); off_track_spans(df2, ax)
    ax.plot(df2['dist'], df2['lat_vel'], color=RED, lw=0.8, label='Lat vel (m/s)')
    ax.axhline(5,  color=YEL, lw=0.5, ls='--', label='+5 m/s danger')
    ax.axhline(-5, color=YEL, lw=0.5, ls='--')
    ax.axhline(0,  color=GRY, lw=0.4)
    # Marquer les transitions EXIT→APPROACH
    for i in range(1, len(df2)):
        if 'state' in df2.columns:
            if df2.iloc[i]['state']=='APPROACH' and df2.iloc[i-1]['state']!='APPROACH':
                v = df2.iloc[i]['lat_vel']
                ax.axvline(df2.iloc[i]['dist'], color=YEL, lw=1, ls=':', alpha=0.7)
                ax.text(df2.iloc[i]['dist'], v, f'{v:.1f}', fontsize=7, color=YEL)
    ax.set_ylabel('Vitesse latérale (m/s)'); ax.legend(fontsize=7); ax.grid(True, axis='y')
    ax.set_title('LATERAL VELOCITY — momentum EXIT→APPROACH (yellow = transition)')

    ax2 = axes[1]
    state_segments(df2, ax2, alpha=0.06)
    ax2.plot(df2['dist'], df2['track_pos'] - df2['target_pos'], color=BLU, lw=0.8, label='pos error')
    ax2.axhline(0.40,  color=YEL, lw=0.5, ls='--', label='Rate limiter bypass (0.40)')
    ax2.axhline(-0.40, color=YEL, lw=0.5, ls='--')
    ax2.axhline(0, color=GRY, lw=0.4)
    ax2.set_ylabel('pos error (track - target)'); ax2.set_xlabel('Distance (m)')
    ax2.legend(fontsize=7); ax2.grid(True, axis='y')
    return fig

# ═══════════════════════════════════════════════════════════════════════════════
#  RAPPORT HTML
# ═══════════════════════════════════════════════════════════════════════════════

ISSUE_COLORS = {'CRITIQUE':'#ff4444','PHYSIQUE':'#ff8c00','TRAJECTOIRE':'#ffcc00',
                'FREIN':'#ff6b35','RECOVERY':'#c084fc','REVERSE':'#ff85a1','OK':'#7bed9f'}

def _legend_states(ax):
    patches = [mpatches.Patch(color=c, alpha=0.5, label=s) for s,c in STATE_COLORS.items()] if hasattr(STATE_COLORS, 'items') else []
    ax.legend(handles=patches + ax.get_legend_handles_labels()[0],
              fontsize=6.5, loc='upper right', ncol=5, framealpha=0.4)

def generate_report(csv_path):
    df = load(csv_path)
    diag = compute_diagnostics(df)
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir = os.path.join(script_dir, 'report')
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, 'rapport.html')

    panels = [
        ('Speed trace',          panel_speed_trace(df)),
        ('Commandes pédales',    panel_pedals(df)),
        ('Cercle de Kamm',       panel_kamm_circle(df)),
        ('Trajectoire latérale', panel_lateral(df)),
        ('G-forces',             panel_longitudinal_g(df)),
        ('Points de freinage',   panel_braking_points(df)),
        ('RPM & Vitesses/gear',  panel_rpm_gear(df)),
        ('Récupération',         panel_recovery_timeline(df)),
        ('Wheelspin & TC',       panel_wheelspin(df)),
        ('Capteurs & Courbure',  panel_curvature_sensor(df)),
        ('Carte circuit',        panel_circuit_map(df)),
        ('Vitesse laterale',     panel_lateral_velocity(df)),
    ]
    imgs = [(title, fig_to_b64(fig)) for title, fig in panels]

    # ── HTML ──────────────────────────────────────────────────────────────────
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    stat_s = diag['session']
    stat_p = diag['performance']
    issues = diag['issues']

    def stat_rows(d):
        return ''.join(f'<tr><td class="k">{k}</td><td class="v">{v}</td></tr>' for k,v in d.items())

    def issue_rows(lst):
        rows = []
        for cat, msg in lst:
            clr = ISSUE_COLORS.get(cat, '#888')
            rows.append(f'<tr><td style="color:{clr};font-weight:bold">{cat}</td><td>{msg}</td></tr>')
        return ''.join(rows)

    def img_section(title, b64):
        return f'''
        <div class="panel">
          <h2>{title}</h2>
          <img src="data:image/png;base64,{b64}" style="width:100%;border-radius:6px">
        </div>'''

    html = f'''<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="UTF-8">
<title>TORCS Rapport Télémétrie</title>
<style>
  * {{ box-sizing:border-box; margin:0; padding:0 }}
  body {{ background:#0a0a0a; color:#ccc; font-family:monospace; font-size:13px; padding:20px }}
  h1 {{ color:#4a9eff; font-size:22px; margin-bottom:4px }}
  h2 {{ color:#ffcc00; font-size:13px; text-transform:uppercase; letter-spacing:2px; margin-bottom:8px; padding-bottom:4px; border-bottom:1px solid #333 }}
  .meta {{ color:#666; font-size:11px; margin-bottom:20px }}
  .grid2 {{ display:grid; grid-template-columns:1fr 1fr; gap:16px; margin-bottom:20px }}
  .grid3 {{ display:grid; grid-template-columns:1fr 1fr 1fr; gap:16px; margin-bottom:20px }}
  .card {{ background:#141414; border:1px solid #222; border-radius:8px; padding:16px }}
  table {{ width:100%; border-collapse:collapse }}
  td {{ padding:4px 6px; border-bottom:1px solid #1e1e1e; vertical-align:top }}
  td.k {{ color:#888; width:55% }}
  td.v {{ color:#eee; font-weight:bold; text-align:right }}
  .issue-table td {{ padding:5px 8px; border-bottom:1px solid #1e1e1e }}
  .panel {{ margin-bottom:24px }}
  .badge {{ display:inline-block; padding:2px 8px; border-radius:4px; font-size:11px; margin:2px }}
</style>
</head>
<body>
<h1>🏎  TORCS Autopilot — Rapport Télémétrie</h1>
<p class="meta">Fichier : {os.path.basename(csv_path)} | Généré : {ts}</p>

<div class="grid3">
  <div class="card">
    <h2>Session</h2>
    <table>{stat_rows(stat_s)}</table>
  </div>
  <div class="card">
    <h2>Performance</h2>
    <table>{stat_rows(stat_p)}</table>
  </div>
  <div class="card">
    <h2>⚠ Diagnostics automatiques</h2>
    <table class="issue-table">{issue_rows(issues)}</table>
  </div>
</div>

{''.join(img_section(t, b) for t, b in imgs)}

</body></html>'''

    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(html)
    print(f"[REPORT] | .HTML saved")
    return out_path


# ═══════════════════════════════════════════════════════════════════════════════
#  RAPPORT MARKDOWN — léger, pensé pour être collé dans une session IA
# ═══════════════════════════════════════════════════════════════════════════════

def generate_md_report(csv_path):
    df = load(csv_path)
    diag = compute_diagnostics(df)
    ts   = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir = os.path.join(script_dir, 'report')
    os.makedirs(out_dir, exist_ok=True)
    out  = os.path.join(out_dir, 'rapport.md')

    lines = []
    w = lines.append   # alias

    # ── En-tête ───────────────────────────────────────────────────────────────
    w(f"# TORCS Telemetry Report")
    w(f"")
    w(f"**File:** `{os.path.basename(csv_path)}`  ")
    w(f"**Generated:** {ts}  ")
    w(f"**Frames:** {len(df)}  ")
    w(f"")

    # ── Session ───────────────────────────────────────────────────────────────
    w("## Session")
    w("")
    w("| Metric | Value |")
    w("|---|---|")
    for k, v in diag['session'].items():
        w(f"| {k} | {v} |")
    w("")

    # ── Performance ───────────────────────────────────────────────────────────
    w("## Performance")
    w("")
    w("| Metric | Value |")
    w("|---|---|")
    for k, v in diag['performance'].items():
        w(f"| {k} | {v} |")
    w("")

    # ── Diagnostics ───────────────────────────────────────────────────────────
    w("## Diagnostics")
    w("")
    for cat, msg in diag['issues']:
        icon = "✅" if cat == "OK" else "⚠️" if cat in ("TRAJECTOIRE","FREIN","PHYSIQUE") else "🔴"
        w(f"- **[{cat}]** {icon} {msg}")
    w("")

    # ── Vitesse par état ──────────────────────────────────────────────────────
    w("## Speed by state")
    w("")
    w("| State | Mean (km/h) | Min | Max | Frames | % total |")
    w("|---|---|---|---|---|---|")
    total = max(len(df), 1)
    for st in ['STRAIGHT', 'APPROACH', 'TURN_IN', 'EXIT']:
        sub = df[df['state'] == st]['speed_x'] if 'state' in df.columns else pd.Series()
        if len(sub):
            w(f"| {st} | {sub.mean():.1f} | {sub.min():.1f} | {sub.max():.1f} | {len(sub)} | {100*len(sub)/total:.1f}% |")
    w("")

    # ── Throttle / brake / coasting ───────────────────────────────────────────
    w("## Pedal usage")
    w("")
    thr_hi  = (df['throttle'] > 0.5).mean() * 100
    thr_any = (df['throttle'] > 0.05).mean() * 100
    brk_hi  = (df['brake'] > 0.3).mean() * 100
    brk_any = (df['brake'] > 0.05).mean() * 100
    coast   = ((df['throttle'] < 0.05) & (df['brake'] < 0.05)).mean() * 100
    w(f"| Action | % frames |")
    w(f"|---|---|")
    w(f"| Full throttle (> 0.5) | {thr_hi:.1f}% |")
    w(f"| Any throttle (> 0.05) | {thr_any:.1f}% |")
    w(f"| Hard braking (> 0.3) | {brk_hi:.1f}% |")
    w(f"| Any braking (> 0.05) | {brk_any:.1f}% |")
    w(f"| **Coasting (neither)** | **{coast:.1f}%** |")
    w("")

    # ── Apex speeds par virage ────────────────────────────────────────────────
    w("## Corner apex speeds")
    w("")
    w("| dist_from_start (m) | v_entry (km/h) | v_apex (km/h) | Δ (km/h) | frames in TI |")
    w("|---|---|---|---|---|")
    in_ti = False
    ti_speeds, ti_start_dist = [], 0.0
    for i in range(len(df)):
        row = df.iloc[i]
        st  = row.get('state', '')
        if st == 'TURN_IN' and not in_ti:
            in_ti = True
            ti_speeds = []
            ti_start_dist = row.get('dist_from_start', row.get('dist', 0))
        if in_ti:
            v = row.get('speed_x', 0)
            ti_speeds.append(float(v) if pd.notna(v) else 0.0)
        if st != 'TURN_IN' and in_ti:
            in_ti = False
            if ti_speeds:
                v_entry = ti_speeds[0]
                v_apex  = min(ti_speeds)
                w(f"| {ti_start_dist:.0f} | {v_entry:.1f} | {v_apex:.1f} | {v_entry-v_apex:.1f} | {len(ti_speeds)} |")
    w("")

    # ── Braking efficiency ────────────────────────────────────────────────────
    w("## Braking efficiency (APPROACH + TURN_IN)")
    w("")
    if 'state' in df.columns:
        bt = df[df['state'].isin(['APPROACH', 'TURN_IN'])].copy()
        bt['delta'] = bt['speed_x'] - bt['target_speed']
        over  = bt['delta'].clip(lower=0)
        under = bt['delta'].clip(upper=0)
        w(f"| Metric | Value |")
        w(f"|---|---|")
        w(f"| Mean over-speed (speed > target) | {over.mean():.1f} km/h |")
        w(f"| Mean under-speed (speed < target) | {under.mean():.1f} km/h |")
        w(f"| Frames over target | {(bt['delta']>0).sum()} ({100*(bt['delta']>0).mean():.0f}%) |")
        w(f"| Frames >10 km/h over target | {(bt['delta']>10).sum()} |")
        w(f"| Frames >30 km/h under target | {(bt['delta']<-30).sum()} |")
    w("")

    # ── Wheelspin & TC ────────────────────────────────────────────────────────
    w("## Wheelspin & traction control")
    w("")
    if 'slip_rear' in df.columns:
        tc  = (df['slip_rear'].abs() > 2).sum()
        tc5 = (df['slip_rear'].abs() > 5).sum()
        w(f"| Metric | Value |")
        w(f"|---|---|")
        w(f"| TC active frames (slip > 2 rad/s) | {tc} ({100*tc/total:.1f}%) |")
        w(f"| Hard wheelspin frames (slip > 5) | {tc5} ({100*tc5/total:.1f}%) |")
        w(f"| Max slip_rear | {df['slip_rear'].abs().max():.1f} rad/s |")
        if tc > 0:
            tc_rows = df[df['slip_rear'].abs() > 2]
            w(f"| Mean speed during TC | {tc_rows['speed_x'].mean():.1f} km/h |")
            w(f"| Mean throttle during TC | {tc_rows['throttle'].mean():.3f} |")
    w("")

    # ── Gears ─────────────────────────────────────────────────────────────────
    w("## Gear usage")
    w("")
    w("| Gear | % frames | Max speed (km/h) | Max RPM |")
    w("|---|---|---|---|")
    for g in [-1, 1, 2, 3, 4, 5, 6, 7]:
        sub = df[df['gear'] == g]
        if len(sub):
            w(f"| G{g} | {100*len(sub)/total:.1f}% | {sub['speed_x'].max():.0f} | {sub['rpm'].max():.0f} |")
    w("")

    # ── Recovery events ───────────────────────────────────────────────────────
    w("## Recovery events")
    w("")
    ot_total = df['off_track'].sum() if 'off_track' in df.columns else (df['track_pos'].abs() > 1.0).sum()
    
    rev_total = (df['reverse_timer'] > 0).sum() if 'reverse_timer' in df.columns else (df['recovery_state'] == 2).sum() if 'recovery_state' in df.columns else 0
    rel_total = (df['relaunch_timer'] > 0).sum() if 'relaunch_timer' in df.columns else (df['recovery_state'] == 4).sum() if 'recovery_state' in df.columns else 0
    
    w(f"| Event | Frames | % total |")
    w(f"|---|---|---|")
    w(f"| Off-track (\\|pos\\| > 1.0) | {ot_total} | {100*ot_total/total:.1f}% |")
    w(f"| Reverse active | {rev_total} | {100*rev_total/total:.1f}% |")
    w(f"| Relaunch active | {rel_total} | {100*rel_total/total:.1f}% |")
    w("")

    # ── Log des incidents : 15 frames avant + 10 après chaque sortie de piste ─
    w("## Incident log")
    w("")
    w("> 15 frames before and 10 frames after each off-track event.")
    w("> Columns: step | speed_x | target_speed | track_pos | state | alpha_deg | max_dist | steer | throttle | brake | angle")
    w("")

    df['_ot'] = df['track_pos'].abs() > 1.0
    # Trouver les débuts d'incidents (transition False→True)
    incident_starts = []
    prev_ot = False
    for i in range(len(df)):
        cur_ot = bool(df.iloc[i]['_ot'])
        if cur_ot and not prev_ot:
            incident_starts.append(i)
        prev_ot = cur_ot
        if len(incident_starts) >= 5:   # max 5 incidents pour rester léger
            break

    COLS = ['step', 'speed_x', 'target_speed', 'track_pos', 'state',
            'alpha_deg', 'max_dist', 'steer', 'throttle', 'brake', 'angle']
    present = [c for c in COLS if c in df.columns]

    for n, idx in enumerate(incident_starts, 1):
        start = max(0, idx - 15)
        end   = min(len(df), idx + 11)
        snippet = df.iloc[start:end][present].copy()
        for c in present:
            if c not in ('state',):
                snippet[c] = pd.to_numeric(snippet[c], errors='coerce').round(3)

        entry_row = df.iloc[idx]
        entry_dist = entry_row.get('dist_from_start', entry_row.get('dist', '?'))
        entry_speed = entry_row.get('speed_x', '?')
        entry_state = entry_row.get('state', '?')
        w(f"### Incident {n} — dist_from_start ≈ {float(entry_dist):.0f} m  |  speed = {float(entry_speed):.1f} km/h  |  state = {entry_state}")
        w("")
        # Header
        w("| " + " | ".join(present) + " |")
        w("|" + "|".join(["---"] * len(present)) + "|")
        for _, row in snippet.iterrows():
            vals = []
            for c in present:
                v = row[c]
                vals.append(str(v) if pd.notna(v) else "—")
            w("| " + " | ".join(vals) + " |")
        w("")

    if not incident_starts:
        w("_No off-track incidents detected. Clean lap!_ ✅")
        w("")

    # ── Committed apex speed log ───────────────────────────────────────────────
    if 'committed_apex_kmh' in df.columns:
        w("## Committed apex speed per corner")
        w("")
        w("| dist_from_start (m) | state at lock | committed (km/h) | actual apex (km/h) |")
        w("|---|---|---|---|")
        # Trouver le minimum de committed_apex_kmh dans chaque bloc APPROACH
        in_app = False
        app_min, app_dist, app_state = 9999, 0, ''
        prev_state = ''
        for i in range(len(df)):
            row   = df.iloc[i]
            st    = row.get('state', '')
            ca    = float(row.get('committed_apex_kmh', 9999) or 9999)
            dist  = float(row.get('dist_from_start', row.get('dist', 0)) or 0)
            if st == 'APPROACH' and not in_app:
                in_app = True; app_min = ca; app_dist = dist
            if in_app and st == 'APPROACH':
                if ca < app_min: app_min = ca; app_dist = dist
            if st != 'APPROACH' and in_app:
                in_app = False
                # Trouver le vrai apex dans le TURN_IN suivant
                ti_rows = df.iloc[i:min(i+100, len(df))]
                ti_spds = ti_rows[ti_rows['state'] == 'TURN_IN']['speed_x']
                v_apex_actual = ti_spds.min() if len(ti_spds) else float('nan')
                w(f"| {app_dist:.0f} | APPROACH→TURN_IN | {app_min:.1f} | {v_apex_actual:.1f} |")
        w("")

    # ── Footer ────────────────────────────────────────────────────────────────
    w("---")
    w(f"_Report generated by analyse.py — {ts}_")

    md_text = "\n".join(lines)
    with open(out, 'w', encoding='utf-8') as f:
        f.write(md_text)
    print(f"[REPORT] | .MD saved")
    return out


# ═══════════════════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    _parent = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    # Cherche en priorité le csv dans /report si aucun argument n'est fourni.
    default_csv_report = os.path.join(_parent, "report", "telemetry.csv")
    default_csv_root = os.path.join(_parent, "telemetry.csv")
    
    if len(sys.argv) > 1:
        csv = sys.argv[1]
    elif os.path.exists(default_csv_report):
        csv = default_csv_report
    else:
        csv = default_csv_root

    if not os.path.exists(csv):
        print(f"Fichier introuvable : {csv}")
        sys.exit(1)
        
    generate_report(csv)
    generate_md_report(csv)