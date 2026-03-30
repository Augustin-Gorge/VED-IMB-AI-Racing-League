#!/usr/bin/env python3
"""
Comprehensive trajectory and control analysis tool.
Identifies geometric trajectory issues and acceleration/braking optimization opportunities.
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import math

plt.rcParams.update({
    'figure.facecolor': '#0f0f0f', 'axes.facecolor': '#1a1a1a',
    'axes.edgecolor': '#333', 'axes.labelcolor': '#ccc',
    'text.color': '#ccc', 'xtick.color': '#888', 'ytick.color': '#888',
    'grid.color': '#2a2a2a', 'grid.linewidth': 0.5,
    'font.family': 'monospace', 'font.size': 9,
})

STATE_COLORS = {
    'STRAIGHT': '#4a9eff', 'APPROACH': '#ffcc00',
    'TURN_IN':  '#ff6b35', 'EXIT':     '#7bed9f',
}

def load_telemetry(path):
    """Load telemetry CSV."""
    df = pd.read_csv(path, sep=';', low_memory=False)
    df.columns = df.columns.str.strip()
    for col in df.columns:
        if col not in ('state', 'surface', 'recovery_state'):
            df[col] = pd.to_numeric(df[col], errors='coerce')
    return df

def analyze_coasting(df):
    """Analyze wasted coasting phases."""
    print("\n" + "="*80)
    print("COASTING ANALYSIS")
    print("="*80)
    
    # Identify coasting frames
    df['coasting'] = (df['throttle'] <= 0.05) & (df['brake'] <= 0.05)
    df['coasting_group'] = (df['coasting'] != df['coasting'].shift()).cumsum()
    
    coasting_pct = df['coasting'].mean() * 100
    print(f"\nTotal coasting: {coasting_pct:.1f}% ({df['coasting'].sum()} frames)")
    
    # Analyze coasting by state
    print("\nCoasting by state:")
    for state in ['STRAIGHT', 'APPROACH', 'TURN_IN', 'EXIT']:
        state_df = df[df['state'] == state]
        coast_pct = state_df['coasting'].mean() * 100 if len(state_df) > 0 else 0
        avg_speed = state_df['speed_x'].mean()
        print(f"  {state:10s}: {coast_pct:5.1f}% ({avg_speed:6.1f} km/h)")
    
    # Find longest coasting phases
    coasting_groups = df[df['coasting']].groupby('coasting_group')
    if len(coasting_groups) > 0:
        durations = coasting_groups.size().sort_values(ascending=False)
        print(f"\nLongest coasting phases: {durations.head(5).values} frames")
        
        # Check if coasting in APPROACH state (waste opportunity)
        approach_coast = df[(df['state'] == 'APPROACH') & (df['coasting'])]
        print(f"  In APPROACH state: {len(approach_coast)} frames ({len(approach_coast)/len(df[df['state']=='APPROACH'])*100:.1f}%)")
        if len(approach_coast) > 0:
            avg_speed_loss = (approach_coast['target_speed'] - approach_coast['speed_x']).mean()
            print(f"  Avg speed deficit: {avg_speed_loss:.1f} km/h")
    
    return coasting_pct

def analyze_acceleration_reserves(df):
    """Check if throttle is being used optimally."""
    print("\n" + "="*80)
    print("ACCELERATION ANALYSIS")
    print("="*80)
    
    # Analyze throttle profiles
    print("\nThrottle usage:")
    print(f"  Full throttle (>0.9): {(df['throttle']>0.9).mean()*100:.1f}%")
    print(f"  High throttle (>0.7): {(df['throttle']>0.7).mean()*100:.1f}%")
    print(f"  Medium throttle (0.3-0.7): {((df['throttle']>0.3) & (df['throttle']<=0.7)).mean()*100:.1f}%")
    
    # Check if target speed is being achieved
    speed_deficit = df['target_speed'] - df['speed_x']
    print(f"\nSpeed tracking:")
    print(f"  Mean deficit (under target): {speed_deficit[speed_deficit>0].mean():.1f} km/h")
    print(f"  Max deficit: {speed_deficit.max():.1f} km/h")
    print(f"  Frames >5 km/h under target: {(speed_deficit>5).sum()} ({(speed_deficit>5).mean()*100:.1f}%)")
    print(f"  Frames >10 km/h under target: {(speed_deficit>10).sum()} ({(speed_deficit>10).mean()*100:.1f}%)")
    
    # Analyze in STRAIGHT state (should be high throttle)
    straight = df[df['state'] == 'STRAIGHT']
    if len(straight) > 0:
        full_throttle_pct = (straight['throttle'] > 0.85).mean() * 100
        print(f"\nIn STRAIGHT state:")
        print(f"  Full throttle: {full_throttle_pct:.1f}%")
        print(f"  Avg throttle: {straight['throttle'].mean():.2f}")
        print(f"  Avg speed deficit: {(straight['target_speed']-straight['speed_x']).mean():.1f} km/h")
    
    # Traction control analysis
    tc_active = (df['slip_rear'].abs() > 2.0) | (df['wheelspin'] > 0)
    print(f"\nTraction control: {tc_active.mean()*100:.1f}% active")
    during_tc = df[tc_active]
    print(f"  Avg throttle during TC: {during_tc['throttle'].mean():.2f}")
    print(f"  Mean throttle reserve during TC: {(1.0-during_tc['throttle']).mean():.2f}")
    
    return speed_deficit

def analyze_braking_efficiency(df):
    """Analyze braking patterns and opportunities."""
    print("\n" + "="*80)
    print("BRAKING ANALYSIS")
    print("="*80)
    
    # Braking intensity
    print("\nBraking usage:")
    print(f"  Hard braking (>0.7): {(df['brake']>0.7).mean()*100:.1f}%")
    print(f"  Medium braking (0.3-0.7): {((df['brake']>0.3) & (df['brake']<=0.7)).mean()*100:.1f}%")
    print(f"  Light braking (0.05-0.3): {((df['brake']>0.05) & (df['brake']<=0.3)).mean()*100:.1f}%")
    
    # Check brake timing relative to corners
    approach = df[df['state'] == 'APPROACH']
    if len(approach) > 0:
        braking_approach = approach[approach['brake'] > 0.05]
        print(f"\nIn APPROACH state:")
        print(f"  Braking frames: {len(braking_approach)} ({len(braking_approach)/len(approach)*100:.1f}%)")
        print(f"  Avg brake: {approach['brake'].mean():.2f}")
        print(f"  Avg speed reduction needed: {(approach['speed_x'].mean() - approach['target_speed'].mean()):.1f} km/h")
        
        # Early vs late braking
        avg_max_dist_brake = braking_approach['max_dist'].mean()
        print(f"  Avg max_dist when braking: {avg_max_dist_brake:.1f} m")
    
    return df

def analyze_corner_geometry(df):
    """Analyze trajectory through corners."""
    print("\n" + "="*80)
    print("CORNER GEOMETRY ANALYSIS")
    print("="*80)
    
    turn_in = df[df['state'] == 'TURN_IN']
    if len(turn_in) > 0:
        print(f"\nTURN_IN phase statistics:")
        print(f"  Avg lateral position: {turn_in['track_pos'].mean():.4f}")
        print(f"  Max lateral pos: {turn_in['track_pos'].max():.4f}")
        print(f"  Min lateral pos: {turn_in['track_pos'].min():.4f}")
        print(f"  Std dev lateral: {turn_in['track_pos'].std():.4f}")
        
        # Entry conditions
        ti_entries = df[(df['state'] == 'TURN_IN') & 
                       (df['state'].shift() != 'TURN_IN')]
        if len(ti_entries) > 0:
            print(f"\nTURN_IN entry conditions ({len(ti_entries)} transitions):")
            print(f"  Avg entry speed: {ti_entries['speed_x'].mean():.1f} km/h")
            print(f"  Avg target speed: {ti_entries['target_speed'].mean():.1f} km/h")
            print(f"  Avg max_dist at entry: {ti_entries['max_dist'].mean():.1f} m")
            print(f"  Avg steer at entry: {ti_entries['steer'].abs().mean():.3f}")
        
        # Apex conditions
        apex_frames = turn_in.groupby((turn_in['speed_x'] != turn_in['speed_x'].shift()).cumsum())
        for group_id, group in apex_frames:
            if len(group) > 0:
                min_speed_frame = group.loc[group['speed_x'].idxmin()]
                if min_speed_frame['speed_x'] < group['speed_x'].mean() - 5:
                    print(f"  Apex at {min_speed_frame['dist_from_start']:.0f}m: speed {min_speed_frame['speed_x']:.1f} km/h")
    
    # Analyze lateral position smoothness
    print(f"\nLateral position smoothness (track_pos spike analysis):")
    df['pos_accel'] = df['track_pos'].diff().diff()
    spikes = df[df['pos_accel'].abs() > 0.05]
    print(f"  Sharp changes: {len(spikes)} frames")
    print(f"  Max accel: {df['pos_accel'].abs().max():.4f}")
    
    return turn_in

def analyze_state_transitions(df):
    """Analyze state machine transitions."""
    print("\n" + "="*80)
    print("STATE TRANSITION ANALYSIS")
    print("="*80)
    
    # Count transitions
    transitions = df['state'].ne(df['state'].shift()).sum()
    print(f"\nTotal state transitions: {transitions}")
    
    # Transition quality
    state_prev = df['state'].shift()
    state_curr = df['state']
    
    for t in ['STRAIGHT->APPROACH', 'APPROACH->TURN_IN', 'TURN_IN->EXIT', 'EXIT->STRAIGHT']:
        from_state, to_state = t.split('->')
        mask = (state_prev == from_state) & (state_curr == to_state)
        count = mask.sum()
        if count > 0:
            trans_data = df[mask]
            avg_speed_delta = (trans_data['speed_x'] - trans_data['target_speed']).mean()
            print(f"\n{t}: {count} transitions")
            print(f"  Avg speed delta at transition: {avg_speed_delta:+.1f} km/h")
            print(f"  Avg max_dist: {trans_data['max_dist'].mean():.1f} m")

def generate_trajectory_profile(df):
    """Generate ideal vs actual trajectory."""
    print("\n" + "="*80)
    print("TRAJECTORY PROFILE")
    print("="*80)
    
    # Group by distance milestones
    bins = np.arange(0, df['dist_from_start'].max(), 100)
    df['bin'] = pd.cut(df['dist_from_start'], bins=bins)
    
    print(f"\nSpeed profile by 100m segments:")
    profile = df.groupby('bin').agg({
        'speed_x': 'mean',
        'target_speed': 'mean',
        'track_pos': lambda x: (x.abs()).mean(),
        'state': lambda x: x.mode()[0] if len(x.mode()) > 0 else 'UNKNOWN',
        'throttle': 'mean',
        'brake': 'mean',
        'steer': lambda x: (x.abs()).mean(),
    })
    
    print(f"\n{'Start':>8} {'Act Spd':>8} {'Tgt Spd':>8} {'Lat Pos':>8} {'State':>10} {'Thr/Brk':>8} {'Steer':>8}")
    for idx, row in profile.iterrows():
        if pd.isna(row['speed_x']):
            continue
        start = int(idx.left) if idx.left else 0
        print(f"{start:8.0f} {row['speed_x']:8.1f} {row['target_speed']:8.1f} "
              f"{row['track_pos']:8.4f} {row['state']:>10s} "
              f"{row['throttle']:.2f}/{row['brake']:.2f} {row['steer']:8.3f}")

def generate_improvement_recommendations(df):
    """Generate specific improvement recommendations."""
    print("\n" + "="*80)
    print("IMPROVEMENT RECOMMENDATIONS")
    print("="*80)
    
    recommendations = []
    
    # 1. Coasting optimization
    coasting_pct = ((df['throttle'] <= 0.05) & (df['brake'] <= 0.05)).mean() * 100
    if coasting_pct > 15:
        time_loss = coasting_pct * 0.5  # rough estimate: 0.5% time per 1% coasting
        recommendations.append({
            'priority': 1,
            'title': 'Reduce coasting phases',
            'current': f'{coasting_pct:.1f}%',
            'target': '12-15%',
            'impact': f'Est. {time_loss:.1f}s faster',
            'action': 'Increase cruise floor in STRAIGHT/APPROACH. Add throttle blip in coast zones.'
        })
    
    # 2. Braking efficiency
    hard_brake_pct = (df['brake'] > 0.7).mean() * 100
    approach_df = df[df['state'] == 'APPROACH']
    if len(approach_df) > 0:
        approach_speed_deficit = (approach_df['target_speed'] - approach_df['speed_x']).mean()
        if approach_speed_deficit > 8:
            recommendations.append({
                'priority': 2,
                'title': 'Brake earlier in APPROACH',
                'current': f'{approach_speed_deficit:.1f} km/h avg deficit',
                'target': '<5 km/h deficit',
                'impact': 'Smoother transitions to corners',
                'action': 'Increase brake_factor or trigger braking at larger max_dist.'
            })
    
    # 3. Acceleration optimization
    straight_df = df[df['state'] == 'STRAIGHT']
    if len(straight_df) > 0:
        full_throttle_pct = (straight_df['throttle'] > 0.9).mean() * 100
        if full_throttle_pct < 60:
            recommendations.append({
                'priority': 2,
                'title': 'Increase throttle in STRAIGHT',
                'current': f'{full_throttle_pct:.1f}% full throttle',
                'target': '>75% full throttle',
                'impact': 'Higher straight speed',
                'action': 'Reduce cruise_floor minimum in compute_pedals() or increase max_thr.'
            })
    
    # 4. Trajectory geometry
    turn_in_df = df[df['state'] == 'TURN_IN']
    if len(turn_in_df) > 0:
        lateral_std = turn_in_df['track_pos'].std()
        if lateral_std > 0.15:
            recommendations.append({
                'priority': 3,
                'title': 'Smoother lateral control in TURN_IN',
                'current': f'Lat position σ={lateral_std:.4f}',
                'target': '<0.10 (more geometric)',
                'impact': 'More consistent cornering',
                'action': 'Reduce steering rate changes. Increase SMOOTH_CORNER coefficient.'
            })
    
    # 5. Early corner identification
    ti_entries = df[(df['state'] == 'TURN_IN') & (df['state'].shift() != 'TURN_IN')]
    if len(ti_entries) > 0:
        avg_max_dist = ti_entries['max_dist'].mean()
        if avg_max_dist > 40:
            recommendations.append({
                'priority': 3,
                'title': 'Earlier TURN_IN trigger',
                'current': f'Avg TI at {avg_max_dist:.0f}m distance',
                'target': '<35m lookahead',
                'impact': 'Better apex positioning',
                'action': 'Reduce dist_turn_in threshold in update_state().'
            })
    
    # 6. TC efficiency
    during_tc = df[df['wheelspinVel_sum_filt'] > 0] if 'wheelspinVel_sum_filt' in df.columns else df[df['slip_rear'].abs() > 2]
    if len(during_tc) > 0:
        tc_throttle = during_tc['throttle'].mean()
        if tc_throttle < 0.35:
            recommendations.append({
                'priority': 2,
                'title': 'More aggressive acceleration during TC',
                'current': f'{tc_throttle:.2f} avg throttle during TC',
                'target': '>0.45',
                'impact': 'Better exit acceleration',
                'action': 'Tune traction_control() slip threshold. Allow higher throttle.'
            })
    
    print("\n")
    for i, rec in enumerate(recommendations, 1):
        print(f"{i}. [{rec['priority']}] {rec['title']}")
        print(f"   Current:  {rec['current']}")
        print(f"   Target:   {rec['target']}")
        print(f"   Impact:   {rec['impact']}")
        print(f"   Action:   {rec['action']}")
        print()
    
    return recommendations

def plot_analysis(df):
    """Generate analysis plots."""
    fig, axes = plt.subplots(3, 2, figsize=(16, 10))
    
    # 1. Speed profile with target
    ax = axes[0, 0]
    ax.plot(df['dist_from_start'], df['target_speed'], 'g--', label='Target', lw=1)
    ax.plot(df['dist_from_start'], df['speed_x'], label='Actual', lw=1.5)
    ax.set_ylabel('Speed (km/h)')
    ax.set_title('Speed Tracking')
    ax.legend()
    ax.grid(True)
    
    # 2. Pedal usage
    ax = axes[0, 1]
    ax.plot(df['dist_from_start'], df['throttle'], 'g', label='Throttle', lw=1)
    ax.plot(df['dist_from_start'], -df['brake'], 'r', label='Brake', lw=1)
    ax.axhline(0, color='k', lw=0.5)
    ax.set_ylabel('Pedal')
    ax.set_title('Throttle & Brake')
    ax.legend()
    ax.grid(True)
    
    # 3. Lateral position
    ax = axes[1, 0]
    for state in ['STRAIGHT', 'APPROACH', 'TURN_IN', 'EXIT']:
        state_df = df[df['state'] == state]
        if len(state_df) > 0:
            ax.plot(state_df['dist_from_start'], state_df['track_pos'], 
                   '.', color=STATE_COLORS.get(state, '#999'), label=state, markersize=2)
    ax.axhline(1.0, color='r', lw=0.5, alpha=0.5)
    ax.axhline(-1.0, color='r', lw=0.5, alpha=0.5)
    ax.axhline(0.0, color='gray', lw=0.5, alpha=0.3)
    ax.set_ylabel('Track Position')
    ax.set_title('Lateral Trajectory')
    ax.legend(fontsize=8)
    ax.grid(True)
    
    # 4. Coasting phases
    ax = axes[1, 1]
    coasting = (df['throttle'] <= 0.05) & (df['brake'] <= 0.05)
    ax.fill_between(df['dist_from_start'], 0, coasting.astype(int), 
                   color='orange', alpha=0.5, label='Coasting')
    ax.set_ylabel('Coasting')
    ax.set_title('Wasted Coasting Phases')
    ax.set_ylim(-0.1, 1.1)
    ax.grid(True)
    
    # 5. State distribution
    ax = axes[2, 0]
    for state in ['STRAIGHT', 'APPROACH', 'TURN_IN', 'EXIT']:
        ax.bar(state, len(df[df['state'] == state]), color=STATE_COLORS.get(state, '#999'))
    ax.set_ylabel('Frames')
    ax.set_title('State Distribution')
    ax.grid(True, axis='y')
    
    # 6. Speed deficit
    ax = axes[2, 1]
    speed_deficit = df['target_speed'] - df['speed_x']
    ax.plot(df['dist_from_start'], speed_deficit, 'orange', alpha=0.7, lw=1)
    ax.axhline(0, color='gray', lw=0.5)
    ax.fill_between(df['dist_from_start'], 0, speed_deficit, 
                   where=(speed_deficit > 0), color='red', alpha=0.2, label='Under')
    ax.fill_between(df['dist_from_start'], 0, speed_deficit, 
                   where=(speed_deficit <= 0), color='green', alpha=0.2, label='Over')
    ax.set_ylabel('Speed Delta (km/h)')
    ax.set_xlabel('Distance (m)')
    ax.set_title('Target Speed Tracking')
    ax.legend()
    ax.grid(True)
    
    plt.tight_layout()
    plt.savefig('report/trajectory_analysis.png', dpi=100, bbox_inches='tight', facecolor='#0f0f0f')
    print("\n[ANALYSIS] Trajectory plot saved to report/trajectory_analysis.png")
    plt.close()

if __name__ == "__main__":
    print("\n" + "="*80)
    print("TORCS AUTOPILOT - TRAJECTORY & CONTROL ANALYSIS")
    print("="*80)
    
    df = load_telemetry('report/telemetry.csv')
    
    coast_pct = analyze_coasting(df)
    speed_def = analyze_acceleration_reserves(df)
    analyze_braking_efficiency(df)
    analyze_corner_geometry(df)
    analyze_state_transitions(df)
    generate_trajectory_profile(df)
    recs = generate_improvement_recommendations(df)
    plot_analysis(df)
    
    print("\n" + "="*80)
    print("ANALYSIS COMPLETE")
    print("="*80)
    print(f"\n✓ Generated psychological profile")
    print(f"✓ Identified {len(recs)} optimization opportunities")
    print(f"✓ Coasting overhead: {coast_pct:.1f}%")
    print("\nNext steps:")
    print("  1. Review specific recommendations above")
    print("  2. Adjust control parameters in autopilot.py")
    print("  3. Re-run race and compare telemetry")
    print("  4. Iterate on highest-priority items")
