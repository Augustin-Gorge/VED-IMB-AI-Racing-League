#!/usr/bin/env python3
"""
Detailed trajectory optimization validation tool.
Compares specific metrics and generates improvement visualization.
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
    df = pd.read_csv(path, sep=';', low_memory=False)
    df.columns = df.columns.str.strip()
    for col in df.columns:
        if col not in ('state', 'surface', 'recovery_state'):
            df[col] = pd.to_numeric(df[col], errors='coerce')
    return df

def compute_metrics(df):
    """Compute key performance metrics."""
    metrics = {}
    
    # Coasting
    coasting = (df['throttle'] <= 0.05) & (df['brake'] <= 0.05)
    metrics['coasting_pct'] = coasting.mean() * 100
    metrics['coasting_approach'] = (df[df['state']=='APPROACH']['throttle'] <= 0.05).mean() * 100
    metrics['coasting_turnin'] = (df[df['state']=='TURN_IN']['throttle'] <= 0.05).mean() * 100
    
    # Speed tracking
    speed_deficit = df['target_speed'] - df['speed_x']
    metrics['avg_speed_deficit'] = speed_deficit[speed_deficit>0].mean()
    metrics['frames_under_5'] = (speed_deficit > 5).mean() * 100
    metrics['frames_under_10'] = (speed_deficit > 10).mean() * 100
    
    # Throttle
    metrics['full_throttle_pct'] = (df['throttle'] > 0.9).mean() * 100
    metrics['high_throttle_pct'] = (df['throttle'] > 0.7).mean() * 100
    metrics['avg_throttle'] = df['throttle'].mean()
    
    # Braking
    straight = df[df['state']=='STRAIGHT']
    metrics['avg_throttle_straight'] = straight['throttle'].mean()
    metrics['full_throttle_straight'] = (straight['throttle']>0.9).mean() * 100
    
    # Lateral
    turnin = df[df['state']=='TURN_IN']
    metrics['lateral_std'] = turnin['track_pos'].std() if len(turnin)>0 else 0
    metrics['lateral_mean'] = turnin['track_pos'].abs().mean() if len(turnin)>0 else 0
    
    # State times
    metrics['state_dist'] = {}
    for state in ['STRAIGHT', 'APPROACH', 'TURN_IN', 'EXIT']:
        state_df = df[df['state']==state]
        metrics['state_dist'][state] = len(state_df)
    
    # Corner entry
    ti_entries = df[(df['state']=='TURN_IN') & (df['state'].shift()!='TURN_IN')]
    metrics['avg_ti_entry_speed'] = ti_entries['speed_x'].mean() if len(ti_entries)>0 else 0
    metrics['avg_ti_entry_maxdist'] = ti_entries['max_dist'].mean() if len(ti_entries)>0 else 0
    
    # Lap & efficiency
    metrics['lap_time'] = df['lap_time'].max()
    metrics['distance'] = df['dist_from_start'].max()
    
    # Traction control
    tc_active = (df['slip_rear'].abs() > 2.0) | (df['wheelspin'] > 0)
    metrics['tc_pct'] = tc_active.mean() * 100
    metrics['avg_throttle_during_tc'] = df[tc_active]['throttle'].mean() if tc_active.sum()>0 else 0
    
    return metrics

def print_comparison(baseline_path, optimized_path=None):
    """Compare baseline vs optimized run."""
    print("\n" + "="*80)
    print("TRAJECTORY OPTIMIZATION VALIDATION")
    print("="*80)
    
    df_base = load_telemetry(baseline_path)
    metrics_base = compute_metrics(df_base)
    
    print(f"\n{'METRIC':<35} {'BASELINE':>15} {'TARGET':>15} {'GAIN':>12}")
    print("-" * 80)
    
    # Define targets based on analysis
    targets = {
        'coasting_pct': (metrics_base['coasting_pct'], 15.0),
        'avg_speed_deficit': (metrics_base['avg_speed_deficit'], 30.0),
        'frames_under_10': (metrics_base['frames_under_10'], 25.0),
        'avg_throttle': (metrics_base['avg_throttle'], 0.45),
        'full_throttle_straight': (metrics_base['full_throttle_straight'], 45.0),
        'lateral_std': (metrics_base['lateral_std'], 0.30),
        'lap_time': (metrics_base['lap_time'], 100.0),
        'avg_ti_entry_maxdist': (metrics_base['avg_ti_entry_maxdist'], 35.0),
    }
    
    for metric_name, (baseline_val, target_val) in targets.items():
        baseline_val = metrics_base[metric_name]
        
        # Compute expected gain
        if metric_name in ('coasting_pct', 'avg_speed_deficit', 'frames_under_10', 
                           'frames_under_5', 'lap_time', 'avg_ti_entry_maxdist'):
            # Lower is better
            improvement = ((baseline_val - target_val) / baseline_val * 100) if baseline_val > 0 else 0
            symbol = '↓' if improvement > 0 else '↑'
        else:
            # Higher is better
            improvement = ((target_val - baseline_val) / abs(baseline_val) * 100) if baseline_val != 0 else 0
            symbol = '↑' if improvement > 0 else '↓'
        
        fmt_base = f"{baseline_val:.1f}" if isinstance(baseline_val, float) else f"{baseline_val}"
        fmt_targ = f"{target_val:.1f}" if isinstance(target_val, float) else f"{target_val}"
        fmt_improv = f"{abs(improvement):+.1f}%" if improvement != 0 else "—"
        
        color = '#7bed9f' if improvement > 0 else '#ffcc00' if improvement == 0 else '#ff6b6b'
        print(f"{metric_name:<35} {fmt_base:>15} {fmt_targ:>15} {fmt_improv:>12} {symbol}")
    
    if optimized_path:
        print("\n" + "-"*80)
        print("COMPARING WITH OPTIMIZED RUN")
        print("-"*80)
        df_opt = load_telemetry(optimized_path)
        metrics_opt = compute_metrics(df_opt)
        
        print(f"\n{'METRIC':<35} {'BASELINE':>15} {'OPTIMIZED':>15} {'GAIN':>12}")
        print("-" * 80)
        
        key_metrics = [
            'coasting_pct', 'avg_speed_deficit', 'lap_time', 'lateral_std',
            'avg_throttle', 'full_throttle_straight'
        ]
        
        total_gain = 0
        for metric in key_metrics:
            base_val = metrics_base[metric]
            opt_val = metrics_opt[metric]
            
            # Compute improvement
            if metric in ('coasting_pct', 'avg_speed_deficit', 'lap_time', 'lateral_std'):
                # Lower is better
                delta = base_val - opt_val
                if base_val > 0:
                    pct_gain = (delta / base_val * 100)
                else:
                    pct_gain = 0
                symbol = '✓' if delta > 0 else '✗'
            else:
                # Higher is better
                delta = opt_val - base_val
                if base_val != 0:
                    pct_gain = (delta / base_val * 100)
                else:
                    pct_gain = 0
                symbol = '✓' if delta > 0 else '✗'
            
            total_gain += pct_gain if pct_gain > 0 else 0
            
            fmt_base = f"{base_val:.2f}" if isinstance(base_val, float) else str(base_val)
            fmt_opt = f"{opt_val:.2f}" if isinstance(opt_val, float) else str(opt_val)
            fmt_delta = f"{abs(pct_gain):+.1f}%" if pct_gain != 0 else "—"
            
            print(f"{metric:<35} {fmt_base:>15} {fmt_opt:>15} {fmt_delta:>12} {symbol}")
        
        print(f"\n{'OVERALL IMPROVEMENT':.<60} {total_gain:.1f}%")

def plot_detailed_comparison(baseline_path, output_file='report/optimization_detailed.png'):
    """Generate detailed comparison plots."""
    df = load_telemetry(baseline_path)
    
    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    
    # 1. Coasting by state
    ax = axes[0, 0]
    states = ['STRAIGHT', 'APPROACH', 'TURN_IN', 'EXIT']
    coasting_by_state = []
    for state in states:
        state_df = df[df['state']==state]
        if len(state_df) > 0:
            coast_pct = ((state_df['throttle']<=0.05) & (state_df['brake']<=0.05)).mean() * 100
            coasting_by_state.append(coast_pct)
        else:
            coasting_by_state.append(0)
    
    colors_bar = [STATE_COLORS.get(s, '#999') for s in states]
    bars = ax.bar(states, coasting_by_state, color=colors_bar, alpha=0.7)
    ax.axhline(15, color='green', ls='--', lw=2, label='Target')
    ax.set_ylabel('Coasting %')
    ax.set_title('Coasting by State (Should be <15%)')
    ax.legend()
    ax.grid(True, axis='y')
    for bar, val in zip(bars, coasting_by_state):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1, 
               f'{val:.1f}%', ha='center', va='bottom', fontsize=9)
    
    # 2. Speed deficit distribution
    ax = axes[0, 1]
    speed_deficit = df['target_speed'] - df['speed_x']
    ax.hist(speed_deficit[speed_deficit>0], bins=30, color='#ff6b6b', alpha=0.7, edgecolor='white')
    ax.axvline(5, color='orange', ls='--', lw=2, label='5 km/h')
    ax.axvline(10, color='red', ls='--', lw=2, label='10 km/h')
    ax.set_xlabel('Speed Deficit (km/h)')
    ax.set_ylabel('Frames')
    ax.set_title('Speed Tracking: Deficit Distribution')
    ax.legend()
    ax.grid(True, axis='y')
    
    # 3. Lateral position smoothness
    ax = axes[1, 0]
    for state in states:
        state_df = df[df['state']==state]
        if len(state_df) > 0:
            lat_std = state_df['track_pos'].std()
            ax.bar(state, lat_std, color=STATE_COLORS.get(state, '#999'), alpha=0.7)
    
    ax.axhline(0.30, color='green', ls='--', lw=2, label='Geometric target')
    ax.set_ylabel('Lateral Position σ')
    ax.set_title('Trajectory Smoothness by State')
    ax.legend()
    ax.grid(True, axis='y')
    
    # 4. Throttle levels
    ax = axes[1, 1]
    throttle_ranges = {
        'Full (>0.9)': (df['throttle'] > 0.9).mean() * 100,
        'High (0.7-0.9)': ((df['throttle'] > 0.7) & (df['throttle'] <= 0.9)).mean() * 100,
        'Med (0.3-0.7)': ((df['throttle'] > 0.3) & (df['throttle'] <= 0.7)).mean() * 100,
        'Low (0.05-0.3)': ((df['throttle'] > 0.05) & (df['throttle'] <= 0.3)).mean() * 100,
        'Coast (<0.05)': (df['throttle'] <= 0.05).mean() * 100,
    }
    
    colors_throttle = ['#7bed9f', '#ffcc00', '#ff9500', '#ff6b35', '#555']
    ax.pie(throttle_ranges.values(), labels=throttle_ranges.keys(), colors=colors_throttle,
          autopct='%1.1f%%', startangle=90)
    ax.set_title('Throttle Usage Distribution')
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=100, bbox_inches='tight', facecolor='#0f0f0f')
    print(f"\n[VALIDATION] Detailed comparison plot saved to {output_file}")
    plt.close()

if __name__ == "__main__":
    import sys
    import os
    
    baseline_path = 'report/telemetry.csv'
    optimized_path = None
    
    if len(sys.argv) > 1:
        baseline_path = sys.argv[1]
    if len(sys.argv) > 2:
        optimized_path = sys.argv[2]
    
    if not os.path.exists(baseline_path):
        print(f"Error: {baseline_path} not found")
        sys.exit(1)
    
    print_comparison(baseline_path, optimized_path)
    plot_detailed_comparison(baseline_path)
    print("\n✓ Validation complete")
