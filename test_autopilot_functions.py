import math
import unittest
from types import SimpleNamespace

import autopilot as ap
import simulate_functions as sim


def make_controller():
    return SimpleNamespace(
        state=ap.STATE_STRAIGHT,
        turn_sign=0.0,
        phase_timer=0,
        phase_duration=1,
        prev_max_dist=120.0,
        approach_entry_pos=0.0,
        target_pos_filtered=0.0,
        last_speed_ms=25.0,
        last_track_pos=0.0,
        off_track_timer=0,
        recovery_state="NONE",
        exit_arc_m=20.0,
        yaw_f=0.0,
        prev_steer=0.0,
    )


class TestAutopilotFunctions(unittest.TestCase):
    def test_state_transitions_straight_to_approach(self):
        c = make_controller()
        ap.update_state(c, alpha_deg=10.0, max_dist=90.0, speed_ms=30.0, turn_radius=120.0)
        self.assertEqual(c.state, ap.STATE_APPROACH)

    def test_target_lateral_turn_direction(self):
        c = make_controller()
        c.state = ap.STATE_APPROACH
        c.turn_sign = 1.0
        c.phase_timer = 10
        c.phase_duration = 40
        c.approach_entry_pos = 0.0
        target = ap.get_target_lateral(c)
        self.assertLess(target, 0.0)

    def test_target_speed_decreases_with_curvature(self):
        distances = [120.0] * 19
        speed_low_curve = ap.compute_target_speed(
            distances=distances,
            max_dist=120.0,
            curvature=0.002,
            mu_eff=ap.MU_EFFECTIVE,
            a_brake=ap.A_PHYS_BRAKE,
        )
        speed_high_curve = ap.compute_target_speed(
            distances=distances,
            max_dist=120.0,
            curvature=0.04,
            mu_eff=ap.MU_EFFECTIVE,
            a_brake=ap.A_PHYS_BRAKE,
        )
        self.assertGreater(speed_low_curve, speed_high_curve)

    def test_pedals_brake_when_too_fast(self):
        S = {"speedX": 150.0, "angle": 0.05, "rpm": 12000}
        R = {"steer": 0.05}
        accel, brake = ap.compute_pedals(
            S=S,
            R=R,
            target_speed=90.0,
            max_dist=80.0,
            max_thr=1.0,
            a_brake=ap.A_PHYS_BRAKE,
        )
        self.assertEqual(accel, 0.0)
        self.assertGreater(brake, 0.0)

    def test_pedals_accel_when_below_target(self):
        S = {"speedX": 60.0, "angle": 0.02, "rpm": 9000}
        R = {"steer": 0.02}
        accel, brake = ap.compute_pedals(
            S=S,
            R=R,
            target_speed=120.0,
            max_dist=120.0,
            max_thr=1.0,
            a_brake=ap.A_PHYS_BRAKE,
        )
        self.assertGreater(accel, 0.0)
        self.assertEqual(brake, 0.0)

    def test_steering_straight_damping_near_center(self):
        c = make_controller()
        c.state = ap.STATE_STRAIGHT
        S = {"angle": 0.0, "trackPos": 0.01}
        steer, _, _ = ap.compute_steering(S=S, target_pos=0.0, c=c)
        self.assertAlmostEqual(steer, 0.0, places=4)

    def test_turn_in_transitions_to_exit_on_opening(self):
        c = make_controller()
        c.state = ap.STATE_TURN_IN
        c.phase_timer = ap.PHASE_MIN_FRAMES + 12
        c.phase_duration = 40
        c.prev_max_dist = 36.4
        c.exit_arc_m = 20.0

        # Reproduces the reported situation: angle collapses, distance nearly flat.
        ap.update_state(c, alpha_deg=0.0, max_dist=36.4, speed_ms=17.0, turn_radius=50.0)
        self.assertEqual(c.state, ap.STATE_EXIT)

    def test_exit_recenters_when_near_edge(self):
        c = make_controller()
        c.state = ap.STATE_EXIT
        c.turn_sign = -1.0
        c.phase_timer = 20
        c.phase_duration = 40
        c.last_track_pos = -1.02
        target = ap.get_target_lateral(c)
        self.assertAlmostEqual(target, 0.0, places=6)

    def test_exit_force_to_straight_without_timer_delay(self):
        c = make_controller()
        c.state = ap.STATE_EXIT
        c.phase_timer = 2
        c.phase_duration = 50
        c.off_track_timer = 30
        c.last_track_pos = -1.12

        ap.update_state(c, alpha_deg=0.0, max_dist=36.0, speed_ms=17.0, turn_radius=60.0)
        self.assertEqual(c.state, ap.STATE_STRAIGHT)

    def test_exit_force_on_large_trackpos_even_low_offtrack_timer(self):
        c = make_controller()
        c.state = ap.STATE_EXIT
        c.phase_timer = 1
        c.phase_duration = 50
        c.off_track_timer = 3
        c.last_track_pos = 1.08

        ap.update_state(c, alpha_deg=1.0, max_dist=38.0, speed_ms=17.0, turn_radius=60.0)
        self.assertEqual(c.state, ap.STATE_STRAIGHT)

    def test_tight_corner_target_zero_when_near_edge(self):
        # Replay-like case from report: low speed, tiny max_dist, car already near track limit.
        constrained = ap.stabilize_target_dynamic(
            raw_target=-0.85,
            track_pos=0.995,
            max_dist=0.4,
            speed_kmh=14.0,
        )
        self.assertAlmostEqual(constrained, 0.0, places=4)

    def test_tight_corner_target_reduced_not_removed(self):
        raw = 0.82
        constrained = ap.stabilize_target_dynamic(
            raw_target=raw,
            track_pos=0.45,
            max_dist=6.5,
            speed_kmh=28.0,
        )
        self.assertLess(abs(constrained), abs(raw))
        self.assertGreater(abs(constrained), 0.05)

    def test_normal_corner_target_unchanged(self):
        raw = -0.62
        constrained = ap.stabilize_target_dynamic(
            raw_target=raw,
            track_pos=0.2,
            max_dist=40.0,
            speed_kmh=90.0,
        )
        self.assertAlmostEqual(constrained, raw, places=6)

    def test_recovery_starts_earlier_offtrack(self):
        c = make_controller()
        c.off_track_timer = 4
        c.recovery_state = "NONE"
        c.stuck_timer = 0
        c.step = 500
        c.recovery_timer = 0
        c.recovery_steer_dir = 0.0

        S = {"angle": 0.2}
        R = {"gear": 1, "brake": 0.0, "accel": 0.3, "steer": 0.0}
        handled = ap.update_recovery(c, S, R, track_pos=1.08, speed_x=30.0)
        self.assertTrue(handled)

    def test_corkscrew(self):
        rows = sim.run_corkscrew_simulation(speed_ms=28.0)
        self.assertGreater(len(rows), 150)

        states = {r["state"] for r in rows}
        self.assertIn(ap.STATE_APPROACH, states)
        self.assertIn(ap.STATE_TURN_IN, states)
        self.assertIn(ap.STATE_EXIT, states)

        # Lateral target should remain within track bounds in this offline reconstruction.
        self.assertLessEqual(max(abs(r["target_pos"]) for r in rows), 1.0)

    def test_corckscrew(self):
        # Alias with requested spelling.
        self.test_corkscrew()


if __name__ == "__main__":
    unittest.main(verbosity=2)
