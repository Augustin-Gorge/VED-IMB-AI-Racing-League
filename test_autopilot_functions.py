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
