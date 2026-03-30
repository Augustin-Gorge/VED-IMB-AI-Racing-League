import unittest

import pandas as pd

import telemetry_tools as tt


class TestTelemetryTools(unittest.TestCase):
    def test_compute_metrics_has_expected_fields(self):
        df = pd.DataFrame(
            {
                "state": ["APPROACH", "TURN_IN", "TURN_IN", "EXIT", "STRAIGHT"],
                "lap_time": [1.0, 2.0, 3.0, 4.0, 5.0],
                "speed_x": [120.0, 80.0, 70.0, 90.0, 140.0],
                "track_pos": [0.2, 0.4, 0.6, 0.3, 0.1],
                "max_dist": [60.0, 45.0, 30.0, 50.0, 100.0],
            }
        )
        m = tt.compute_metrics(df)
        self.assertGreater(m.lap_time_s, 0.0)
        self.assertGreater(m.avg_speed_kmh, 0.0)
        self.assertGreaterEqual(m.approach_to_turnin_count, 1)

    def test_compare_metrics_delta_sign(self):
        base = tt.RunMetrics(
            lap_time_s=110.0,
            offtrack_pct=0.0,
            avg_speed_kmh=120.0,
            max_abs_trackpos=0.8,
            approach_abs_trackpos_mean=0.25,
            turn_in_abs_trackpos_mean=0.35,
            turn_in_abs_trackpos_p90=0.60,
            turn_in_entry_max_dist_mean=55.0,
            apex_abs_trackpos_mean=0.45,
            approach_to_turnin_count=8,
            turnin_to_exit_count=8,
        )
        cand = tt.RunMetrics(
            lap_time_s=108.0,
            offtrack_pct=0.0,
            avg_speed_kmh=123.0,
            max_abs_trackpos=0.82,
            approach_abs_trackpos_mean=0.30,
            turn_in_abs_trackpos_mean=0.40,
            turn_in_abs_trackpos_p90=0.65,
            turn_in_entry_max_dist_mean=58.0,
            apex_abs_trackpos_mean=0.50,
            approach_to_turnin_count=9,
            turnin_to_exit_count=9,
        )
        comp = tt.compare_metrics(base, cand)
        lap_delta = float(comp[comp["metric"] == "lap_time_s"]["delta"].iloc[0])
        self.assertLess(lap_delta, 0.0)

    def test_score_penalizes_offtrack_heavily(self):
        safe = tt.RunMetrics(
            lap_time_s=108.0,
            offtrack_pct=0.0,
            avg_speed_kmh=122.0,
            max_abs_trackpos=0.9,
            approach_abs_trackpos_mean=0.30,
            turn_in_abs_trackpos_mean=0.42,
            turn_in_abs_trackpos_p90=0.70,
            turn_in_entry_max_dist_mean=56.0,
            apex_abs_trackpos_mean=0.52,
            approach_to_turnin_count=9,
            turnin_to_exit_count=9,
        )
        risky = tt.RunMetrics(
            lap_time_s=103.0,
            offtrack_pct=2.0,
            avg_speed_kmh=130.0,
            max_abs_trackpos=1.25,
            approach_abs_trackpos_mean=0.36,
            turn_in_abs_trackpos_mean=0.48,
            turn_in_abs_trackpos_p90=0.78,
            turn_in_entry_max_dist_mean=61.0,
            apex_abs_trackpos_mean=0.58,
            approach_to_turnin_count=9,
            turnin_to_exit_count=8,
        )
        self.assertLess(tt.score_run(safe), tt.score_run(risky))

    def test_compare_scores_prefers_faster_clean_run(self):
        base = tt.RunMetrics(
            lap_time_s=110.0,
            offtrack_pct=0.0,
            avg_speed_kmh=120.0,
            max_abs_trackpos=0.85,
            approach_abs_trackpos_mean=0.30,
            turn_in_abs_trackpos_mean=0.40,
            turn_in_abs_trackpos_p90=0.68,
            turn_in_entry_max_dist_mean=55.0,
            apex_abs_trackpos_mean=0.50,
            approach_to_turnin_count=8,
            turnin_to_exit_count=8,
        )
        cand = tt.RunMetrics(
            lap_time_s=108.5,
            offtrack_pct=0.0,
            avg_speed_kmh=123.0,
            max_abs_trackpos=0.88,
            approach_abs_trackpos_mean=0.32,
            turn_in_abs_trackpos_mean=0.43,
            turn_in_abs_trackpos_p90=0.72,
            turn_in_entry_max_dist_mean=57.0,
            apex_abs_trackpos_mean=0.54,
            approach_to_turnin_count=9,
            turnin_to_exit_count=9,
        )
        cmp = tt.compare_scores(base, cand)
        self.assertLess(cmp["delta"], 0.0)
        self.assertEqual(cmp["candidate_better"], 1.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
