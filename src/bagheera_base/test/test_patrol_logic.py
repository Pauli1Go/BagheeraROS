import unittest

from bagheera_base.battery_math import CRITICAL, FULL, LOW, NORMAL
from bagheera_base.patrol_logic import (
    CHARGE,
    DOCK,
    DOCK_AND_FINISH,
    DOCK_AND_STOP,
    DOCK_CYCLE,
    NAVIGATE,
    ONCE,
    PAUSE,
    WAIT_FOR_FULL,
    PatrolPlan,
    is_transient_failure,
    parse_waypoints,
)


class ParseWaypointsTest(unittest.TestCase):
    def test_triples(self):
        self.assertEqual(parse_waypoints([1, 2, 0.5, 3, 4, -1]), [(1.0, 2.0, 0.5), (3.0, 4.0, -1.0)])

    def test_rejects_incomplete_or_empty(self):
        with self.assertRaises(ValueError):
            parse_waypoints([1.0, 2.0])
        with self.assertRaises(ValueError):
            parse_waypoints([])
        with self.assertRaises(ValueError):
            parse_waypoints([1.0, float("nan"), 0.0])


class ChargeModeTest(unittest.TestCase):
    def test_loops_until_low_then_docks_after_the_waypoint(self):
        plan = PatrolPlan(3, CHARGE)
        self.assertEqual(plan.start(NORMAL, docked=True), NAVIGATE)
        for _ in range(4):
            self.assertEqual(plan.waypoint_done(True, NORMAL), NAVIGATE)
        self.assertEqual((plan.laps, plan.index), (1, 1))
        self.assertEqual(plan.waypoint_done(True, LOW), DOCK)
        # Resume at the waypoint after the one just reached.
        self.assertEqual(plan.index, 2)
        # Charging clears LOW in the monitor; the plan still waits for FULL.
        self.assertEqual(plan.docked(), WAIT_FOR_FULL)
        plan.charged()
        self.assertEqual(plan.waypoint_done(True, NORMAL), NAVIGATE)

    def test_critical_keeps_the_abandoned_waypoint(self):
        plan = PatrolPlan(3, CHARGE)
        plan.waypoint_done(True, NORMAL)
        self.assertEqual(plan.critical_during_navigation(), DOCK)
        self.assertEqual(plan.index, 1)
        self.assertEqual(plan.docked(), WAIT_FOR_FULL)

    def test_low_at_start(self):
        self.assertEqual(PatrolPlan(3, CHARGE).start(LOW, docked=False), DOCK)
        self.assertEqual(PatrolPlan(3, CHARGE).start(CRITICAL, docked=True), WAIT_FOR_FULL)
        self.assertEqual(PatrolPlan(3, CHARGE).start(FULL, docked=True), NAVIGATE)


class DockCycleModeTest(unittest.TestCase):
    def test_docks_after_each_lap_and_pauses(self):
        plan = PatrolPlan(2, DOCK_CYCLE)
        self.assertEqual(plan.waypoint_done(True, NORMAL), NAVIGATE)
        self.assertEqual(plan.waypoint_done(True, NORMAL), DOCK)
        self.assertEqual((plan.laps, plan.index), (1, 0))
        self.assertEqual(plan.docked(), PAUSE)

    def test_low_battery_charges_instead_of_pausing(self):
        plan = PatrolPlan(2, DOCK_CYCLE)
        self.assertEqual(plan.waypoint_done(True, LOW), DOCK)
        self.assertEqual(plan.docked(), WAIT_FOR_FULL)
        plan.charged()
        plan.waypoint_done(True, NORMAL)
        self.assertEqual(plan.docked(), PAUSE)


class FailureTest(unittest.TestCase):
    def test_skips_and_stops_after_consecutive_failures(self):
        plan = PatrolPlan(5, CHARGE, max_consecutive_failures=3)
        self.assertEqual(plan.waypoint_done(False, NORMAL), NAVIGATE)
        self.assertEqual(plan.waypoint_done(False, NORMAL), NAVIGATE)
        self.assertEqual(plan.waypoint_done(True, NORMAL), NAVIGATE)
        self.assertEqual(plan.waypoint_done(False, NORMAL), NAVIGATE)
        self.assertEqual(plan.waypoint_done(False, NORMAL), NAVIGATE)
        self.assertEqual(plan.waypoint_done(False, NORMAL), DOCK_AND_STOP)
        self.assertEqual((plan.reached, plan.skipped), (1, 5))

    def test_classifies_logged_nav2_failures(self):
        # error_msg texts from the patrol runs of 2026-09-24.
        for detail in (
            "Costmap timed out waiting for update",
            "GridBasedplugin failed to plan calculation to (-2.23, 5.79): "
            "Failed to create plan with tolerance of: 0.200000",
            "Unable to transform robot pose into global plan's frame",
            "Lookup would require extrapolation into the future",
            "goal rejected by Nav2",
        ):
            self.assertTrue(is_transient_failure(detail), detail)
        for detail in (
            "Controller patience exceeded",
            "Failed to make progress",
            "Collision ahead",
            "waypoint timeout 600 s",
            "error code 0",
        ):
            self.assertFalse(is_transient_failure(detail), detail)

    def test_transient_failure_retries_before_it_skips(self):
        plan = PatrolPlan(5, CHARGE, max_consecutive_failures=3, max_transient_retries=2)
        self.assertTrue(plan.retry_after("Costmap timed out waiting for update"))
        self.assertTrue(plan.retry_after("Costmap timed out waiting for update"))
        self.assertFalse(plan.retry_after("Costmap timed out waiting for update"))
        self.assertEqual((plan.index, plan.retries, plan.retried), (0, 2, 2))
        self.assertEqual(plan.waypoint_done(False, NORMAL), NAVIGATE)
        # The next waypoint gets its own retries.
        self.assertEqual(plan.retries, 0)
        self.assertTrue(plan.retry_after("Costmap timed out waiting for update"))

    def test_real_obstacle_skips_at_once(self):
        plan = PatrolPlan(5, CHARGE)
        self.assertFalse(plan.retry_after("Controller patience exceeded"))
        self.assertEqual(plan.retries, 0)

    def test_retried_success_resets_the_failure_streak(self):
        # The 13:24 run: three late-data aborts in a row no longer end the
        # patrol when the retries get through.
        plan = PatrolPlan(5, CHARGE, max_consecutive_failures=3)
        self.assertEqual(plan.waypoint_done(False, NORMAL), NAVIGATE)
        self.assertTrue(plan.retry_after("GridBasedplugin failed to plan"))
        self.assertEqual(plan.waypoint_done(True, NORMAL), NAVIGATE)
        self.assertEqual(plan.consecutive_failures, 0)

    def test_critical_resets_the_waypoint_retries(self):
        plan = PatrolPlan(5, CHARGE)
        plan.retry_after("Costmap timed out waiting for update")
        self.assertEqual(plan.critical_during_navigation(), DOCK)
        self.assertEqual(plan.retries, 0)

    def test_once_docks_and_finishes_after_one_lap(self):
        plan = PatrolPlan(2, ONCE)
        self.assertEqual(plan.start(NORMAL, docked=True), NAVIGATE)
        self.assertEqual(plan.waypoint_done(True, NORMAL), NAVIGATE)
        self.assertEqual(plan.waypoint_done(True, NORMAL), DOCK_AND_FINISH)
        self.assertEqual(plan.laps, 1)

    def test_rejects_bad_setup(self):
        with self.assertRaises(ValueError):
            PatrolPlan(0, CHARGE)
        with self.assertRaises(ValueError):
            PatrolPlan(3, "sightseeing")


if __name__ == "__main__":
    unittest.main()
