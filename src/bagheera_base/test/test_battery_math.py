import unittest

from bagheera_base.battery_math import (
    CRITICAL,
    FULL,
    LOW,
    NORMAL,
    BatteryLevelTracker,
    TimeWindowAverage,
    percentage_from_voltage,
)


def feed(tracker, start, duration, voltage, docked=False, current=0.0, step=0.25):
    """Feed constant samples at 4 Hz; returns the time after the last one."""
    now = start
    while now <= start + duration:
        tracker.update(now, voltage, docked, current)
        now += step
    return now


class PercentageTest(unittest.TestCase):
    def test_table_points(self):
        self.assertAlmostEqual(percentage_from_voltage(7 * 3.61, 7), 20.0)
        self.assertAlmostEqual(percentage_from_voltage(7 * 3.50, 7), 10.0)
        self.assertAlmostEqual(percentage_from_voltage(29.4, 7), 100.0)

    def test_interpolates_and_clamps(self):
        self.assertAlmostEqual(percentage_from_voltage(7 * 3.735, 7), 45.0)
        self.assertEqual(percentage_from_voltage(18.0, 7), 0.0)
        self.assertEqual(percentage_from_voltage(30.5, 7), 100.0)


class TimeWindowAverageTest(unittest.TestCase):
    def test_old_samples_expire(self):
        average = TimeWindowAverage(10.0)
        average.add(0.0, 100.0)
        average.add(5.0, 20.0)
        average.add(12.0, 30.0)
        self.assertAlmostEqual(average.mean(), 25.0)
        self.assertAlmostEqual(average.span(), 7.0)


class BatteryLevelTrackerTest(unittest.TestCase):
    def test_no_voltage_before_warmup(self):
        tracker = BatteryLevelTracker()
        feed(tracker, 0.0, 5.0, 24.0)
        self.assertIsNone(tracker.voltage)
        self.assertEqual(tracker.level, NORMAL)

    def test_short_dip_does_not_trigger_low(self):
        tracker = BatteryLevelTracker()
        now = feed(tracker, 0.0, 60.0, 25.6)
        now = feed(tracker, now, 3.0, 24.0)
        self.assertEqual(tracker.level, NORMAL)

    def test_low_then_critical_latch(self):
        tracker = BatteryLevelTracker()
        now = feed(tracker, 0.0, 70.0, 25.2)
        self.assertEqual(tracker.level, LOW)
        now = feed(tracker, now, 70.0, 26.0)  # recovers at rest
        self.assertEqual(tracker.level, LOW)
        now = feed(tracker, now, 70.0, 24.3)
        self.assertEqual(tracker.level, CRITICAL)
        feed(tracker, now, 70.0, 25.0)
        self.assertEqual(tracker.level, CRITICAL)

    def test_charging_clears_low(self):
        tracker = BatteryLevelTracker()
        now = feed(tracker, 0.0, 70.0, 24.3)
        self.assertEqual(tracker.level, CRITICAL)
        now = feed(tracker, now, 20.0, 25.0, docked=True, current=0.0)
        self.assertEqual(tracker.level, CRITICAL)
        feed(tracker, now, 20.0, 25.5, docked=True, current=1.1)
        self.assertEqual(tracker.level, NORMAL)

    def test_docked_without_charge_still_warns(self):
        tracker = BatteryLevelTracker()
        feed(tracker, 0.0, 70.0, 25.0, docked=True, current=0.0)
        self.assertEqual(tracker.level, LOW)

    def test_full_needs_hold_and_ends_on_undock(self):
        tracker = BatteryLevelTracker(full_hold_s=120.0)
        now = feed(tracker, 0.0, 60.0, 28.5, docked=True, current=0.5)
        self.assertEqual(tracker.level, NORMAL)
        now = feed(tracker, now, 60.0, 28.5, docked=True, current=0.05)
        self.assertEqual(tracker.level, NORMAL)
        now = feed(tracker, now, 90.0, 28.5, docked=True, current=0.05)
        self.assertEqual(tracker.level, FULL)
        feed(tracker, now, 20.0, 28.2, docked=False)
        self.assertEqual(tracker.level, NORMAL)

    def test_undock_resets_average(self):
        tracker = BatteryLevelTracker()
        now = feed(tracker, 0.0, 60.0, 28.5, docked=True, current=1.0)
        feed(tracker, now, 12.0, 27.6, docked=False)
        self.assertAlmostEqual(tracker.voltage, 27.6)

    def test_rejects_inverted_thresholds(self):
        with self.assertRaises(ValueError):
            BatteryLevelTracker(low_voltage=24.0, critical_voltage=24.5)


if __name__ == "__main__":
    unittest.main()
