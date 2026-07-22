import unittest

from power_monitor import PowerMonitor


class FakeINA226:
    def __init__(self, sample):
        self.sample = dict(sample)
        self.reset_count = 0

    def read_sample(self):
        return dict(self.sample)

    def reset_peak(self):
        self.reset_count += 1


class PowerMonitorTests(unittest.TestCase):
    def test_negative_hardware_current_is_reported_as_consumption_magnitude(self):
        hardware = FakeINA226({
            "current_a": -0.325,
            "bus_voltage_v": 5.0,
            # The unsigned chip Power register may be invalid in reverse;
            # the wrapper must use Vbus * abs(I) instead.
            "power_w": 40.0,
            "pmax_w": 40.0,
            "overflow": False,
        })
        monitor = PowerMonitor(
            smoothing=1.0,
            hardware_monitor=hardware,
            hardware_poll_interval_ms=0,
        )

        self.assertTrue(monitor.poll_hardware(force=True, timestamp_ms=100))
        status = monitor.status()
        self.assertAlmostEqual(status["current_a"], 0.325)
        self.assertAlmostEqual(status["current_w"], 1.625)
        self.assertAlmostEqual(status["maximum_w"], 1.625)
        self.assertEqual(status["current_direction"], -1)
        self.assertEqual(
            status["hardware_warning"],
            "INA226_REVERSED_CURRENT",
        )
        self.assertIsNone(status["hardware_error"])

    def test_positive_hardware_current_has_no_direction_warning(self):
        hardware = FakeINA226({
            "current_a": 0.4,
            "power_w": 2.0,
            "pmax_w": 2.0,
            "overflow": False,
        })
        monitor = PowerMonitor(
            smoothing=1.0,
            hardware_monitor=hardware,
            hardware_poll_interval_ms=0,
        )

        self.assertTrue(monitor.poll_hardware(force=True, timestamp_ms=100))
        status = monitor.status()
        self.assertAlmostEqual(status["current_a"], 0.4)
        self.assertEqual(status["current_direction"], 1)
        self.assertIsNone(status["hardware_warning"])

    def test_hardware_current_offset_updates_current_and_bus_power(self):
        hardware = FakeINA226({
            "current_a": 0.4,
            "bus_voltage_v": 5.0,
            "power_w": 2.0,
            "pmax_w": 2.0,
            "overflow": False,
        })
        monitor = PowerMonitor(
            smoothing=1.0,
            hardware_monitor=hardware,
            hardware_poll_interval_ms=0,
            current_offset_a=0.077,
        )

        self.assertTrue(monitor.poll_hardware(force=True, timestamp_ms=100))
        status = monitor.status()
        self.assertAlmostEqual(status["current_a"], 0.477)
        self.assertAlmostEqual(status["current_w"], 2.385)


if __name__ == "__main__":
    unittest.main()
