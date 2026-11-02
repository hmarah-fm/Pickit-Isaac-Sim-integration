import os
import unittest

from pcd_converter.configuration import DEFAULT_PARAMETERS, validate_parameters


class ParameterTests(unittest.TestCase):
    def test_defaults_are_valid_and_cap_at_one_hz(self):
        validate_parameters(DEFAULT_PARAMETERS)
        expected_hz = float(os.environ.get("PUBLISH_HZ", 0.5))
        self.assertEqual(DEFAULT_PARAMETERS["publish_hz"], expected_hz)

    def test_rates_reject_nonpositive_and_nonfinite_values(self):
        for name in ("publish_hz", "registration_publish_hz", "sync_slop_seconds"):
            for value in (0.0, -1.0, float("nan"), float("inf")):
                with self.subTest(name=name, value=value):
                    with self.assertRaisesRegex(ValueError, name):
                        validate_parameters({**DEFAULT_PARAMETERS, name: value})

    def test_delays_and_sync_bounds(self):
        for name in ("registration_delay_seconds",):
            validate_parameters({**DEFAULT_PARAMETERS, name: 0.0})
            for value in (-0.1, float("nan"), float("inf")):
                with self.subTest(name=name, value=value):
                    with self.assertRaisesRegex(ValueError, name):
                        validate_parameters({**DEFAULT_PARAMETERS, name: value})
        with self.assertRaisesRegex(ValueError, "sync_queue_size"):
            validate_parameters({**DEFAULT_PARAMETERS, "sync_queue_size": 0})

    def test_registration_message_ranges(self):
        for name, value in (("camera_source", 256), ("camera_type", -1)):
            with self.subTest(name=name):
                with self.assertRaisesRegex(ValueError, name):
                    validate_parameters({**DEFAULT_PARAMETERS, name: value})

    def test_empty_topics_and_frames_are_rejected(self):
        for name, default in DEFAULT_PARAMETERS.items():
            if isinstance(default, str):
                with self.subTest(name=name):
                    with self.assertRaisesRegex(ValueError, name):
                        validate_parameters({**DEFAULT_PARAMETERS, name: " "})
