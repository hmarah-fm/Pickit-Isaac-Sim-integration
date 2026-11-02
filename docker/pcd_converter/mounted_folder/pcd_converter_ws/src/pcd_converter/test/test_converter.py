import struct
import unittest
from unittest.mock import Mock, patch

import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

from pcd_converter.zivid_to_simulated_camera import ZividToSimulatedCamera


class ConverterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = ZividToSimulatedCamera(parameter_overrides=[
            Parameter("publish_hz", value=2.0),
            Parameter("input_pointcloud_topic", value="/test/input"),
            Parameter("output_pointcloud_topic", value="/test/output"),
            Parameter("output_frame_id", value="test_optical_frame"),
            Parameter("camera_serial", value="test_serial"),
            Parameter("registration_delay_seconds", value=0.1),
            Parameter("registration_publish_hz", value=0.5),
        ])
        self.addCleanup(self.node.destroy_node)
        self.pc = PointCloud2(
            height=1, width=2, point_step=12, row_step=24,
            fields=[
                PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                for name, offset in (("x", 0), ("y", 4), ("z", 8))
            ],
            data=struct.pack("<6f", 1, 2, 3, 4, 5, 6),
        )
        self.image = Image(
            height=1, width=2, encoding="rgb8", step=6,
            data=bytes([255, 0, 0, 0, 255, 0]),
        )
        self.calibration = CameraInfo(
            height=1, width=2,
            k=[500.0, 0.0, 1.0, 0.0, 500.0, 0.5, 0.0, 0.0, 1.0],
        )

    def capture_publishers(self):
        self.node.pc_pub = Mock()
        self.node.ci_pub = Mock()
        self.node.reg_pub = Mock()

    def test_parameter_overrides_reach_ros_endpoints(self):
        self.assertEqual(self.node.pc_pub.topic_name, "/test/output")
        self.assertIn("/test/input", [sub.topic_name for sub in self.node.subscriptions])
        self.assertEqual(self.node._publish_period, 0.5)
        self.assertEqual(self.node._registration_period, 2.0)
        self.assertEqual(self.node._reg_timer.timer_period_ns, 100_000_000)

    def test_invalid_rate_fails_startup(self):
        with self.assertRaisesRegex(ValueError, "publish_hz"):
            ZividToSimulatedCamera(parameter_overrides=[
                Parameter("publish_hz", value=0.0),
            ])

    def test_cloud_and_calibration_have_identical_headers(self):
        self.capture_publishers()
        self.node._ci_cb(self.calibration)
        self.node._synced_cb(self.pc, self.image)
        cloud = self.node.pc_pub.publish.call_args.args[0]
        info = self.node.ci_pub.publish.call_args.args[0]
        self.assertEqual(cloud.header, info.header)
        self.assertEqual(cloud.header.frame_id, "test_optical_frame")
        self.assertEqual(list(info.k), list(self.calibration.k))
        self.assertEqual(self.calibration.header.frame_id, "")
        self.assertEqual((cloud.height, cloud.width, cloud.point_step, cloud.row_step),
                         (1, 2, 16, 32))
        self.assertEqual(struct.unpack_from("<3fI", bytes(cloud.data)), (1, 2, 3, 0xFF0000))
        self.assertEqual(struct.unpack_from("<3fI", bytes(cloud.data), 16), (4, 5, 6, 0x00FF00))

    def test_strict_wall_clock_cap_does_not_catch_up_after_stalls(self):
        self.capture_publishers()
        self.node._ci_cb(self.calibration)
        with patch("pcd_converter.zivid_to_simulated_camera.time.monotonic") as clock:
            for value in (10.0, 10.1, 10.49, 10.5, 100.0, 100.1, 100.49):
                clock.return_value = value
                self.node._synced_cb(self.pc, self.image)
        self.assertEqual(self.node.pc_pub.publish.call_count, 3)
        self.assertEqual(self.node.ci_pub.publish.call_count, 3)

    def test_no_fabricated_calibration_when_input_is_missing(self):
        self.capture_publishers()
        self.node._synced_cb(self.pc, self.image)
        self.node.pc_pub.publish.assert_not_called()
        self.node.ci_pub.publish.assert_not_called()

    def test_camera_info_is_cached_not_passed_through(self):
        self.capture_publishers()
        for _ in range(10):
            self.node._ci_cb(self.calibration)
        self.node.ci_pub.publish.assert_not_called()

    def test_registration_uses_parameters_and_changes_to_periodic_timer(self):
        self.capture_publishers()
        self.node._start_registration()
        msg = self.node.reg_pub.publish.call_args.args[0]
        self.assertEqual(msg.serial, "test_serial")
        self.assertEqual(msg.camera_source, 1)
        self.assertEqual(self.node._reg_timer.timer_period_ns, 2_000_000_000)
