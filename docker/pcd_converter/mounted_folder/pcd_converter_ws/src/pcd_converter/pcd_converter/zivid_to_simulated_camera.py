#!/usr/bin/env python3
"""
Zivid-to-Pickit organized XYZRGB converter.
───────────────────────────────────
Synchronizes /ZividCamera/pointcloud (XYZ float32) with /ZividCamera/rgb,
republishes as organized XYZRGB PCL PointCloud2, and emits paired CameraInfo
and periodic CameraMetaInfo for Pickit.

Output rate, topics, synchronization, frames and registration are startup parameters.
CameraInfo is cached and published only with a cloud, using the same timestamp.
"""

import copy
import time
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy,
    QoSHistoryPolicy, QoSDurabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, CameraInfo, PointField, Image
from pickit_camera_msgs.msg import CameraMetaInfo
import message_filters
from pcd_converter.configuration import (
    DEFAULT_PARAMETERS,
    PARAMETER_DESCRIPTIONS,
    READONLY_PARAMETERS,
    DYNAMIC_PARAMETERS,
    validate_parameters,
)


# ── QoS ───────────────────────────────────────────────────────────────────────

BRIDGE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE,
)

REG_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

SUB_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
    durability=QoSDurabilityPolicy.VOLATILE,
)

OUTPUT_FIELDS = [
    PointField(name="x",   offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name="y",   offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name="z",   offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
]


# ── Helpers ───────────────────────────────────────────────────────────────────

def _get_field_offset(fields, name):
    for f in fields:
        if f.name == name:
            return f.offset
    return None


def image_to_rgb_array(img_msg: Image, step: int = 1) -> np.ndarray:
    enc = img_msg.encoding.lower()
    raw = np.frombuffer(img_msg.data, dtype=np.uint8)
    h, w = img_msg.height, img_msg.width

    if enc == "rgb8":
        arr = raw.reshape(h, w, 3)
        if step > 1:
            arr = arr[::step, ::step]
        return arr.reshape(-1, 3)
    elif enc == "bgr8":
        arr = raw.reshape(h, w, 3)
        if step > 1:
            arr = arr[::step, ::step]
        return arr[:, :, ::-1].reshape(-1, 3).copy()
    elif enc == "rgba8":
        arr = raw.reshape(h, w, 4)
        if step > 1:
            arr = arr[::step, ::step]
        return arr[:, :, :3].reshape(-1, 3).copy()
    elif enc == "bgra8":
        arr = raw.reshape(h, w, 4)
        if step > 1:
            arr = arr[::step, ::step]
        return arr[:, :, [2, 1, 0]].reshape(-1, 3).copy()
    elif enc == "mono8":
        arr = raw.reshape(h, w, 1)
        if step > 1:
            arr = arr[::step, ::step]
        return np.repeat(arr.reshape(-1, 1), 3, axis=1)
    else:
        raise ValueError(f"Unsupported image encoding: {img_msg.encoding!r}")


def pack_rgb_pcl(rgb: np.ndarray) -> np.ndarray:
    r = rgb[:, 0].astype(np.uint32)
    g = rgb[:, 1].astype(np.uint32)
    b = rgb[:, 2].astype(np.uint32)
    return ((r << 16) | (g << 8) | b).view(np.float32)


def _resolve_output_dims(pc_msg: PointCloud2, img_msg: Image):
    pc_n  = pc_msg.height * pc_msg.width
    img_h = img_msg.height
    img_w = img_msg.width
    img_n = img_h * img_w

    if pc_msg.height > 1:
        return pc_msg.height, pc_msg.width, "organized"
    if pc_n == img_n:
        return img_h, img_w, "flat→organized"
    return 1, pc_n, "unorganized(flat)"


# ── Node ──────────────────────────────────────────────────────────────────────

class ZividToSimulatedCamera(Node):

    def __init__(self, **kwargs):
        super().__init__("zivid_to_simulated_camera", **kwargs)
        self._config = {}
        for name, default in DEFAULT_PARAMETERS.items():
            read_only = name in READONLY_PARAMETERS
            desc = ParameterDescriptor(
                description=PARAMETER_DESCRIPTIONS.get(name, ""),
                read_only=read_only,
            )
            if self.has_parameter(name):
                self._config[name] = self.get_parameter(name).value
            else:
                self._config[name] = self.declare_parameter(name, default, desc).value
        try:
            validate_parameters(self._config)
        except ValueError as exc:
            self.get_logger().error(f"Invalid converter configuration: {exc}")
            self.destroy_node()
            raise
        self._publish_period = 1.0 / self._config["publish_hz"]
        self._registration_period = 1.0 / self._config["registration_publish_hz"]

        # ── Dynamic parameter reconfigure callback ────────────────────────────
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # Resolution and camera info cache
        self._cached_n_pts:  int | None = None
        self._cached_out_h:  int | None = None
        self._cached_out_w:  int | None = None
        self._cached_layout: str | None = None
        self._latest_ci_msg: CameraInfo | None = None
        self._latest_synced_pair: tuple[PointCloud2, Image] | None = None
        self._pending_ci: tuple[float, CameraInfo] | None = None
        self._next_publish_time = 0.0

        # ── Publishers ────────────────────────────────────────────────────────
        self.pc_pub = self.create_publisher(
            PointCloud2, self._config["output_pointcloud_topic"], BRIDGE_QOS)
        self.ci_pub = self.create_publisher(
            CameraInfo, self._config["output_camera_info_topic"], BRIDGE_QOS)
        self.reg_pub = self.create_publisher(
            CameraMetaInfo, self._config["registration_topic"], REG_QOS)

        # ── Synchronized subscribers ──────────────────────────────────────────
        pc_sub = message_filters.Subscriber(
            self, PointCloud2, self._config["input_pointcloud_topic"], qos_profile=SUB_QOS)
        rgb_sub = message_filters.Subscriber(
            self, Image, self._config["input_rgb_topic"], qos_profile=SUB_QOS)

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [pc_sub, rgb_sub],
            queue_size=self._config["sync_queue_size"],
            slop=self._config["sync_slop_seconds"],
        )
        self._sync.registerCallback(self._synced_cb)

        # Subscribe and cache incoming CameraInfo without direct pass-through
        self.create_subscription(
            CameraInfo, self._config["input_camera_info_topic"], self._ci_cb, SUB_QOS)

        # Periodic check timer to eliminate phase-slip when synced frames arrive slightly before _next_publish_time
        self._pub_timer = self.create_timer(0.05, self._check_publish_timer)

        # ── Camera registration timer ─────────────────────────────────────────
        delay = self._config["registration_delay_seconds"]
        if delay > 0:
            self._reg_timer = self.create_timer(delay, self._start_registration)
        else:
            self._publish_registration()
            self._reg_timer = self.create_timer(
                self._registration_period, self._publish_registration)

        self.get_logger().info(
            f"Converter ready: {self._config['input_pointcloud_topic']} + "
            f"{self._config['input_rgb_topic']} -> "
            f"{self._config['output_pointcloud_topic']} and "
            f"{self._config['output_camera_info_topic']} "
            f"@ {self._config['publish_hz']} Hz max; "
            f"registration @ {self._config['registration_publish_hz']} Hz"
        )

    # ── CameraInfo cache callback ─────────────────────────────────────────────

    def _ci_cb(self, msg: CameraInfo):
        """Cache latest CameraInfo; do NOT publish directly."""
        self._latest_ci_msg = msg

    # ── Camera registration ───────────────────────────────────────────────────

    def _start_registration(self):
        self.destroy_timer(self._reg_timer)
        self._publish_registration()
        self._reg_timer = self.create_timer(
            self._registration_period, self._publish_registration)

    def _publish_registration(self):
        msg = CameraMetaInfo()
        msg.serial = self._config["camera_serial"]
        msg.camera_namespace = self._config["camera_namespace"]
        msg.camera_type = self._config["camera_type"]
        msg.camera_source = self._config["camera_source"]
        msg.rgb_optical_frame = self._config["rgb_optical_frame"]
        msg.root_frame = self._config["root_frame"]
        msg.heartbeat_timeout_seconds = self._config["heartbeat_timeout_seconds"]

        self.reg_pub.publish(msg)
        self.get_logger().info(
            f"Camera registration sent to {self._config['registration_topic']}",
            throttle_duration_sec=30.0
        )

    # ── Resolution cache ──────────────────────────────────────────────────────

    def _update_resolution_cache(self, pc_msg: PointCloud2, img_msg: Image):
        n_pts = pc_msg.height * pc_msg.width
        if n_pts == self._cached_n_pts:
            return

        out_h, out_w, layout = _resolve_output_dims(pc_msg, img_msg)

        if self._cached_n_pts is None:
            D = self._get_effective_downsample_factor(out_w, out_h)
            eff_w = out_w // D if D > 1 else out_w
            eff_h = out_h // D if D > 1 else out_h
            self.get_logger().info(
                f"Resolution detected: PC {pc_msg.width}×{pc_msg.height} "
                f"| Image {img_msg.width}×{img_msg.height} "
                f"→ raw {out_w}×{out_h} (output factor {D} → {eff_w}×{eff_h}) layout={layout}"
            )

        self._cached_n_pts  = n_pts
        self._cached_out_h  = out_h
        self._cached_out_w  = out_w
        self._cached_layout = layout

    def _on_set_parameters(self, params):
        """Handle dynamic parameter changes at runtime (e.g. downsample_factor, publish_hz)."""
        for param in params:
            if param.name == "downsample_factor":
                val = param.value
                if not isinstance(val, int) or val < 1:
                    return SetParametersResult(
                        successful=False,
                        reason=f"downsample_factor must be an integer >= 1 (got {val})"
                    )
                old_val = self._config.get("downsample_factor", 1)
                self._config["downsample_factor"] = val
                self._cached_n_pts = None  # trigger layout re-detection log
                raw_w = self._cached_out_w or 1024
                raw_h = self._cached_out_h or 640
                new_w = len(range(0, raw_w, val))
                new_h = len(range(0, raw_h, val))
                est_bytes = new_w * new_h * 16
                self.get_logger().info(
                    f"Parameter 'downsample_factor' updated: {old_val} -> {val} "
                    f"(target resolution: {new_w}x{new_h}, {est_bytes} bytes per frame)"
                )
            elif param.name == "publish_hz":
                val = param.value
                if not isinstance(val, (int, float)) or val <= 0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"publish_hz must be a number > 0 (got {val})"
                    )
                old_val = self._config.get("publish_hz", 0.5)
                self._config["publish_hz"] = float(val)
                self._publish_period = 1.0 / float(val)
                self.get_logger().info(
                    f"Parameter 'publish_hz' updated: {old_val} -> {val} Hz "
                    f"(period: {self._publish_period:.2f}s)"
                )
            elif param.name in READONLY_PARAMETERS:
                return SetParametersResult(
                    successful=False,
                    reason=f"Parameter '{param.name}' is read-only at runtime"
                )
        return SetParametersResult(successful=True)

    def _get_effective_downsample_factor(self, w: int, h: int) -> int:
        try:
            D = int(self._config.get("downsample_factor", 1))
        except (ValueError, TypeError):
            D = 1
        return max(1, D)

    # ── Synchronized PC + RGB callback ───────────────────────────────────────

    def _synced_cb(self, pc_msg: PointCloud2, img_msg: Image):
        # Always cache the latest synchronized pair so frames arriving slightly
        # before _next_publish_time are published immediately on schedule without phase-slip.
        self._latest_synced_pair = (pc_msg, img_msg)
        self._check_publish_timer()

    def _check_publish_timer(self):
        now = time.monotonic()

        # Flush delayed CameraInfo so it queues behind the PointCloud2 on the TCP link
        if self._pending_ci is not None and now >= self._pending_ci[0]:
            _, ci_msg = self._pending_ci
            self._pending_ci = None
            self.ci_pub.publish(ci_msg)

        if self._latest_synced_pair is None:
            return
        # A wall-clock cap must hold even when simulation time jumps or accelerates.
        if now < self._next_publish_time:
            return

        if self._latest_ci_msg is None:
            self.get_logger().warning(
                "Waiting for camera calibration; not publishing uncalibrated clouds",
                throttle_duration_sec=5.0,
            )
            return

        pc_msg, img_msg = self._latest_synced_pair
        self._latest_synced_pair = None

        self._update_resolution_cache(pc_msg, img_msg)

        try:
            # 1. Convert PointCloud2 first
            out_pc = self._convert(pc_msg, img_msg)
            payload_bytes = len(out_pc.data)

            # Pace publication to never exceed the ~1.15 MB/s VPN link capacity (750 KB/s safe ceiling)
            min_drain_period = payload_bytes / 750_000.0
            effective_period = max(self._publish_period, min_drain_period)
            self._next_publish_time = time.monotonic() + effective_period

            # Compensate for multi-megabyte VPN serialization delay (~2.3s for 2.62 MB)
            # so header.stamp is fresh when the final TCP fragment arrives at Pickit
            transit_offset_ns = (
                int(min(1.8, payload_bytes / 1_500_000.0) * 1e9)
                if payload_bytes > 800_000
                else 0
            )
            stamp = (self.get_clock().now() + Duration(nanoseconds=transit_offset_ns)).to_msg()
            out_pc.header.stamp = stamp
            self.pc_pub.publish(out_pc)

            # 2. Schedule paired CameraInfo 200ms later so zenoh_bridge_connector finishes reading
            #    the 2.62 MB PointCloud2 from local DDS (~173ms) and queues it on TCP first.
            ci_msg = self._build_synced_camera_info(stamp, out_pc.width, out_pc.height)
            self._pending_ci = (time.monotonic() + 0.20, ci_msg)

            self.get_logger().info(
                f"Published synced cloud ({out_pc.width}x{out_pc.height}, {payload_bytes} bytes, "
                f"period={effective_period:.2f}s) + CameraInfo",
                throttle_duration_sec=5.0
            )

        except (ValueError, TypeError, IndexError) as e:
            self._next_publish_time = time.monotonic() + self._publish_period
            self.get_logger().error(
                f"Conversion failed: {e}",
                throttle_duration_sec=2.0,
            )

    def _build_synced_camera_info(self, stamp, out_w: int, out_h: int) -> CameraInfo:
        """Builds CameraInfo using the exact timestamp and dimensions of the PointCloud2."""
        ci_msg = copy.deepcopy(self._latest_ci_msg)

        ci_msg.header.stamp = stamp
        ci_msg.header.frame_id = self._config["output_frame_id"]

        raw_w = self._cached_out_w
        raw_h = self._cached_out_h

        # Set exact dimensions to match out_pc
        ci_msg.width = out_w
        ci_msg.height = out_h

        if raw_w and raw_h and (out_w != raw_w or out_h != raw_h):
            scale_x = float(out_w) / float(raw_w)
            scale_y = float(out_h) / float(raw_h)

            k = list(ci_msg.k)
            k[0] *= scale_x  # fx
            k[2] *= scale_x  # cx
            k[4] *= scale_y  # fy
            k[5] *= scale_y  # cy
            ci_msg.k = k

            p = list(ci_msg.p)
            p[0] *= scale_x  # fx
            p[2] *= scale_x  # cx
            p[5] *= scale_y  # fy
            p[6] *= scale_y  # cy
            ci_msg.p = p

        return ci_msg

    def _convert(self, pc_msg: PointCloud2, img_msg: Image) -> PointCloud2:
        n_pts      = pc_msg.height * pc_msg.width
        point_step = pc_msg.point_step
        out_h      = self._cached_out_h
        out_w      = self._cached_out_w

        D = self._get_effective_downsample_factor(out_w, out_h)
        raw_buf = np.frombuffer(pc_msg.data, dtype=np.uint8)

        if D > 1 and out_h > 1 and out_w > 1:
            raw_2d = raw_buf.reshape(out_h, out_w, point_step)[::D, ::D]
            out_h, out_w = raw_2d.shape[:2]
            raw = np.ascontiguousarray(raw_2d).reshape(out_h * out_w, point_step)
            rgb_array = image_to_rgb_array(img_msg, step=D)
        else:
            raw = raw_buf.reshape(n_pts, point_step)
            rgb_array = image_to_rgb_array(img_msg, step=1)

        x_off = _get_field_offset(pc_msg.fields, "x")
        y_off = _get_field_offset(pc_msg.fields, "y")
        z_off = _get_field_offset(pc_msg.fields, "z")

        x = raw[:, x_off:x_off+4].view(np.float32).reshape(-1).copy()
        y = raw[:, y_off:y_off+4].view(np.float32).reshape(-1).copy()
        z = raw[:, z_off:z_off+4].view(np.float32).reshape(-1).copy()

        invalid = np.isinf(x) | np.isinf(y) | np.isinf(z)
        x[invalid] = np.nan
        y[invalid] = np.nan
        z[invalid] = np.nan

        rgb_as_float = pack_rgb_pcl(rgb_array)
        rgb_as_float[invalid] = np.uint32(0).view(np.float32)

        out = np.empty((out_h * out_w, 4), dtype=np.float32)
        out[:, 0] = x
        out[:, 1] = y
        out[:, 2] = z
        out[:, 3] = rgb_as_float

        out_msg = PointCloud2()
        out_msg.header.frame_id = self._config["output_frame_id"]
        out_msg.height          = out_h
        out_msg.width           = out_w
        out_msg.is_dense        = False
        out_msg.is_bigendian    = False
        out_msg.point_step      = 16
        out_msg.row_step        = 16 * out_w
        out_msg.fields          = OUTPUT_FIELDS
        out_msg.data            = out.tobytes()
        return out_msg


def main():
    rclpy.init()
    node = None
    try:
        node = ZividToSimulatedCamera()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
