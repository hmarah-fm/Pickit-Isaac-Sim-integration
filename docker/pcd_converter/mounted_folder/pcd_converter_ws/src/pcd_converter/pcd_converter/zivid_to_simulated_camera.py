#!/usr/bin/env python3
"""
zivid_to_simulated_camera.py  v1.7
───────────────────────────────────
Synchronizes /ZividCamera/pointcloud (XYZ float32) with /ZividCamera/rgb,
republishes as organized XYZRGB PCL PointCloud2, and emits paired CameraInfo
and periodic CameraMetaInfo for Pickit.

Changes in v1.7:
  - Synchronizes output CameraInfo rate (2.0 Hz) and timestamp with PointCloud2.
  - Caches input CameraInfo and re-stamps it synchronously upon publishing cloud.
  - Eliminates 18 Hz CameraInfo flood causing buffer drops on Pickit processor.
"""

import copy
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy,
    QoSHistoryPolicy, QoSDurabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, CameraInfo, PointField, Image
from pickit_camera_msgs.msg import CameraMetaInfo
import message_filters


# ── Configuration ─────────────────────────────────────────────────────────────

OUTPUT_FRAME_ID = "simulated_camera/simulated_camera_rgb_optical_frame"
OUTPUT_PC_TOPIC = "/simulated_camera/depth_registered/points_3d_rectified"
OUTPUT_CI_TOPIC = "/simulated_camera/depth_registered/camera_info"
REG_TOPIC       = "/pickit/camera_registrations"

INPUT_PC_TOPIC  = "/ZividCamera/pointcloud"
INPUT_RGB_TOPIC = "/ZividCamera/rgb"
INPUT_CI_TOPIC  = "/ZividCamera/camera_info"

# Camera registration values — must never change
REG_SERIAL             = "1-simulated_camera"
REG_NAMESPACE          = "/simulated_camera"
REG_CAMERA_TYPE        = 1
REG_CAMERA_SOURCE      = 1    # MUST be 1 for virtual cameras
REG_RGB_OPTICAL_FRAME  = "simulated_camera/simulated_camera_rgb_optical_frame"
REG_ROOT_FRAME         = "simulated_camera/simulated_camera_link"
REG_HEARTBEAT_TIMEOUT  = -1.0  # -1.0 disables heartbeat for virtual cameras

# Camera registration publishing rate
REG_PUBLISH_HZ     = 0.2
REG_PUBLISH_PERIOD = 1.0 / REG_PUBLISH_HZ  # 5.0 seconds

# Hard output rate cap
PUBLISH_HZ     = 2.0
PUBLISH_PERIOD = 1.0 / PUBLISH_HZ

# ApproximateTimeSynchronizer tolerance (seconds)
SYNC_SLOP = 0.1


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


def image_to_rgb_array(img_msg: Image) -> np.ndarray:
    enc = img_msg.encoding.lower()
    raw = np.frombuffer(bytes(img_msg.data), dtype=np.uint8)
    h, w = img_msg.height, img_msg.width

    if enc == "rgb8":
        return raw.reshape(h * w, 3)
    elif enc == "bgr8":
        return raw.reshape(h * w, 3)[:, ::-1].copy()
    elif enc == "rgba8":
        return raw.reshape(h * w, 4)[:, :3].copy()
    elif enc == "bgra8":
        return raw.reshape(h * w, 4)[:, [2, 1, 0]].copy()
    elif enc == "mono8":
        return np.repeat(raw.reshape(h * w, 1), 3, axis=1)
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

    def __init__(self):
        super().__init__("zivid_to_simulated_camera")

        # Resolution and camera info cache
        self._cached_n_pts:  int | None = None
        self._cached_out_h:  int | None = None
        self._cached_out_w:  int | None = None
        self._cached_layout: str | None = None
        self._latest_ci_msg: CameraInfo | None = None
        self._next_publish_time = 0.0

        # ── Publishers ────────────────────────────────────────────────────────
        self.pc_pub  = self.create_publisher(PointCloud2,   OUTPUT_PC_TOPIC, BRIDGE_QOS)
        self.ci_pub  = self.create_publisher(CameraInfo,    OUTPUT_CI_TOPIC, BRIDGE_QOS)
        self.reg_pub = self.create_publisher(CameraMetaInfo, REG_TOPIC,       REG_QOS)

        # ── Synchronized subscribers ──────────────────────────────────────────
        pc_sub  = message_filters.Subscriber(self, PointCloud2, INPUT_PC_TOPIC,  qos_profile=SUB_QOS)
        rgb_sub = message_filters.Subscriber(self, Image,        INPUT_RGB_TOPIC, qos_profile=SUB_QOS)

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [pc_sub, rgb_sub],
            queue_size=5,
            slop=SYNC_SLOP,
        )
        self._sync.registerCallback(self._synced_cb)

        # Subscribe and cache incoming CameraInfo without direct pass-through
        self.create_subscription(CameraInfo, INPUT_CI_TOPIC, self._ci_cb, SUB_QOS)

        # ── Camera registration timer ─────────────────────────────────────────
        self._reg_timer = self.create_timer(REG_PUBLISH_PERIOD, self._publish_registration)

        self.get_logger().info(
            f"zivid_to_simulated_camera v1.7 ready\n"
            f"  SUB  {INPUT_PC_TOPIC}  +  {INPUT_RGB_TOPIC}  (synchronized, slop={SYNC_SLOP}s)\n"
            f"  SUB  {INPUT_CI_TOPIC}  (cached)\n"
            f"  PUB  {OUTPUT_PC_TOPIC}  @ {PUBLISH_HZ} Hz max\n"
            f"  PUB  {OUTPUT_CI_TOPIC}  @ {PUBLISH_HZ} Hz (synced with PointCloud2)\n"
            f"  PUB  {REG_TOPIC}  (continuously @ {REG_PUBLISH_HZ} Hz)"
        )

    # ── CameraInfo cache callback ─────────────────────────────────────────────

    def _ci_cb(self, msg: CameraInfo):
        """Cache latest CameraInfo; do NOT publish directly."""
        self._latest_ci_msg = msg

    # ── Camera registration ───────────────────────────────────────────────────

    def _publish_registration(self):
        msg = CameraMetaInfo()
        msg.serial               = REG_SERIAL
        msg.camera_namespace     = REG_NAMESPACE
        msg.camera_type          = REG_CAMERA_TYPE
        msg.camera_source        = REG_CAMERA_SOURCE
        msg.rgb_optical_frame    = REG_RGB_OPTICAL_FRAME
        msg.root_frame           = REG_ROOT_FRAME
        msg.heartbeat_timeout_seconds = REG_HEARTBEAT_TIMEOUT

        self.reg_pub.publish(msg)
        self.get_logger().info(
            f"✓ Camera registration sent → {REG_TOPIC}",
            throttle_duration_sec=30.0
        )

    # ── Resolution cache ──────────────────────────────────────────────────────

    def _update_resolution_cache(self, pc_msg: PointCloud2, img_msg: Image):
        n_pts = pc_msg.height * pc_msg.width
        if n_pts == self._cached_n_pts:
            return

        out_h, out_w, layout = _resolve_output_dims(pc_msg, img_msg)

        if self._cached_n_pts is None:
            self.get_logger().info(
                f"Resolution detected: PC {pc_msg.width}×{pc_msg.height} "
                f"| Image {img_msg.width}×{img_msg.height} "
                f"→ output {out_w}×{out_h}  layout={layout}"
            )

        self._cached_n_pts  = n_pts
        self._cached_out_h  = out_h
        self._cached_out_w  = out_w
        self._cached_layout = layout

    # ── Synchronized PC + RGB callback ───────────────────────────────────────

    def _synced_cb(self, pc_msg: PointCloud2, img_msg: Image):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now < self._next_publish_time:
            return
        self._next_publish_time += PUBLISH_PERIOD
        if self._next_publish_time < now:
            self._next_publish_time = now + PUBLISH_PERIOD

        self._update_resolution_cache(pc_msg, img_msg)

        try:
            # Generate shared timestamp for both outputs
            stamp = self.get_clock().now().to_msg()

            # 1. Convert and publish PointCloud2
            out_pc = self._convert(pc_msg, img_msg, stamp)
            self.pc_pub.publish(out_pc)

            # 2. Publish paired CameraInfo with identical timestamp
            self._publish_synced_camera_info(stamp)

        except Exception as e:
            self.get_logger().error(
                f"Conversion failed: {e}",
                throttle_duration_sec=2.0,
            )

    def _publish_synced_camera_info(self, stamp):
        """Publishes CameraInfo using the exact timestamp of the PointCloud2."""
        if self._latest_ci_msg is not None:
            ci_msg = copy.deepcopy(self._latest_ci_msg)
        else:
            # Construct minimal fallback CameraInfo if input topic hasn't published yet
            ci_msg = CameraInfo()
            ci_msg.height = self._cached_out_h or 480
            ci_msg.width = self._cached_out_w or 640
            cx, cy = ci_msg.width / 2.0, ci_msg.height / 2.0
            ci_msg.distortion_model = "plumb_bob"
            ci_msg.k = [500.0, 0.0, cx, 0.0, 500.0, cy, 0.0, 0.0, 1.0]

        ci_msg.header.stamp = stamp
        ci_msg.header.frame_id = OUTPUT_FRAME_ID
        self.ci_pub.publish(ci_msg)

    def _convert(self, pc_msg: PointCloud2, img_msg: Image, stamp) -> PointCloud2:
        n_pts      = pc_msg.height * pc_msg.width
        point_step = pc_msg.point_step
        raw        = np.frombuffer(bytes(pc_msg.data), dtype=np.uint8).reshape(n_pts, point_step)

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

        rgb_array    = image_to_rgb_array(img_msg)
        rgb_as_float = pack_rgb_pcl(rgb_array)
        rgb_as_float[invalid] = np.uint32(0).view(np.float32)

        out_h = self._cached_out_h
        out_w = self._cached_out_w

        out = np.empty((out_h * out_w, 4), dtype=np.float32)
        out[:, 0] = x
        out[:, 1] = y
        out[:, 2] = z
        out[:, 3] = rgb_as_float

        out_msg = PointCloud2()
        out_msg.header.stamp    = stamp
        out_msg.header.frame_id = OUTPUT_FRAME_ID
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
    node = ZividToSimulatedCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
