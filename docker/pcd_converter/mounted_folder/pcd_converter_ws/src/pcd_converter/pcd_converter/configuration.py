"""Startup defaults shared by the converter and its launch file."""

import math
import os


DEFAULT_PARAMETERS = {
    "input_pointcloud_topic": os.environ.get("INPUT_POINTCLOUD_TOPIC", "/ZividCamera/pointcloud"),
    "input_rgb_topic": os.environ.get("INPUT_RGB_TOPIC", "/ZividCamera/rgb"),
    "input_camera_info_topic": os.environ.get("INPUT_CAMERA_INFO_TOPIC", "/ZividCamera/camera_info"),
    "output_pointcloud_topic": os.environ.get("OUTPUT_POINTCLOUD_TOPIC", "/simulated_camera/depth_registered/points_3d_rectified"),
    "output_camera_info_topic": os.environ.get("OUTPUT_CAMERA_INFO_TOPIC", "/simulated_camera/depth_registered/camera_info"),
    "registration_topic": os.environ.get("REGISTRATION_TOPIC", "/pickit/camera_registrations"),
    "output_frame_id": os.environ.get("OUTPUT_FRAME_ID", "simulated_camera/simulated_camera_rgb_optical_frame"),
    "camera_serial": os.environ.get("CAMERA_SERIAL", "1-simulated_camera"),
    "camera_namespace": os.environ.get("CAMERA_NAMESPACE", "/simulated_camera"),
    "camera_type": int(os.environ.get("CAMERA_TYPE", 1)),
    "camera_source": int(os.environ.get("CAMERA_SOURCE", 1)),
    "rgb_optical_frame": os.environ.get("RGB_OPTICAL_FRAME", "simulated_camera/simulated_camera_rgb_optical_frame"),
    "root_frame": os.environ.get("ROOT_FRAME", "simulated_camera/simulated_camera_link"),
    "heartbeat_timeout_seconds": float(os.environ.get("HEARTBEAT_TIMEOUT_SECONDS", -1.0)),
    "publish_hz": float(os.environ.get("PUBLISH_HZ", 0.5)),
    "downsample_factor": int(os.environ.get("DOWNSAMPLE_FACTOR", 4)),
    "sync_slop_seconds": float(os.environ.get("SYNC_SLOP_SECONDS", 0.2)),
    "sync_queue_size": int(os.environ.get("SYNC_QUEUE_SIZE", 10)),
    "registration_delay_seconds": float(os.environ.get("REGISTRATION_DELAY_SECONDS", 2.0)),
    "registration_publish_hz": float(os.environ.get("REGISTRATION_PUBLISH_HZ", 0.2)),
    "use_sim_time": os.environ.get("USE_SIM_TIME", "false").lower() in ("true", "1", "yes"),
}

PARAMETER_DESCRIPTIONS = {
    "downsample_factor": "Decimation step for spatial downsampling (1=full 1024x640, 2=512x320, 3=342x214, 4=256x160, etc.)",
    "publish_hz": "Maximum publication rate in Hz for point cloud and camera info",
    "sync_slop_seconds": "Maximum timestamp difference between pointcloud and RGB image in seconds",
    "sync_queue_size": "Queue size for message synchronizer",
    "input_pointcloud_topic": "Topic name for input point cloud from simulator",
    "input_rgb_topic": "Topic name for input RGB image from simulator",
    "input_camera_info_topic": "Topic name for input CameraInfo calibration from simulator",
    "output_pointcloud_topic": "Topic name for output organized XYZRGB PointCloud2",
    "output_camera_info_topic": "Topic name for output scaled CameraInfo",
    "registration_topic": "Topic name for Pickit CameraMetaInfo registrations",
    "output_frame_id": "Frame ID to stamp on output point clouds and camera info",
    "camera_serial": "Serial identifier for Pickit registration",
    "camera_namespace": "Camera namespace for Pickit registration",
    "camera_type": "Camera type integer for Pickit registration",
    "camera_source": "Camera source integer for Pickit registration",
    "rgb_optical_frame": "Optical frame name in camera metadata",
    "root_frame": "Root frame name in camera metadata",
    "heartbeat_timeout_seconds": "Heartbeat timeout in seconds (-1 to disable)",
    "registration_delay_seconds": "Delay before first registration publication in seconds",
    "registration_publish_hz": "Periodic publication rate of camera registration",
    "use_sim_time": "Whether to use ROS simulation time",
}

READONLY_PARAMETERS = {
    "input_pointcloud_topic",
    "input_rgb_topic",
    "input_camera_info_topic",
    "output_pointcloud_topic",
    "output_camera_info_topic",
    "registration_topic",
    "output_frame_id",
    "camera_serial",
    "camera_namespace",
    "camera_type",
    "camera_source",
    "rgb_optical_frame",
    "root_frame",
    "heartbeat_timeout_seconds",
    "registration_delay_seconds",
    "registration_publish_hz",
    "use_sim_time",
}

DYNAMIC_PARAMETERS = {
    "downsample_factor",
    "publish_hz",
    "sync_slop_seconds",
    "sync_queue_size",
}


def validate_parameters(values):
    for name in ("publish_hz", "registration_publish_hz", "sync_slop_seconds"):
        if not math.isfinite(values[name]) or values[name] <= 0:
            raise ValueError(f"{name} must be finite and greater than zero")
    if values.get("downsample_factor", 1) < 1:
        raise ValueError("downsample_factor must be an integer >= 1")
    for name in ("registration_delay_seconds",):
        if not math.isfinite(values[name]) or values[name] < 0:
            raise ValueError(f"{name} must be finite and non-negative")
    if values["sync_queue_size"] < 1:
        raise ValueError("sync_queue_size must be greater than zero")
    if not math.isfinite(values["heartbeat_timeout_seconds"]):
        raise ValueError("heartbeat_timeout_seconds must be finite")
    for name, maximum in (("camera_type", 2**32 - 1), ("camera_source", 255)):
        if not 0 <= values[name] <= maximum:
            raise ValueError(f"{name} must be between zero and {maximum}")
    if not isinstance(values.get("use_sim_time", True), bool):
        raise ValueError("use_sim_time must be a boolean")
    for name, default in DEFAULT_PARAMETERS.items():
        if isinstance(default, str) and not values[name].strip():
            raise ValueError(f"{name} must not be empty")
