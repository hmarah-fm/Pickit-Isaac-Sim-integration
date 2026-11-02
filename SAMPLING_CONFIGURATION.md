# Simulated Camera Point Cloud Sampling & Quality Tuning Guide

This guide documents the parameterized sampling architecture for [`zivid_to_simulated_camera`](.//docker/pcd_converter/mounted_folder/pcd_converter_ws/src/pcd_converter/pcd_converter/zivid_to_simulated_camera.py) within the **Pickit & Isaac Sim Integration**, enabling real-time experimentation with point cloud resolution, bandwidth consumption, and detection accuracy.

---

## 1. Resolution & Quality Matrix

The native raw simulation resolution from Isaac Sim is **1024 &times; 640** (655,360 points).  
Decimation downsampling selects every $D$-th pixel across rows and columns ($O(1)$ striding), preserving the organized grid layout and point-to-pixel correspondence.

| `downsample_factor` | Output Cloud Resolution | Total Points | Payload Size / Frame | Zenoh & Pickit Status | Best For |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **`4`** | **256 &times; 160** | 40,960 | **655 KB** | Verified (1.32s detect) | Maximum speed & minimal DDS link load |
| **`3`** *(Recommended)* | **342 &times; 214** | 73,188 | **1.17 MB** | Verified (3.04s detect, 5 objects found) | **Optimal balance of detail and stability** |
| **`2`** | 512 &times; 320 | 163,840 | 2.62 MB | High DDS buffer load; causes `ros1_bridge` restart | High-resolution offline inspection |
| **`1`** | 1024 &times; 640 | 655,360 | 10.48 MB | Exceeds real-time DDS packet limits | Native raw resolution capture only |

> [!TIP]
> **Factor `3` (342 &times; 214, 1.17 MB)** provides fine edge definition for complex workpieces (like gears and screws) while maintaining consistent delivery across the Zenoh TCP bridge to the Pickit processor without triggering DDS buffer timeouts.

---

## 2. Dynamic Live Tuning (No Restart Required)

The node exposes `downsample_factor` and `publish_hz` as dynamic parameters with automatic `CameraInfo` dimension scaling. You can change them live from any ROS 2 terminal on domain 42:

### 2.1 Adjusting Spatial Resolution (`downsample_factor`)

Execute from `robot_container` or host:

```bash
# Factor 3 (342x214, 1.17 MB - Recommended Quality):
docker exec robot_container bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 param set /zivid_to_simulated_camera downsample_factor 3'

# Factor 4 (256x160, 655 KB - Lightweight Baseline):
docker exec robot_container bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 param set /zivid_to_simulated_camera downsample_factor 4'

# Factor 2 (512x320, 2.62 MB):
docker exec robot_container bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 param set /zivid_to_simulated_camera downsample_factor 2'

# Factor 1 (1024x640, 10.48 MB - Full Unfiltered):
docker exec robot_container bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 param set /zivid_to_simulated_camera downsample_factor 1'
```

### 2.2 Adjusting Publication Rate (`publish_hz`)

```bash
# Set publication rate to 0.5 Hz (1 frame every 2.0s):
docker exec robot_container bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 param set /zivid_to_simulated_camera publish_hz 0.5'

# Set publication rate to 1.0 Hz (1 frame every 1.0s):
docker exec robot_container bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 param set /zivid_to_simulated_camera publish_hz 1.0'
```

### 2.3 Inspecting Current Parameters

```bash
# List all parameters:
docker exec robot_container bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 param list /zivid_to_simulated_camera'

# Get current downsample factor:
docker exec robot_container bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 param get /zivid_to_simulated_camera downsample_factor'
```

---

## 3. Persistent Configuration via Docker Compose

To set the startup defaults permanently, configure the environment variables in `compose.yaml` under `ros2_pcd_publisher`:

```yaml
  ros2_pcd_publisher:
    image: gearbox-cell51-ros2_pcd_publisher
    container_name: ros2_pcd_publisher
    environment:
      <<: *ros2-fastdds-env
      ROS_DISTRO: jazzy
      WORKSPACE_DIR: /mounted_folder/pcd_converter_ws
      PACKAGE_NAME: pcd_converter
      LAUNCH_FILE: zivid_pcd_launch.launch.py
      DOWNSAMPLE_FACTOR: 3     # Default sampling factor (1, 2, 3, 4, ...)
      PUBLISH_HZ: 0.5          # Default publication frequency (Hz)
```

Alternatively, set them in your local `.env` file:

```env
DOWNSAMPLE_FACTOR=3
PUBLISH_HZ=0.5
```

---

## 4. Verification & Diagnostics

### 4.1 Confirming Output Cloud Resolution

Check the live container logs to confirm the updated dimensions and payload size:

```bash
docker logs --tail 10 ros2_pcd_publisher
```

Expected output when setting `downsample_factor` to `3`:

```
[zivid_to_simulated_camera-1] [INFO]: Parameter 'downsample_factor' updated: 4 -> 3 (target resolution: 342x214, 1171008 bytes per frame)
[zivid_to_simulated_camera-1] [INFO]: Resolution detected: PC 655360×1 | Image 1024×640 → raw 1024×640 (output factor 3 → 342×214) layout=flat→organized
[zivid_to_simulated_camera-1] [INFO]: Published synced cloud (342x214, 1171008 bytes) + CameraInfo
```

### 4.2 Verifying Pickit Image Capture

Trigger a service capture from `robot_container` to ensure the Pickit vision engine ingests the new resolution with zero dropouts:

```bash
docker exec robot_container bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 service call /pickit/capture_image pickit_msgs/srv/CaptureImage "{}"'
```

Expected response:

```yaml
response:
  pickit_msgs.srv.CaptureImage_Response(status=0) # 0 = PICKIT_IMAGE_CAPTURED
```
