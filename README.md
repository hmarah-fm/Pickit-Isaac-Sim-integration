# Pickit Isaac-Sim Interface

Isaac Sim simulation and Pickit integration workflow. Supports both native and dockerised Isaac Sim deployment.

## What it provides

- Isaac Sim 5.1 simulation with Isaac's internal Jazzy ROS 2 bridge
- Native Jazzy point cloud conversion node for Pickit with runtime parameterized sampling (see [SAMPLING_CONFIGURATION.md](SAMPLING_CONFIGURATION.md))
- Zenoh bridge for cross-network bridging between ROS 2 topic space and Pickit host interfaces
- Shared Fast DDS configuration and isolated ROS domain for ROS 2 participants

## Architecture
The Docker Compose dependency order is process startup order only. It does not prove that Isaac is playing, topics are publishing, controllers are active, or MoveIt is ready to accept actions.

The intended startup order is:

1. isaac-sim service publishes `/clock`, `/joint_states`, `/joint_command`, and camera topics.
2. pcd-converter service publishes `/simulated_camera/depth_registered/*`.
3. zenoh-bridge service transmits the `/simulated_camera/depth_registered/*` via tcp to Pickit processor


## Repository layout

```text
.
# Below is a high-level summary of the repository layout; auxiliary files have been hidden for the sake of brevity
├── compose.yaml
├── config
│   └── fastdds.xml			    # Fast-DDS configuration
├── docker
│   ├── isaacsim51
│   │   ├── Dockerfile			# Ubuntu 24.04 Image with Isaac Sim and ROS2 Jazzy natively installed
│   │   └── launch-isaac-jazzy.sh	# Headless Isaac Sim Launcher
│   ├── pcd_converter
│   │   └── mounted_folder
│   │       ├── pcd_converter_ws	# This workspace contains the pcd_converter pkg required by pickit-sim-camera-bridge
│   │       └── run_zivid_pcd.sh	# Auto builds the workspace and runs the converter node [ZividCamera -> SimulatedCamera]
│   └── pickit-sim-camera-bridge	# Submodule: https://github.com/Pickit3D/pickit-sim-camera-bridge ; zenoh-bridge for Pickit
├── isaacsim_workspace	      # Submodule: https://github.com/ifu-ev/robot_workcell_usd
│   ├── README.md			  # Refer to README for configuring ROS2 Omnigraph to the example USD file
│   ├── scenes				
│   │   └── gearbox_disassembly.usda	        # Example USD file
│   ├── scripts
│   │   └── add_pickit_and_ros2_omnigraph.py	# Standalone script for adding Omnigraph
└── README.md
```

## Requirements

- Linux host supported by Isaac Sim 5.1
- NVIDIA GPU and a compatible NVIDIA driver
- Docker Engine and Docker Compose v2
- NVIDIA Container Toolkit
- Access to `nvcr.io/nvidia/isaac-sim:5.1.0`
- Git
- X11 session when running RViz from the MoveIt container
- \>100Gb Disk capacity for Isaac images, cache, logs, assets, and ROS workspaces

## Prerequisits

- Install Docker Engine (with Docker Compose) and NVIDIA container Toolkit following their official installation guide.

Verify Docker and GPU support:
```bash
nvidia-smi
docker --version
docker compose version
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
```

## Quick start

### 1. Clone the Jazzy branch

```bash
git clone --recurse-submodules https://github.com/Pickit3D/Pickit-Isaac-Sim-integration.git Pickit-Isaac-Sim-integration
cd Pickit-Isaac-Sim-integration
git branch --show-current
```

If you cloned the repository without the submodules flag, then the submodule directories will remain empty. Run the following to fetch those directories:
```bash
cd Pickit-Isaac-Sim-integration
git submodule update --init --recursive
```

Use the branch intended for Isaac Sim 5.1 and Jazzy.

### 2. Configure `.env`

Create `.env` from the project example when available:

```bash
cp .env.example .env
```

Set deployment-specific values:

```dotenv
# ROS 2: Set the domain id to isolate the Isaac Sim's ros2 topics; avoid setting to default 0
# Source the same .env if an external node also needs to access the Isaac Sim topics
ROS_DOMAIN_ID=28    # Set any suitable value

# IP address of Pickit processor; ping -c 4 $PICKIT_ROS2_IP to confirm IP is reachable
PICKIT_ROS2_IP=192.168.1.100

# Persistent Isaac Sim cache, logs, configuration, and data.
ISAAC_DATA_DIR=/home/user/storage/isaac-sim
ISAAC_CACHE_DIR=/home/user/storage/isaac-sim/cache
```

Replace all example IP addresses and paths with values for the target environment.

### 3. Create Isaac persistent storage

Note: Make sure cache and data directories contain atleast 10Gb of free storage
```bash
source .env

mkdir -p \
  "$ISAAC_CACHE_DIR/main" \
  "$ISAAC_CACHE_DIR/computecache" \
  "$ISAAC_DATA_DIR/logs" \
  "$ISAAC_DATA_DIR/config" \
  "$ISAAC_DATA_DIR/data" \
  "$ISAAC_DATA_DIR/pkg"
```

The Isaac service runs as root in the current Compose configuration, so these directories must be writable by Docker. If a future Compose version uses an unprivileged Isaac user, adjust ownership to that configured UID/GID.

### 4. Validate and build images

```bash
source .env
docker compose config             # verify config, add -q to suppress output
docker compose build --no-cache   # builds everything including isaac sim image
# remove --no-cache for subsequent rebuilds
```
For individual build:
```bash
source .env
docker compose config
docker compose build --no-cache isaac-sim   # skip if isaac sim is installed natively on workstation
docker compose build --no-cache ros2_pcd_publisher
docker compose build --no-cache zenoh-bridge
```

Refer to [section 7](#7-start-services) before running the container.

### 6. Enable display access within the containers

**Following section is optional**

If you wish to enable display access for the container, on the host desktop session: 

```bash
xhost +si:localuser:root
```

Confirm a display is available:

```bash
echo "$DISPLAY"
```

When finished with RViz, to revoke this permission:

```bash
xhost -si:localuser:root
```

### 7. Start services

**Dockerised Isaac Sim build**:

Edit the `ROS_AUTOMATIC_DISCOVERY_RANGE` in the compose.yaml file to `LOCALHOST`. This restricts the dds to localhost (Only to nodes running on the docker containers). If you need to access the nodes/topics from host pc, set this to `SUBNET`

Start Isaac first and confirm the scene is playing and publishing ROS topics:

```bash
source .env
docker compose up -d isaac-sim --no-deps --force-recreate # force-recreate: rebuild container if image, code or config has changed
docker compose logs -f isaac-sim
```

Note: For dockerised Isaac-sim, download the IsaacSim WebRTC streaming client from the [official website](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/download.html) 

**Native Isaac Sim**:

Edit the `ROS_AUTOMATIC_DISCOVERY_RANGE` in the compose.yaml file to `SUBNET`. 

Source the appropriate .env file before launching IsaacSim
```text
# source your native ros2 jazzy installation first

export ROS_DOMAIN_ID=42                  # same domain id as .env
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/location_to_the_fastdds_config/fastdds.xml
export FASTDDS_DEFAULT_PROFILES_FILE=/home/user/location_to_the_fastdds_config/fastdds.xml

```

Navigate to the `isaacsim_workspace/scenes` folder and open the example scene `gearbox_disassembly.usda`. 

The default scene doesn't run the necessary ros2 nodes. Follow the instructions mentioned in `isaacsim_workspace/README.md` and run the `isaacsim_workspace/scripts/add_pickit_and_ros2_omnigraph.py` to add ROS2 Omnigraphs to the scene. With the omnigraphs added, play the simulation to stream the point cloud data to ros2 topics.

Then start the remaining ROS services (if not launched) to connect IsaacSim with Pickit:

```bash
source .env
docker compose up -d ros2_pcd_publisher zenoh-bridge --no-deps --force-recreate
```
Launching ros2_pcd_publisher auto-builds the internal ros2 pcd converter package. If you make any local changes to this package, delete the install folders to trigger auto-build on relaunch. 
```bash
docker compose down ros2_pcd_publisher
cd ./docker/pcd_converter/mounted_folder/pcd_converter_ws # run from repo root
sudo rm -rf build log install # verify before deleting
cd ../../../../   
docker compose up ros2_pcd_publisher --no-deps --force-recreate
```
If the containers crash and gets stuck in an endless restart loop, try configuring the [UDP buffer size](#udp-receive-buffer).

Stop the stack:

```bash
docker compose down
# or docker compose down container-name
```

## ROS and DDS contract

All ROS 2 services participating in the shared local DDS domain must remain aligned on:

Set `ROS_AUTOMATIC_DISCOVERY_RANGE` to `SUBNET` if Isaac Sim is running natively. `LOCALHOST` explicitly restricts all DDS traffic to the local machine loopback interface (lo) used by docker.

Default Profile:
```text
ROS_DOMAIN_ID=42           # change to value set in your .env. See section 2.
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FASTRTPS_DEFAULT_PROFILES_FILE=/config/fastdds.xml
FASTDDS_DEFAULT_PROFILES_FILE=/config/fastdds.xml
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST # set SUBNET vs LOCALHOST appropriately, see section 7.
```

The shared profile is mounted as:
```text
/config/fastdds.xml
```

## Isaac and MoveIt contract

### Isaac scene and topics

The Jazzy/Isaac Sim 5.1 scene is:

```text
isaacsim_workspace/scenes/gearbox_disassembly.usda
```

Isaac uses its internal Jazzy ROS bridge. Do not source arbitrary system ROS installations into the Isaac application process.

Expected simulator endpoints include:

```text
Isaac publishes:
  /clock
  /joint_states
  /ZividCamera/rgb
  /ZividCamera/camera_info
  /ZividCamera/pointcloud
  /tf

Isaac subscribes:
  /joint_command
```

`/joint_states` and `/joint_command` must both use:

```text
sensor_msgs/msg/JointState
```

### Topic-based ros2_control from Moveit2

On the Jazzy branch, the user must build the hardware interface package from source:

[joint_state_topic_hardware_interface](https://github.com/ros-controls/topic_based_hardware_interfaces/tree/main)

The ROS2 Control tags (<Hardware>) in urdf must contain the following to enable topic based control:

```xml
<plugin>joint_state_topic_hardware_interface/JointStateTopicSystem</plugin>
<param name="joint_commands_topic">/joint_command</param>
<param name="joint_states_topic">/joint_states</param>
```

The Humble branch may use its legacy topic-based binary plugin instead. Keep the implementation branch-specific or select it explicitly in shared Xacro with `ROS_DISTRO`.

## Camera transport operations

Run these commands **on the host**, not inside `robot_container`. They manage
only `ros2_pcd_publisher` and `zenoh-bridge`; leave Isaac running and play the
simulation to supply camera data.

### 1. Enter the integration directory

For the Cypress host checkout:

```bash
cd /home/hussein/workspaces/2022-0780-Cypress/workspace/external/Pickit-Isaac-Sim-integration
```

For another checkout, use its integration directory containing `compose.yaml`
and the configured `.env`.

### 2. Build the camera transport images

```bash
docker compose build ros2_pcd_publisher zenoh-bridge
```

This builds the converter image and the patched Zenoh bridge. The initial
Zenoh build may take several minutes.

### 3. Deploy or update

Create or update the converter container from the built image, rebuild its
bind-mounted ROS package, then recreate the bridge:

```bash
docker compose up -d --no-deps --force-recreate ros2_pcd_publisher

docker compose exec ros2_pcd_publisher bash -lc \
  'source /opt/ros/jazzy/setup.bash &&
   source /ros2_ws/install/setup.bash &&
   cd /mounted_folder/pcd_converter_ws &&
   colcon build --packages-select pcd_converter --symlink-install'

docker compose restart ros2_pcd_publisher

docker compose up -d --no-deps --force-recreate zenoh-bridge
```

Use the build and deployment steps after code or image changes. A restart alone
does not install a changed image. The explicit ROS package build also refreshes
an existing bind-mounted workspace.

### 4. Normal start

After deployment:

```bash
docker compose up -d --no-deps ros2_pcd_publisher zenoh-bridge
```

### 5. Stop camera transport only

```bash
docker compose stop zenoh-bridge ros2_pcd_publisher
```

This leaves Isaac and unrelated services running. Avoid `docker compose down`
for camera-only operations: it can stop the wider integration stack.

### 6. Restart, status and logs

```bash
# Restart both camera services.
docker compose restart ros2_pcd_publisher zenoh-bridge

# Check status.
docker compose ps ros2_pcd_publisher zenoh-bridge

# Follow logs. Ctrl+C exits the viewer without stopping services.
docker compose logs -f --tail 50 ros2_pcd_publisher zenoh-bridge
```

Check the image in the [Cypress Pickit web UI](http://192.168.66.1/), or use your
configured Pickit address. For message-count and timestamp-pairing checks, see
[Regression and live-delivery tests](#regression-and-live-delivery-tests).

## Point-cloud pipeline

The intended topic ownership is:

```text
Isaac Sim
  /ZividCamera/pointcloud
       |
       v
PCD converter
  /simulated_camera/depth_registered/points_3d_rectified
```

Only Isaac publishes `/ZividCamera/*`. Only the PCD converter publishes `/simulated_camera/depth_registered/*`. Avoid duplicate publishers for either pipeline stage.

### Duplicate-free Zenoh camera transport

The top-level Compose service builds
`cypress/zenoh-bridge-ros2dds:1.9.0-route-lifetime` instead of running the unpatched
upstream image. [The build](docker/zenoh/Dockerfile) pins upstream commit
`26e66e1f9867b9faf140fde4e38785508024a554` and applies
[a route-lifetime patch](docker/zenoh/route-lifetime.patch).

Zenoh 1.9.0 detaches matching listeners from their publisher and service-client
routes. When a route is removed, an old listener can survive and create another
DDS reader on reconnect. Each reader forwards another copy of the same sample.
See [upstream issue 722](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/issues/722).
The patch owns each listener in its route, undeclares it and waits for in-flight
callbacks before deleting the DDS entities. Restarting alone only clears the
accumulated readers; it does not prevent recurrence.

The bridge additionally limits the point-cloud route to 1 Hz with
`plugins.ros2dds.pub_max_frequencies`. This is a bandwidth safeguard, not a
replacement for the lifetime fix. A roughly 10.5 MB cloud at 1 Hz should use
roughly 10.5 MB/s of payload, not a saturated gigabit link. Independently
downsampling CameraInfo could break timestamp pairing, so it is not rate-limited
in the bridge.

There is a separate large-frame delivery safeguard: top-level
`qos.publication` sets `congestion_control: "block"` **only** for the cloud's
Zenoh key. Pickit's best-effort DDS subscriber otherwise creates a droppable
route, even when `reliable_routes_blocking` is enabled. In the live test, the
unmodified droppable route delivered CameraInfo but zero clouds through the
remote router; the scoped override delivered complete clouds and restored the
web image. It does not change DDS reliability or block unrelated routes. Keep
the 1 Hz caps: blocking is intended to send a complete frame at the bounded
camera rate, not to queue unbounded high-rate clouds on a slow link.

For build, deployment, start, stop and log commands, see
[Camera transport operations](#camera-transport-operations).

Changing the image/build configuration requires `up --force-recreate`, not only
`restart`. Initial compilation needs outbound access to GitHub/crates.io and may
take several minutes; two build jobs limit memory use alongside Isaac.

To roll back, set `zenoh-bridge.image` in the top-level Compose file back to
`eclipse/zenoh-bridge-ros2dds:1.9.0`, remove its `build` block and recreate only
that service with `docker compose up -d --no-deps --force-recreate zenoh-bridge`.
This reintroduces the known duplication
bug; use it for diagnosis only. Keep the patched image available.

### Converter startup parameters

[The shared defaults](docker/pcd_converter/mounted_folder/pcd_converter_ws/src/pcd_converter/pcd_converter/configuration.py)
are declared and consumed by the node and exposed as typed launch arguments.
Parameters are read-only after startup. For example, when launching the installed
package in a sourced ROS environment:

```bash
ros2 launch pcd_converter zivid_pcd_launch.launch.py \
  publish_hz:=1.0 sync_slop_seconds:=0.1 sync_queue_size:=5 \
  registration_delay_seconds:=2.0 registration_publish_hz:=0.2
```

Do not launch a second converter alongside the Compose-managed publisher.
Other parameters configure the three input topics, both output topics,
`registration_topic`, `output_frame_id`, `camera_serial`, `camera_namespace`,
`camera_type`, `camera_source`, `rgb_optical_frame`, `root_frame`, and
`heartbeat_timeout_seconds`. Existing registration identity/source defaults are
preserved. Topic changes must also be reflected in the bridge allowlist and
Pickit camera configuration.

`publish_hz` defaults to **1.0**, using monotonic wall time to prevent catch-up
bursts after delays or simulation-clock jumps. CameraInfo is cached, then
published with the cloud's timestamp and frame. Until input CameraInfo arrives,
the converter warns and withholds output rather than inventing calibration.
The first registration is sent after `registration_delay_seconds`, then repeated
at `registration_publish_hz`. Invalid rates, delays, queue sizes and empty
topics/frames fail startup with an error.

### Regression and live-delivery tests

Converter tests use the actual ROS message classes and node in an isolated domain:

```bash
docker compose exec -e ROS_DOMAIN_ID=42 ros2_pcd_publisher bash -lc \
  'source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash; source /mounted_folder/pcd_converter_ws/install/setup.bash; cd /mounted_folder/pcd_converter_ws/src/pcd_converter; python3 -m unittest discover -s test -v'
```

The transport tests require a Jazzy Python environment with `rclpy`, `std_msgs`
and `eclipse-zenoh==1.9.0`. Install the latter into a dedicated Python virtual
environment with access to the system ROS packages, not into Isaac's Python.
On an Ubuntu/Jazzy host:

```bash
/usr/bin/python3 -m venv --system-site-packages /tmp/pickit-zenoh-tests
source /tmp/pickit-zenoh-tests/bin/activate
python3 -m pip install eclipse-zenoh==1.9.0
source /opt/ros/jazzy/setup.bash
```

Start a disposable test bridge in a separate terminal; do not use production
domain 42:

```bash
docker run --rm --name cypress-zenoh-regression --network host \
  cypress/zenoh-bridge-ros2dds:1.9.0-route-lifetime \
  --domain 42 --listen tcp/127.0.0.1:17447 \
  --no-multicast-scouting --ros-localhost-only
```

Then run from a sourced ROS environment **on the host** (a bridge-networked
container has a different loopback interface):

```bash
ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 python3 tests/test_zenoh_reconnect.py \
  --endpoint tcp/127.0.0.1:17447 --cycles 10
docker stop cypress-zenoh-regression
```

Each cycle restarts a small DDS publisher and its native Zenoh subscriber plus
ROS liveliness token. Acceptance: at least five unique messages per cycle and
exactly one copy each. The same test must fail against a clean upstream 1.9.0
bridge after reconnect, demonstrating that the test exercises the original bug.

With the real simulation playing, test each end of the camera link:

```bash
python3 tests/verify_camera_delivery.py --endpoint tcp/127.0.0.1:7447 --seconds 60
# Replace with the Pickit address configured in .env:
python3 tests/verify_camera_delivery.py --endpoint tcp/192.168.66.1:7447 --seconds 60
```

The live probe checks ROS CDR timestamps, duplicate counts, cloud rate and
cloud/CameraInfo pairing. It announces properly escaped ROS subscriber liveliness
tokens so a Pickit camera reconnect cannot disable its DDS route during the
measurement. By default it requires paired clouds at at least 80% of the expected
1 Hz rate; use `--minimum-clouds` only for an intentionally slower source.
A missing stream or duplicate is a nonzero exit, not a successful empty
measurement. A subscriber connected to the remote router proves
transport to that router, not image rendering or consumption by Pickit's camera
node. Also inspect the Pickit web UI. If the UI remains blank despite paired
remote delivery, inspect Pickit's camera subscriptions/driver separately.

Validated on the Cypress workcell on 2026-10-27 after restarting both camera
services: all 12 converter tests passed; the stock bridge duplicated messages
on reconnect 2, while the patched bridge passed 10 reconnect cycles (200 checked
messages, one copy each). In a final 120-second window, the host received 114
clouds and the Pickit-router client received 115 clouds, with no duplicate
timestamps and approximately 10 MB/s of cloud payload at each endpoint.
Timestamp pairing passed with the permitted measurement-window boundary
differences. Pickit's web UI displayed the gearbox image at 1024 x 640.
This is a finite live test plus a reconnect regression, not an indefinite soak
or a power-cycle test. The robot-container profile bind mount takes effect on
its next recreation; Isaac was not restarted for this repair.

## Host configuration

### UDP receive buffer

Large `PointCloud2` messages may span many UDP datagrams. Increase host socket-buffer ceilings before validating high-volume point cloud traffic:

```bash
sudo sysctl -w net.core.rmem_max=26214400
sudo sysctl -w net.core.rmem_default=26214400
sudo sysctl -w net.core.wmem_max=26214400
```

To persist the values, add to `/etc/sysctl.conf`:

```text
net.core.rmem_max=26214400
net.core.rmem_default=26214400
net.core.wmem_max=26214400
```

Apply and verify:

```bash
sudo sysctl -p
cat /proc/sys/net/core/rmem_max
```

### Docker log rotation

Configure Docker log rotation to avoid unbounded JSON logs. Edit `/etc/docker/daemon.json`:

```json
{
  "runtimes": {
    "nvidia": {
      "args": [],
      "path": "nvidia-container-runtime"
    }
  },
  "log-driver": "json-file",
  "log-opts": {
    "max-size": "10m",
    "max-file": "10"
  }
}
```

Restart Docker after editing the daemon configuration:

```bash
sudo systemctl restart docker
```

Stopping Docker also stops the running containers. Restart the Compose stack afterward (`docker compose up -d`).

## Validation order

Validate in this order:

1. Run `docker compose config -q`.
2. Build the images.
3. Start Isaac Sim alone and ensure the scene is playing.
4. Start the PCD converter and verify its converted point cloud.
5. From `ros2_pcd_publisher` container, verify live Isaac samples of point cloud topic.

Useful topic checks:

```bash
docker compose run --rm --no-deps \
  --entrypoint /bin/bash \
  ros-bridge \
  -lc '
    source /opt/ros/jazzy/setup.bash
    source /ros2_ws/install/setup.bash

    ros2 topic echo /clock --once
    ros2 topic echo /joint_states --once
    ros2 topic info -v /joint_command
    ros2 topic info -v /simulated_camera/depth_registered/points_3d_rectified
  '
```

## Useful commands

```bash
# Service state and logs
docker compose ps
docker compose logs -f
docker compose logs -f isaac-sim
docker compose logs -f ros2_pcd_publisher
docker compose logs -f zenoh-bridge

# Enter selected containers
docker compose exec isaac-sim bash
docker compose exec ros2_pcd_publisher bash
docker compose exec zenoh-bridge bash

```
## License

Third-party assets remain subject to their applicable rights, terms, notices, and redistribution conditions.
