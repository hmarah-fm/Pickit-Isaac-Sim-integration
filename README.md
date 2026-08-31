# Pickit Isaac-Sim Interface

Isaac Sim simulation and Pickit integration workflow. Supports both native and dockerised Isaac Sim deployment.

## What it provides

- Isaac Sim 5.1 simulation with Isaac's internal Jazzy ROS 2 bridge
- Native Jazzy point cloud conversion node for Pickit
- Zenoh bridge for cross-network bridging between ROS 2 topic space and Pickit host interfaces..
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

export ROS_DOMAIN_ID=28                     # same domain id as .env
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
ROS_DOMAIN_ID=28            # change to value set in your .env. See section 2.
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
