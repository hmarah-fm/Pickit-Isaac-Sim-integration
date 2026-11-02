#!/usr/bin/env bash
# Entrypoint for the bind-mounted pcd_converter ROS 2 workspace.
#
# Workspace source order:
#   1. /opt/ros/<distro>          ROS 2 binary underlay
#   2. /ros2_ws                   Image-baked custom packages, e.g. pickit_camera_msgs
#   3. /mounted_folder/...        Bind-mounted development overlay, pcd_converter
#
# The script rebuilds pcd_converter only when it is not discoverable, then
# launches zivid_pcd_launch.launch.py.

set -Eeo pipefail

source_ros_setup() {
  local setup_file="$1"

  [[ -f "${setup_file}" ]] || \
    error "ROS setup file does not exist: ${setup_file}"

  # Generated ROS/Ament setup scripts may expand optional variables that are
  # unset. Disable nounset only while evaluating those external scripts.
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

error() {
  echo "[run_zivid_pcd] ERROR: $*" >&2
  exit 1
}

IMAGE_WORKSPACE_DIR="${IMAGE_WORKSPACE_DIR:-/ros2_ws}"
WORKSPACE_DIR="${WORKSPACE_DIR:-/mounted_folder/pcd_converter_ws}"
PACKAGE_NAME="${PACKAGE_NAME:-pcd_converter}"
LAUNCH_FILE="${LAUNCH_FILE:-zivid_pcd_launch.launch.py}"
REQUIRED_MESSAGE_PACKAGE="${REQUIRED_MESSAGE_PACKAGE:-pickit_camera_msgs}"
REQUIRED_EXECUTABLE="${REQUIRED_EXECUTABLE:-zivid_to_simulated_camera}"

[[ -d "${WORKSPACE_DIR}/src" ]] || \
  error "Expected development workspace at '${WORKSPACE_DIR}' with a src/ directory."

[[ -f "${WORKSPACE_DIR}/src/${PACKAGE_NAME}/package.xml" ]] || \
  error "Package manifest not found: '${WORKSPACE_DIR}/src/${PACKAGE_NAME}/package.xml'."

# ---------------------------------------------------------------------------
# 1. Source the ROS distribution underlay.
# ---------------------------------------------------------------------------
if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "[run_zivid_pcd] Sourcing ROS distribution: /opt/ros/${ROS_DISTRO}"
  source_ros_setup "/opt/ros/${ROS_DISTRO}/setup.bash"
else
  ros_setups=(/opt/ros/*/setup.bash)

  [[ -e "${ros_setups[0]}" ]] || \
    error "No ROS 2 installation found below /opt/ros."

  [[ "${#ros_setups[@]}" -eq 1 ]] || \
    error "Multiple ROS installations found below /opt/ros. Set ROS_DISTRO explicitly."

  echo "[run_zivid_pcd] Sourcing ROS distribution: ${ros_setups[0]}"
  source_ros_setup "${ros_setups[0]}"
fi

# ---------------------------------------------------------------------------
# 2. Source project-specific packages baked into the Docker image.
#
# This must happen before sourcing the mounted pcd_converter workspace because
# pcd_converter imports pickit_camera_msgs.msg.CameraMetaInfo at runtime.
# ---------------------------------------------------------------------------
[[ -f "${IMAGE_WORKSPACE_DIR}/install/setup.bash" ]] || \
  error "Image workspace setup not found: '${IMAGE_WORKSPACE_DIR}/install/setup.bash'. Ensure the image builds /ros2_ws and contains '${REQUIRED_MESSAGE_PACKAGE}'."

echo "[run_zivid_pcd] Sourcing image workspace: ${IMAGE_WORKSPACE_DIR}"
source_ros_setup "${IMAGE_WORKSPACE_DIR}/install/setup.bash"

ros2 pkg prefix "${REQUIRED_MESSAGE_PACKAGE}" >/dev/null 2>&1 || \
  error "Required ROS package '${REQUIRED_MESSAGE_PACKAGE}' is not discoverable after sourcing '${IMAGE_WORKSPACE_DIR}/install/setup.bash'. Ensure it exists under the Docker build input ros2_src/ and rebuild the image."

# ---------------------------------------------------------------------------
# 3. Source the bind-mounted development overlay if it already exists.
# ---------------------------------------------------------------------------
if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "[run_zivid_pcd] Sourcing development workspace: ${WORKSPACE_DIR}"
  source_ros_setup "${WORKSPACE_DIR}/install/setup.bash"
fi

# Build only if the pcd_converter package is not already available.
if ! ros2 pkg prefix "${PACKAGE_NAME}" >/dev/null 2>&1; then
  echo "[run_zivid_pcd] Package '${PACKAGE_NAME}' is not discoverable; building workspace..."

  cd "${WORKSPACE_DIR}"

  colcon build \
    --packages-select "${PACKAGE_NAME}" \
    --symlink-install \
    --event-handlers console_direct+

  [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]] || \
    error "Build completed but '${WORKSPACE_DIR}/install/setup.bash' does not exist."

  echo "[run_zivid_pcd] Sourcing newly built development workspace: ${WORKSPACE_DIR}"
  source_ros_setup "${WORKSPACE_DIR}/install/setup.bash"
fi

# All generated setup scripts have been sourced. Enable strict unset-variable
# checking for the rest of this entrypoint.
set -u

# ---------------------------------------------------------------------------
# Preflight validation.
# ---------------------------------------------------------------------------
ros2 pkg prefix "${REQUIRED_MESSAGE_PACKAGE}" >/dev/null 2>&1 || \
  error "Required package '${REQUIRED_MESSAGE_PACKAGE}' is not discoverable after all workspace setup files were sourced."

ros2 pkg prefix "${PACKAGE_NAME}" >/dev/null 2>&1 || \
  error "Package '${PACKAGE_NAME}' is not discoverable after sourcing/building '${WORKSPACE_DIR}'."

if ! ros2 pkg executables "${PACKAGE_NAME}" | awk '{print $2}' | \
  grep -Fxq "${REQUIRED_EXECUTABLE}"; then
  error "Executable '${PACKAGE_NAME}/${REQUIRED_EXECUTABLE}' is not installed. Check setup.py entry_points and setup.cfg."
fi

PACKAGE_PREFIX="$(ros2 pkg prefix "${PACKAGE_NAME}")"
PACKAGE_SHARE="${PACKAGE_PREFIX}/share/${PACKAGE_NAME}"
INSTALLED_LAUNCH_FILE="${PACKAGE_SHARE}/launch/${LAUNCH_FILE}"

[[ -f "${INSTALLED_LAUNCH_FILE}" ]] || \
  error "Launch file not installed: '${INSTALLED_LAUNCH_FILE}'. Check the filename and setup.py data_files."

echo "[run_zivid_pcd] ROS distribution: ${ROS_DISTRO:-unknown}"
echo "[run_zivid_pcd] Message-package prefix: $(ros2 pkg prefix "${REQUIRED_MESSAGE_PACKAGE}")"
echo "[run_zivid_pcd] Bridge-package prefix: ${PACKAGE_PREFIX}"
echo "[run_zivid_pcd] Launching ${PACKAGE_NAME}/${LAUNCH_FILE}"

# Intentionally do not forward "$@" here. A legacy Docker CMD may contain
# Bash '-c ...' arguments, which are not valid ros2 launch arguments.
exec ros2 launch "${PACKAGE_NAME}" "${LAUNCH_FILE}"
