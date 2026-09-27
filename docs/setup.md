# Setup and run

[Project overview](../README.md) · [Detailed operations](operations.md)

## Environment

Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic. A functioning graphical/GPU environment is needed for headed Gazebo and simulated cameras. The screenshot run used an existing installation; the complete dependency installation below was not repeated on a clean operating system.

Install ROS following the [official Jazzy instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html). See [Gazebo ROS installation](https://gazebosim.org/docs/harmonic/ros_installation/).

```bash
sudo apt update
sudo apt install -y git cmake g++ python3-colcon-common-extensions python3-rosdep   ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers   ros-jazzy-xacro python3-numpy python3-opencv python3-yaml

git clone https://github.com/VIGIL-MULTI-PURPOSE-ROBOT/vigil-multi-purpose-ugv.git
cd vigil-multi-purpose-ugv
source /opt/ros/jazzy/setup.bash
# Run sudo rosdep init once only if rosdep has not been initialized.
rosdep update
rosdep install --from-paths ros2_ws/src vigil_rough_terrain_ws/src military_world/sar_ws/src   --ignore-src -r -y --rosdistro jazzy
```

## Recommended first demonstration: rough terrain

From the repository root, in a fresh terminal:

```bash
cd vigil_rough_terrain_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select vigil_rough_terrain --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
export ROS_DOMAIN_ID=71
ros2 launch vigil_rough_terrain vision_nav.launch.py
```

Open **http://localhost:8080**. Expect a camera overlay, terrain map, path and navigation status. The default short mission starts automatically. Ctrl+C stops the launch. Do not run another environment on the same dashboard port at the same time.

For the cliff test, stop the first launch, then run in the same sourced terminal:

```bash
ros2 launch vigil_rough_terrain vision_nav.launch.py scenario:=cliff_front
```

To lower display-camera load:

```bash
ros2 launch vigil_rough_terrain vision_nav.launch.py camera_width:=960 camera_height:=720 camera_rate:=5
```

The depth stream remains at its original settings. `gui:=false` hides the Gazebo GUI, but camera rendering still needs a supported rendering environment. It is not a guarantee of operation without GPU/display support.

The launch normally cleans up stale processes belonging to VIGIL partitions. Use `cleanup:=false` when managing isolated runs yourself; the capture session did this with a dedicated ROS domain and Gazebo partition.

## Agriculture

In a fresh terminal from the repository root:

```bash
cd ros2_ws
./run.sh row_mission:=true row_count:=3
```

The script builds the workspace. Open **http://localhost:8080** and use the dashboard's **START ROBOT** control. A three-row request is a mission configuration, not a guarantee that all three rows finish. Full-row acceptance remains pending. See [acceptance](../ros2_ws/ACCEPTANCE.md).

## Search and rescue

See the preserved [build/run instructions](operations.md#4-build) for the rover workspace and separate walking-people plugin. SAR has a larger world and heavier runtime requirements. This documentation capture did not rerun the full SAR mission.

## If startup fails

- Open a fresh terminal and source only the intended workspace.
- Check that ROS dependencies and wheel controllers are available.
- Confirm that port 8080 is free before starting the next dashboard.
- Check rendering access if camera images are blank.
- Do not treat a visible world as proof that controllers or navigation started; inspect status and sensor output.
- Historical package guides contain author-specific absolute paths. Use the clone-relative commands in this guide.
