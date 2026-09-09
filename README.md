# Agricultural UGV simulation

ROS 2 Jazzy / Gazebo Harmonic project for an eight-wheel rover in the supplied
cotton farm. The complete source and simulation assets are under `ros2_ws/src`.

```text
ros2_ws/
├── src/
│   ├── agri_ugv/              # Field, assets, autonomy, controllers, launch files
│   ├── agri_ugv_description/  # CAD meshes, Xacro, robot and sensor configuration
│   └── agri_ugv_setup/        # Environment checks
├── tools/                    # Diagnostics and presentation controls
├── tests/                    # Map-search and suspension-math tests
├── reports/                  # Historical simulation evidence
├── run.sh                    # Build and start Gazebo + RViz
├── stop.sh                   # Stop the workspace simulation
└── demo.sh                   # Controls for the optional frontier demo
```

From the repository root, with Jazzy and the project dependencies installed:

```bash
cd ros2_ws
./run.sh row_mission:=true row_count:=23
```

Use `row_count:=3` for a shorter configured sweep. Stop with Ctrl-C or, in another
terminal inside `ros2_ws`, `./stop.sh`. The launcher builds before starting and
sources the workspace automatically. For no windows:

```bash
./run.sh headless:=true rviz:=false row_mission:=true row_count:=3
```

Build and check without starting Gazebo:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install
source install/setup.bash
python3 -m pytest -q tests
python3 tools/check_structure.py
```

The workspace was relocated from `agri_ugv_ws` to `ros2_ws`. Use a fresh terminal
instead of sourcing an old install directory. Local maps, logs and the previous
build/install backup remain on disk and are excluded from Git.

Current status: this is a simulation prototype. Row missions use Gazebo ground
truth; EKF and RTAB-Map run separately. Full 23-row completion, crop clearance and
tractor avoidance have **not** passed an end-to-end acceptance run. The current
tractor guard is a fixed-coordinate stop rule, not a verified detour planner.
See [workspace instructions](ros2_ws/README.md),
[presentation commands](ros2_ws/HACKATHON.md) and
[acceptance history](ros2_ws/ACCEPTANCE.md) before describing results.
