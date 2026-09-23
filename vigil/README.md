# VIGIL master launcher

A menu that starts **one** of three independent ROS 2 / Gazebo projects. Nothing here is part of
any project: no packages, worlds, robots, dashboards or parameters. The projects stay where they are
and build on their own.

```
~/Documents/vigil/
├── vigil                      the `vigil` command (runs the launcher)
├── setup_vigil.sh             one-time: installs `vigil` in ~/.local/bin, makes the shortcut links
├── publish_to_github.sh       uploads the three projects + this launcher to GitHub (see below)
├── COLCON_IGNORE              colcon never treats this folder (or the links) as a workspace
├── master_launcher/
│   └── vigil_launcher.py      the menu
├── military_sar/military_world         -> ~/Documents/military_world            (link, made by setup)
├── agriculture/ros2_ws                 -> ~/Documents/robot/ros2_ws             (link, made by setup)
└── rock_terrain/vigil_rough_terrain_ws -> ~/Documents/robot/vigil_rough_terrain_ws (link, made by setup)
```

| Option | Workspace | Sourced | Launch (the project's own, unchanged) | ROS_DOMAIN_ID |
|---|---|---|---|---|
| 1 Military Search and Rescue | `~/Documents/military_world/sar_ws` | `/opt/ros/jazzy` + that `install/setup.bash` | `ros2 launch vigil_sar sar_mission.launch.py` | 72 |
| 2 Agriculture | `~/Documents/robot/ros2_ws` | `/opt/ros/jazzy` + that `install/setup.bash` | `ros2 launch agri_ugv field.launch.py` (as `./run.sh`, without its rebuild) | 91 |
| 3 Rock Terrain | `~/Documents/robot/vigil_rough_terrain_ws` | `/opt/ros/jazzy` + that `install/setup.bash` | `ros2 launch vigil_rough_terrain vision_nav.launch.py` | 71 |

## Setup (once)

```bash
bash ~/Documents/vigil/setup_vigil.sh
```

## Run

```bash
vigil
# or, without setup:
python3 ~/Documents/vigil/master_launcher/vigil_launcher.py
```

Ctrl+C stops the running project; the launcher stops anything it left behind and shows the menu
again. Before starting a project, processes still running from any of the three are stopped, so
only one world, one robot and one dashboard exist at a time.

## Build (each project on its own; the launcher never builds)

```bash
# Military SAR
cd ~/Documents/military_world/sar_ws && source /opt/ros/jazzy/setup.bash && \
  colcon build --symlink-install --packages-select vigil_sar --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
# Agriculture
cd ~/Documents/robot/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
# Rock Terrain
cd ~/Documents/robot/vigil_rough_terrain_ws && source /opt/ros/jazzy/setup.bash && \
  colcon build --symlink-install --packages-select vigil_rough_terrain --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```
Use a fresh terminal for each build, so one workspace is never built on top of another.

## Publish to GitHub

```bash
bash ~/Documents/vigil/publish_to_github.sh
```
Copies all three projects and this launcher into a separate clone of
`VIGIL-MULTI-PURPOSE-ROBOT/agri-ugv` (`~/Documents/.vigil_publish`), shows what is new, and after
your `y` commits and pushes to `main` with your own GitHub login. Layout in the repo:
`ros2_ws/`, `vigil_rough_terrain_ws/`, `military_world/`, `vigil/`. Build output is never uploaded,
nothing on GitHub is deleted, and the folders on your PC are not changed. Run it again after later
changes; it only pushes what changed.
