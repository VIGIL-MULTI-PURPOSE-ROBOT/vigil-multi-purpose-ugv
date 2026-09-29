# VIGIL master launcher

A menu that starts **one** of the three independent ROS 2 / Gazebo projects. This folder has no
robot code: no packages, worlds, robots, dashboards or parameters. The projects stay where they
are and are built on their own.

```
vigil/
├── vigil                      the `vigil` command (runs the launcher)
├── setup_vigil.sh             one-time: installs `vigil` in ~/.local/bin
├── master_launcher/
│   └── vigil_launcher.py      the menu
├── COLCON_IGNORE              colcon never treats this folder as a workspace
├── publish_to_github.sh       (maintainer) uploads the projects to GitHub
└── REPO_README.md             (maintainer) source of the repository's front-page README.md
```

## Where it looks for the projects

It finds them in either of these layouts:

1. **A clone of the GitHub repository:** the folders next to `vigil/`, i.e. `../Military Search and Rescue/sar_ws`,
   `../Agriculture` and `../Rock Terrain`. Nothing to set up.
2. **The author's PC:** the shortcut links `military_sar/`, `agriculture/` and `rock_terrain/`
   that `setup_vigil.sh` creates, which point to `~/Documents/military_world`,
   `~/Documents/robot/ros2_ws` and `~/Documents/robot/vigil_rough_terrain_ws`.

| Option | Project | Sourced | Launch (the project's own, unchanged) | ROS_DOMAIN_ID |
|---|---|---|---|---|
| 1 Military Search and Rescue | `Military Search and Rescue/sar_ws` | `/opt/ros/jazzy` + that `install/setup.bash` | `ros2 launch vigil_sar sar_mission.launch.py` | 72 |
| 2 Agriculture | `Agriculture` | `/opt/ros/jazzy` + that `install/setup.bash` | `ros2 launch agri_ugv field.launch.py` (as `./run.sh`, without its rebuild) | 91 |
| 3 Rock Terrain | `Rock Terrain` | `/opt/ros/jazzy` + that `install/setup.bash` | `ros2 launch vigil_rough_terrain vision_nav.launch.py` | 71 |

## Setup (once)

```bash
bash vigil/setup_vigil.sh             # from the repository root
# if it asks: echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc && source ~/.bashrc
```

## Run

```bash
vigil
# or, without setup:
python3 vigil/master_launcher/vigil_launcher.py
```

Type 1, 2 or 3. **Ctrl+C** stops the running project. The launcher then stops anything the project
left behind and shows the menu again. Before a project starts, processes still running from any of
the three are stopped, so only one world, one robot and one dashboard exist at a time. The launcher
cleans the environment first, so a workspace sourced in `~/.bashrc` cannot leak into the chosen project.

## Build (each project on its own; the launcher never builds)

See section 4 of the [repository README](../README.md#4-build). Use a fresh terminal for each
build, so one workspace is never built on top of another.

If a project is not built, the menu prints the exact build command instead of starting it.

## Publish to GitHub (maintainer)

```bash
bash ~/Documents/vigil/publish_to_github.sh                       # default commit message
bash ~/Documents/vigil/publish_to_github.sh "Fix SAR sim speed"     # your own message
```

This copies the three projects and this launcher into a separate clone of
`VIGIL-MULTI-PURPOSE-ROBOT/vigil-multi-purpose-ugv` (`~/Documents/.vigil_publish`). It also copies
`REPO_README.md` as the front-page `README.md`. It shows what is new, and only after you type `y`
does it commit and push to `main` with your own GitHub login. Build output is never uploaded,
nothing on GitHub is deleted, and the folders on your PC are not changed.
