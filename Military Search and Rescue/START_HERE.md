# Military Search and Rescue

Navigation across a 300 x 300 m disaster area, followed by a thermal search for people.

| Environment | Dashboard |
|---|---|
| ![VIGIL rover in the Military Search and Rescue environment](cover.png) | ![Military Search and Rescue dashboard](Images/Dashboards/dashboard.png) |

| What | Where |
|---|---|
| ROS 2 workspace (package `vigil_sar`) | [`sar_ws/`](sar_ws) |
| Gazebo world (Blender source, exported SDF, meshes) | [`military_world.blend`](military_world.blend), [`gazebo_export/`](gazebo_export) |
| Full documentation of this environment | [`README.md`](README.md), [`sar_ws/README.md`](sar_ws/README.md) |
| Environment and dashboard images | [`Images/`](Images) |
| Validation records | [`validation/`](validation), [`sar_validation_report.json`](sar_validation_report.json) |
| Overview, setup and evidence | [`../docs/`](../docs) |

Run (after building, see `sar_ws/README.md`): `ros2 launch vigil_sar sar_mission.launch.py` with `ROS_DOMAIN_ID=72`,
or start it from the VIGIL launcher (option 1) in [`../vigil/`](../vigil).
