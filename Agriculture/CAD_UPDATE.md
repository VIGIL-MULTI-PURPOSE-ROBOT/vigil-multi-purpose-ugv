# Agriculture rover CAD update

The default `agri_ugv` spawned in the existing cotton farm now uses
`/home/user/Downloads/full shhhh_activesuspension.stl` (SHA-256
`660776500e243604f17f388921b051b83f0a0db94dda5d01ccfc34054a9ba37e`).
The agricultural world and launch command are unchanged.

```bash
cd /home/user/Documents/robot/ros2_ws
./run.sh
```

For an initial inspection without autonomy, SLAM, or RViz:

```bash
./run.sh stage:=3 rviz:=false row_mission:=false explore:=false
```

Stop with Ctrl-C before starting another run. `run.sh` builds automatically.

## Model changes

- All 557,698 triangles / 90 connected components are assigned exactly once to
  21 meshes: chassis and mounted hardware, two rockers, eight tyres, eight hubs,
  camera, and LiDAR. The new export also replaces the static CAD preview.
- STL millimetres are converted to metres; CAD -Y is forward, +X is left, +Z is up.
- Each wheel rotates around its measured axle location. CAD asymmetry and the
  elevated rear wheels are retained; the model is not flattened into eight
  identical wheel heights. Physical wheel tracks are approximately 0.932 m at
  the front and 0.871 m at the remaining axles.
- The placeholder side bars are replaced with CAD rocker and cylinder geometry.
  Eight provisional sliding suspension joints are replaced by fixed axle mounts
  so the wheels stay attached to the visible shafts. Two existing effort-driven
  rocker joints retain springs, damping, limits and controller names.
- Existing eight-wheel drive commands, six encoder interfaces, sensor topics,
  total assumed mass (286.11 kg), and autonomy interfaces are retained.


## Reproduce and check

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/import_cad_preview.py '/home/user/Downloads/full shhhh_activesuspension.stl'
python3 tools/import_rover_cad.py '/home/user/Downloads/full shhhh_activesuspension.stl'
colcon build --symlink-install
python3 tools/check_structure.py
python3 tools/check_cad_geometry.py
python3 -m pytest -q tests
```

`src/agri_ugv_description/config/cad_geometry.json` records every component
assignment, source hash, mesh origin and wheel dimension. The importer refuses
unreviewed STL revisions rather than applying component IDs to unrelated data.
The geometry check rebuilds the assembly through the URDF joint tree at the
standing angle and compares every exported vertex against the supplied STL.
Its evidence and preview are in `reports/cad_update/`.

## Validation on 2026-09-21

- Both modified ROS packages build successfully; all four existing tests pass.
- URDF parses and converts to Gazebo SDF; controller state interfaces match the
  revised joint graph. Geometry reconstruction verifies every triangle with a
  maximum coordinate error of 5.92e-8 m.
- A 20-second headless field test at stage 3 loaded all three controllers and
  received valid RGB, depth, LiDAR, IMU and six encoder streams. A bounded
  0.25 m/s drive request produced 3.65 m horizontal displacement. Final chassis
  roll/pitch were -1.28 / 0.50 degrees. Evidence: `reports/cad_update/field_drive.json`.
- The smoke test used the CAD-aligned zero-pitch camera frame; afterward the
  original 10-degree sensor pitch was restored with an inverse visual transform.
  The final geometry check verifies that transform. Driving geometry is unchanged.
- This checks spawning, sensors and short motion, not full row missions or
  hydraulic performance. The test simulation was stopped after validation.
