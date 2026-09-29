# Recorded Gazebo validation

The A-to-B comparison was driven only by wheel velocity commands. No robot pose, terrain, gravity, collision or joint-position manipulation was used.

| Model | Outcome | Arrival / last sample | Maximum measured tilt |
|---|---|---|---|
| eight wheel | reached | 27.70 simulation seconds to B | 34.96° |
| four wheel | stuck | 75.30 simulation seconds (common start) | 27.05° |

Arrival is measured from each robot’s motion-start delay; the last sample time for an unsuccessful rover is measured from common simulation start. Four-wheel failure was a 25-second lack of waypoint progress, not rollover.

The comparator is a derived rigid model, with retained front and rear ground-bearing wheels and mass transferred from removed assemblies to the payload. Starts are offset; this does not isolate suspension as the only experimental variable.

Calibration: flat.json reports 0.33753896 m at the lowest chassis-box corner after settling. Terrain test logs traverse.json and traverse2.json record earlier tilt-stop failures. Sensor reception and TF were verified; contact data and full Nav2 operation were not.

Model contract: four assertions groups passed, covering preserved geometry/interfaces, native spring conversion, positive physical inertia, and recovery of original axle CAD triangles.

Changed/new source files include urdf/agri_ugv.urdf.xacro, urdf/simulation.xacro, config/controllers.yaml, config/bridge.yaml, worlds/rock_terrain.sdf (physics/contact plugin only), launch/terrain_sim.launch.py, launch/comparison.launch.py, scripts/make_four_wheel.py, scripts/comparison_driver.py, test/model_contract.py, test/traverse.py, CMakeLists.txt, package.xml, two suspended-rocker meshes and eight extracted axle meshes. Original CAD meshes are retained. Terrain mesh files were not edited.

Agriculture verification: git status and git diff for ros2_ws were empty at completion. No agriculture file was written by this task. The initial /tmp hash manifest did not survive the interrupted session, so a full before/after hash comparison cannot be claimed.

The final run includes an exit manoeuvre: the eight-wheel rover drives to a parking point near (-4.7,-1.5) after reaching B. It parked clear before the four-wheel progress timeout. The four-wheel rover remained stuck near (-5.81,-3.03); it did not roll over. `comparison_obstructed.json` and the earlier screenshot are retained as preliminary evidence, not the final result.
