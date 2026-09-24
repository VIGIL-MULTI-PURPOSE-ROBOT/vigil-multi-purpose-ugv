# military_world

Autonomous **search-and-rescue / disaster-response** simulation range for an
8-wheel UGV. Authored procedurally in Blender, built to be exported to
**Gazebo Sim Harmonic** and driven from ROS 2.

**Non-weaponised.** No weapons, no ranges, no combat targets, no offensive
scenarios anywhere in this scene or its generator.

## Run with moving obstacles

```bash
bash run_gazebo.sh                 # full environment, people walking
bash run_gazebo.sh --demo          # two visible people crossing near the origin
bash run_gazebo.sh --contact       # people collide with the world and the robot
bash run_gazebo.sh --software      # use software rendering if the GPU fails
bash run_gazebo.sh --server        # physics server only
bash run_gazebo.sh --demo --check  # validate SDF and measure motion + pause
bash run_gazebo.sh --check         # check all 20 movers in the full environment
```

### Two ways the people can move

`<controller_version>` inside each `sar::WaypointSystem` plugin selects one.
`run_gazebo.sh` writes it for you through `sar_physics.py`.

| | `--kinematic` (1, default) | `--contact` (2) |
|---|---|---|
| How it moves | placed on its route pose every step | forces push a dynamic body along the route |
| Collides with terrain, walls, props, the robot | no, walks through them | yes |
| Blocked by an obstacle | no, keeps going | yes, waits until the way clears |
| Can fall over or sink | no | possible if physics is unhappy |
| Cost | negligible | a rigid body and contacts per person |

Kinematic is the default because it always works: the person is where the
route says it is, which is what a perception or detection experiment needs.
Switch to `--contact` when you specifically want the robot to be physically
obstructed by a person. If a model cannot support the contact controller --
no canonical link, no `<inertial>`, no world parent -- the plugin logs a
warning naming the reason and runs that one person kinematically instead of
bringing the server down.

Switching modes only rewrites SDF; meshes are reused, so it is fast.

The launcher builds the local Gazebo Harmonic plugin automatically. Build
requirements are `cmake`, `g++`, and `libgz-sim8-dev`. Blender is needed only
when exporting or updating the scene. Export errors stop the launch and are
saved in `gazebo_export/export.log`; existing meshes are never deleted first.

The full world contains **19 moving people and one carried stretcher**.
Each walks at a fixed `<cruise_speed>` -- 2.2 m/s alone, 1.8 m/s for the
stretcher party, so bearers stay together -- and long authored rests are
compressed to a third so nobody stands still for a whole minute. Restart
Gazebo after rebuilding the plugin.
Look for `SAR_DynamicPerson_024` near (4, 22), `SAR_DynamicPerson_027`
near (5, 26), or the stretcher party near (-7, 38) in the Entity Tree.
Several patrols deliberately dwell at waypoints; others move immediately.
The `--demo` world removes the surrounding terrain and clutter so motion is
easy to see. Press Play if you have manually paused the simulation.

Use `--static` for a frozen comparison, `--reexport` to regenerate the
meshes, or `--actors` as a compatibility alias for the default moving mode.
The 77 loose physics props move under gravity/contact and are not scripted.

The full terrain and contact simulation can run slower than real time. Motion
is measured in simulation seconds, so use the small demo to distinguish slow
physics from a stopped controller. Software rendering helps some GPU setups,
but cannot repair an incompatible EGL / graphics-driver installation.


## Files

| file | what it is |
|---|---|
| `military_world.blend` | the scene |
| `generate_military_sar_environment.py` | the generator that produced it |
| `export_gazebo.py` | Blender -> glTF/COLLADA meshes + per-object SDF |
| `sar_lighting.py` | lighting preset switcher (run inside Blender) |
| `sar_metadata.json` | semantics, traversability, thermal, zones, roads, hazards |
| `sar_ground_truth.json` | **evaluation only** -- casualty + hazard truth |
| `sar_thermal_table.csv` | thermal class -> apparent temperature / emissivity |
| `sar_dynamic_paths.json` | dynamic-human waypoint loops, portable form |
| `sar_dynamic_props.json` | physics-reactive props: mass, inertia, surface |
| `sar_physics_preview.py` | arms a Blender rigid-body preview (preview only) |
| `sar_world_template.sdf` | reference skeleton + `<actor>` trajectory blocks |
| `sar_validation_report.json` | machine-checked results of the checks below |

---

## 1. Environment dimensions

- 300 m x 300 m, X in [-150, 150], Y in [-40, 260], Z up.
- Metric throughout: 1 Blender unit = 1 m, `scale_length = 1.0`. No
  conversion factor anywhere in the pipeline.
- The base sits on the world origin and the map runs north, so mission
  coordinates near home are small numbers.
- Relief: base apron at 0 m, mountain crest around +30 m. Nothing alpine;
  slopes are sized for a wheeled UGV, with a few deliberate exceptions.

## 2. Sector layout

```
  Y=260  +---------------------------------------------------+
         |   SNOW / AVALANCHE        MOUNTAIN / CLIFFS       |
         |   (-45,228)                (72,216)               |
         |                                                   |
  Y=170  |   FOREST (-85,168)     OPEN FIELD N (6,166)       |
         |                                  DESERT (104,128) |
  Y=110  |   FLOOD /        RUBBLE + COLLAPSED               |
         |   RIVER          BUILDINGS (-2,108)               |
  Y= 55  |   (-98,58)     OPEN FIELD S     URBAN DISASTER    |
         |                  (-18,48)        (88,44)          |
  Y=  0  |                  == BASE (0,-14) ==               |
  Y=-40  +---------------------------------------------------+
        X=-150                                          X=150
```

Sectors are **not** rectangles. They are noise-warped inverse-distance
weighted fields around those centroids, and the height function is blended
the same way, so every boundary is a transition rather than a step. The
robot drives grass -> scrub -> forest floor -> rock without ever crossing a
seam. Terrain per tile:

- `GRASS`: 6 tile(s) dominant
- `SAND`: 5 tile(s) dominant
- `MUD`: 4 tile(s) dominant
- `GRAVEL`: 4 tile(s) dominant
- `ROCK`: 4 tile(s) dominant
- `SOIL`: 3 tile(s) dominant
- `SNOW`: 3 tile(s) dominant
- `RUBBLE`: 3 tile(s) dominant
- `CONCRETE`: 3 tile(s) dominant
- `WATER`: 1 tile(s) dominant

## 3-6. Humans

| | count |
|---|---|
| total human entities | **34** |
| static | 15 |
| dynamic | 19 |
| stationary casualties (lying / sitting / kneeling) | 11 |
| partially occluded | 9 |
| thermal HIGH / MEDIUM / LOW contrast | 14 / 16 / 4 |

Distribution by zone:

- `SAR_ZONE_A`: 10
- `SAR_ZONE_B`: 10
- `SAR_ZONE_C`: 10
- `SAR_ZONE_D`: 2
- `SAR_ZONE_E`: 1
- `SAR_ZONE_F`: 1

Poses: standing, walking, sitting, kneeling, lying. Statuses: `AMBULATORY`,
`INJURED_RESPONSIVE`, `INJURED_UNRESPONSIVE`, `TRAPPED`, `RESCUER`. No
graphic injury is modelled -- status is metadata, not geometry.

**What the humans actually are.** Segmented proxy humanoids: head, torso,
hips, two arms, two legs, each a separate object at correct human scale
(1.55-1.87 m). They are not rigged characters. Dynamic humans move by
keyframed root transforms with a 1 s limb-swing cycle on an F-Curve CYCLES
modifier. That is enough for LiDAR returns, depth, bounding boxes, thermal
blobs and obstacle avoidance. It is not enough to train a pose estimator or
to look photoreal on an RGB camera. If you need better, swap in rigged
meshes and keep the custom properties -- the pipeline consumes the
metadata contract, not the mesh.

## 6b. Dynamic obstacles

Two mechanisms, deliberately different, because they fail a planner in
different ways.

**Scripted movers** — 19 walking humans plus 1 carried stretcher(s).
They follow bounded, seeded loops and move regardless of what the robot
does. Motion patterns present: `CONVERGING`, `CROSSING`, `GROUP_CARRY`, `HEAD_ON`, `ORBIT`, `PATROL`, `ROAD_PATROL`.

- `HEAD_ON` — walks the main supply route and turns around, closing on
  the robot with almost no lateral optical flow.
- `CONVERGING` — two tracks that merge near (0, 68), occlude each other,
  then separate. The classic ID-swap trap for nearest-neighbour
  association; their speeds differ so the meeting drifts instead of
  repeating on a fixed beat.
- `CROSSING` — cut across the main route, Street A/C and the rubble
  bypass. The bypass one matters most: the robot meets it while already
  mid-replan around the blocked main road.
- `GROUP_CARRY` — a two-person stretcher party. Both bearers and the
  stretcher share one key list, so they hold formation over the whole
  run. Together they read as one long obstacle to a clustering
  front-end and three separate tracks to a naive one.
- `ORBIT` — laps a building, passing through the urban canyon and going
  fully out of sight each circuit: track continuation through total
  occlusion.

**Physics-reactive props** — 77 objects in 9 clusters.
Barrels, crates, pallets, cones, sheet panels, loose pipe, jerry cans and
wheeled bins. These are **not keyframed**. Each carries mass, a computed
principal inertia tensor, friction and restitution, and exports as a
non-static SDF model — every bit of their motion comes from Gazebo's
solver when something pushes them.

That distinction is the point. A keyframed obstacle moves the same way
whatever the robot does, so avoidance can succeed by accident. A prop
that only moves when hit punishes a planner for treating "it moved" as
"it will keep moving", and produces the case that breaks naive
static-world SLAM: a landmark that is in the map, then is somewhere else,
because the robot moved it.

| kind | mass | shape | behaviour |
|---|---|---|---|
| `BARREL` | 21.0 kg | CYL_Z 0.29 x 0.88 | Part-full steel drum. Tips, then rolls. |
| `CRATE` | 14.0 kg | BOX 0.60 x 0.60 x 0.55 | Supply crate. Slides and tumbles, does not roll. |
| `PALLET` | 16.0 kg | BOX 1.20 x 0.80 x 0.14 | Flat and low -- a genuine LiDAR ambiguity: the robot must decide whether to drive over it. |
| `CONE` | 4.5 kg | CYL_Z 0.19 x 0.75 | Traffic cone. Light enough to knock flat without stopping the robot -- tests whether contact is detected at all. |
| `PANEL` | 9.0 kg | BOX 1.40 x 0.90 x 0.05 | Loose sheet metal. Low friction, slides a long way. |
| `PIPE` | 15.0 kg | CYL_X 0.12 x 2.20 | Loose pipe lying across the ground. Rolls perpendicular to its axis and not at all along it. |
| `JERRYCAN` | 6.5 kg | BOX 0.35 x 0.18 x 0.45 | Fuel can. Small enough to vanish under a bumper. |
| `BIN` | 9.0 kg | BOX 0.58 x 0.52 x 0.92 | Wheeled bin. Tall, light, topples easily. |

Cluster exposure is mixed on purpose: 2 of 9 clusters sit on a route
corridor (the base gate and Urban Street A), the rest are adjacent or
off-route so they enter sensor range without stopping the robot every
thirty seconds.

- **`SAR_DynProps_BaseYard`** (ADJACENT) at (-14, -8) r6 m — Staging yard on the apron. First movable things the robot sees, on flat ground where a failed push is unambiguous.
- **`SAR_DynProps_BaseGate`** (ON_ROUTE) at (2, 3) r4 m — Cones narrowing the base gate. Directly on the exit path -- the robot meets these in the first ten metres of every run.
- **`SAR_DynProps_StreetA`** (ON_ROUTE) at (61, 44) r7 m — Street furniture strewn across Urban Street A, the alternative when Street C is blocked by the overturned truck.
- **`SAR_DynProps_Canyon`** (ADJACENT) at (80, 50) r3 m — Inside the 7.5 m canyon between URB_001 and URB_002. Narrows an already tight corridor, and the walls leave nowhere to swing wide -- the robot either pushes through or backs out.
- **`SAR_DynProps_WarehouseYard`** (ADJACENT) at (108, 40) r5 m — Loading yard outside the warehouse roller door.
- **`SAR_DynProps_RubbleEdge`** (ADJACENT) at (14, 104) r7 m — Loose pipe and sheet at the collapse-field margin, beside the rubble bypass. Rolling pipe on sloped debris is the nastiest case here: it moves after the robot has passed it.
- **`SAR_DynProps_FloodBank`** (OFF_ROUTE) at (-96, 58) r8 m — Debris washed onto the flood plain. Scattered, low contrast against mud.
- **`SAR_DynProps_DesertTrack`** (ADJACENT) at (116, 104) r6 m — Dropped stores beside the desert track. Sand gives low friction under both the prop and the robot.
- **`SAR_DynProps_ForestStaging`** (ADJACENT) at (-40, 162) r5 m — Forward staging point on the forest track, under canopy shadow.

To watch one topple in Blender, run the embedded `sar_physics_preview.py`
and press Play. It arms Blender's own rigid-body system on the props and
on the terrain tiles beneath them only — making all 36 tiles passive mesh
colliders is enough to make Bullet crawl. It is a preview; Blender rigid
bodies do not export, and Gazebo is what actually runs these.

## 7. Buildings

6 structures, plus the base compound.

- **`SAR_Building_URB_001`** (3 storey, damage `MODERATE`, roof `PARTIAL`) at (70, 50) -- 3-storey office block. West face of the urban canyon.
- **`SAR_Building_URB_002`** (3 storey, damage `LIGHT`, roof `FULL`) at (89, 50) -- 3-storey block. East face of the urban canyon; ~7.5 m of open street between 001 and 002 with 10 m walls either side -> GPS multipath / sky occlusion, and wide enough to drive.
- **`SAR_Building_URB_003`** (2 storey, damage `SEVERE`, roof `COLLAPSED`) at (76, 79) -- Partially collapsed 2-storey. Victim inside, visible ONLY through the collapsed north-east wall opening.
- **`SAR_Building_URB_004`** (1 storey, damage `MODERATE`, roof `PARTIAL`) at (114, 56) -- Warehouse. Wide roller entrance + narrow side door + one blocked door; dark interior, rooms and a corridor.
- **`SAR_Building_RUB_001`** (1 storey, damage `COLLAPSED`, roof `COLLAPSED`) at (-14, 100) -- Collapsed house. One survivable void with a crawl-height opening; casualty adjacent, NOT buried.
- **`SAR_Building_RUB_002`** (2 storey, damage `SEVERE`, roof `PARTIAL`) at (14, 118) -- Partially collapsed 2-storey. Three approaches: wide (unsafe), narrow (tight clearance), blocked. Forces entrance selection.

Walls are assembled from solid piers and lintels around real openings --
1.05 x 2.10 m doors, 1.25 x 1.20 m windows on a 1.00 m sill. No booleans and
no invisible doorway planes: the mesh, the collision proxy and the LiDAR all
agree on where the gap is. Interiors have rooms, a corridor with doorways,
and furniture, so the indoor/outdoor localisation transition is real.

## 8. Vehicles

10 static vehicles: rescue truck, medical van, utility 4x4, five damaged
civilian vehicles, an overturned truck blocking a street, a stranded
vehicle at the flood margin, an abandoned desert truck.

Each carries a separate `_EngineBay` child with its own thermal class, so a
recently-run engine is a legitimate 62 C blob and a long-cold wreck is 15 C.

**No dynamic vehicles.** The spec said to add them *only if the pipeline
supports their movement correctly*, and it does not: Blender keyframes do
not survive a mesh export into an SDF world. Moving humans are handled
through waypoint loops and the Gazebo motion plugin; vehicles
would need the same treatment plus wheel/suspension models, which belongs
in the Gazebo layer, not here. Adding them in Blender would have looked
like progress and delivered nothing.

## 9. Hazards

32 hazards with ground-truth markers and metadata:

- `DITCH`: 5
- `RUBBLE`: 4
- `HOLE`: 3
- `STEEP_SLOPE`: 3
- `BLOCKED_PATH`: 3
- `DEAD_END`: 3
- `TRENCH`: 2
- `RAVINE`: 2
- `ROAD_WASHOUT`: 2
- `CLIFF`: 2
- `WATER`: 2
- `COLLAPSED_GROUND`: 1

Negative obstacles are **real voids in the terrain mesh** -- 5 ditches,
3 holes, 2 trenches, 2 ravines, 2 road washouts, 1 subsidence. Tiles
containing them are tessellated at 0.50 m instead of 1.00 m so a 1.6 m hole
is actually a hole. Some are flagged `OBVIOUS`, some `SUBTLE` (soft lips, in
vegetation or shadow) -- the subtle ones are a genuine 3D-LiDAR and depth
problem, not a texture.

Both cliffs are modelled as a 5.6-7.8 m drop over roughly 2.5 m of run. The robot
must refuse them from terrain analysis; nothing invisible stops it.

## 10. Terrain types and traversability

| class | traversability | rel. cost | nominal mu | risk |
|---|---|---|---|---|
| `ROAD` | EASY | 1.0 | 0.85 | LOW |
| `CONCRETE` | EASY | 1.0 | 0.82 | LOW |
| `GRASS` | EASY | 1.2 | 0.65 | LOW |
| `SOIL` | EASY | 1.3 | 0.62 | LOW |
| `GRAVEL` | MODERATE | 1.6 | 0.55 | LOW |
| `DIRT` | MODERATE | 1.5 | 0.58 | LOW |
| `SNOW` | DIFFICULT | 2.6 | 0.30 | MEDIUM |
| `MUD` | DIFFICULT | 3.0 | 0.28 | MEDIUM |
| `SAND` | DIFFICULT | 2.9 | 0.35 | MEDIUM |
| `ROCK` | DIFFICULT | 3.2 | 0.70 | MEDIUM |
| `RUBBLE` | VERY_DIFFICULT | 5.5 | 0.60 | HIGH |
| `WATER` | FORBIDDEN | n/a | 0.05 | CRITICAL |
| `CLIFF` | FORBIDDEN | n/a | 0.70 | CRITICAL |

Classification happens in the same pass as the height, per terrain face, so
the label and the geometry cannot disagree. Transitions present in the
scene: grass->mud, grass->forest, forest->rock, rock->mountain, road->rubble,
road->flood, grass->snow, snow->rock, grass->sand.

## 11. Search areas

- **`SAR_ZONE_HOME`** (TRIVIAL) -- Base / staging. Initialisation, docking, return-to-base
- **`SAR_ZONE_A`** (EASY) -- Open search area. Lawnmower / grid coverage, long-range LiDAR, baseline detection
- **`SAR_ZONE_B`** (HARD) -- Collapse field + forest. Rubble traversability, forest occlusion, building interiors
- **`SAR_ZONE_C`** (MEDIUM) -- Urban disaster. GPS-denied urban canyon, blocked routes, multi-person triage
- **`SAR_ZONE_D`** (HARD) -- Mountain + snow. Risk-aware route choice, cliffs, ravines, avalanche debris
- **`SAR_ZONE_E`** (MEDIUM) -- Desert transition. Wheel slip, dune traverse, low-feature localisation
- **`SAR_ZONE_F`** (HARD) -- Flood / river margin. Water boundary detection, washouts, bridge crossing

Difficulty ladder: EASY = open field, MEDIUM = urban / desert, HARD =
rubble / forest / mountain / snow / flood. The overall mission boundary is a
logical polygon (`SAR_MISSION_BOUNDARY`), not a fence -- only the base
compound is physically fenced, and it has a 9 m vehicle gate.

## 12. Ground-truth system

`GROUND_TRUTH` holds `GT_PERSON_*` and `GT_HAZARD_*` empties, zone outlines,
the mission polygon and the validation routes. Every one carries full
metadata: id, position, type, static/dynamic, thermal class and delta-T,
occlusion, search zone, difficulty.

**It is not linked into `SAR_EXPORT`, and the validator asserts that.** The
robot is never given a map, a pose, a casualty position or a terrain label.
Ground truth is for scoring a run afterwards. Publish it at runtime and the
environment stops measuring anything.

## 13. Blender collections

```
SAR_GENERATED
  SAR_ENVIRONMENT
    SAR_TERRAIN  BASE  OPEN_FIELD  FOREST  MOUNTAIN  SNOW  DESERT
    FLOOD  URBAN  RUBBLE  BUILDINGS  VEHICLES  VEGETATION  ROADS
    BRIDGES  HAZARDS  STATIC_HUMANS  DYNAMIC_HUMANS  PROPS
  LIGHTING          6 sun presets + base floodlights + weather holders
  GROUND_TRUTH      evaluation only, never exported
  SAR_EXPORT        everything Gazebo needs, and nothing it should not see
  SAR_VISUAL        high-quality visual geometry
  SAR_COLLISION     objects that participate in physics
  SAR_HUMAN_TEST    representative static/dynamic/pose/occlusion set
  SAR_SENSOR_TEST   depth ladder, clearance gates, decoys, landmarks
```

`SAR_EXPORT` / `SAR_VISUAL` / `SAR_COLLISION` / `SAR_HUMAN_TEST` /
`SAR_SENSOR_TEST` are **views** -- objects are multi-linked, not duplicated.
One object, one source of truth, no drift between a visual copy and a
collision copy that someone edited six months later.

Object counts per collection:

- `SAR_GENERATED`: 3537
- `SAR_ENVIRONMENT`: 3281
- `SAR_VISUAL`: 3238
- `SAR_EXPORT`: 3144
- `SAR_COLLISION`: 2483
- `VEGETATION`: 1667
- `FOREST`: 684
- `RUBBLE`: 578
- `SAR_HUMAN_TEST`: 272
- `GROUND_TRUTH`: 193
- `SNOW`: 188
- `DYNAMIC_HUMANS`: 153
- `SAR_SENSOR_TEST`: 143
- `BUILDINGS`: 137
- `STATIC_HUMANS`: 120
- `DYNAMIC_PROPS`: 78
- `VEHICLES`: 74
- `SAR_TERRAIN`: 36
- `BASE`: 36
- `HAZARDS`: 34
- `MOUNTAIN`: 33
- `LIGHTING`: 16
- `ROADS`: 14
- `BRIDGES`: 6
- `OPEN_FIELD`: 1
- `FLOOD`: 1
- `URBAN`: 1
- `DESERT`: 0
- `PROPS`: 0

## 14. Naming

```
SAR_Terrain_<col>_<row>            SAR_Road_<name> / SAR_Street_* / SAR_Track_*
SAR_Building_<SECTOR>_<nnn>_<part>  SAR_Bridge_001_<part>
SAR_StaticPerson_<nnn>_<part>       SAR_DynamicPerson_<nnn>_<part>
SAR_Vehicle_<Role>_<nnn>_<part>     SAR_Tree_/Bush_/Rock_/Boulder_<nnnn>
SAR_Ditch_/Hole_/Trench_/Ravine_/Cliff_<nnn>   SAR_Rubble_<nnn>_Piece_<nnnn>
SAR_Decoy_<Kind>_<nnn>             SAR_Proto_<Kind>_<nn>   (hidden templates)
GT_PERSON_<nnn>  GT_HAZARD_<nnn>  SAR_ZONE_<X>  SAR_Route_<nn>_<Name>
SAR_ROBOT_START  SAR_BASE_LOCATION  SAR_DOCK_POSE  SAR_SEARCH_START/END
```

## 15. Materials

31 reusable RGB materials (`MAT_*`), assigned per terrain face by class, and
19 thermal **preview** materials (`TPREV_THERMAL_*`) whose emission grey
encodes apparent temperature. The preview set is a sanity-check aid, not a
simulation -- see section 18.

## 16. Collision strategy

- **Terrain: visual mesh == collision mesh.** Deliberate. A decimated
  collision copy quietly heals ditches, holes and washouts, which is the one
  failure this environment exists to prevent. 276036 verts / 270000 quads total,
  adaptive resolution so the cost lands where the geometry matters.
- **Discrete objects carry a `collision` property naming their proxy** and
  `export_gazebo.py` derives it from the bounding box at export time:
  `BOX`, `BOX_COMPOUND`, `CYLINDER`, `CYLINDER_TRUNK_ONLY` (trunks only --
  nobody should pay for leaf collision), `CONVEX_HULL`, `CAPSULE_APPROX`
  (humans), `BOX_LOW` (bushes: soft obstacle), `NONE_VISUAL_ONLY` (grass,
  water surfaces, antennas, signage).
- Grass has no collision at all. Put it in the costmap and the robot
  refuses to move.

## 17. Export strategy

```bash
blender --background military_world.blend --python export_gazebo.py -- \
        --out ./gazebo_export          # add --format dae or obj to override
```

Writes a directory you can run directly:

```
gazebo_export/military_world.sdf     one complete world, ~2.5 MB
gazebo_export/meshes/*.glb           one mesh per unique Blender datablock
```

Then:

```bash
export GZ_SIM_RESOURCE_PATH=$PWD/gazebo_export
gz sim -v 4 gazebo_export/military_world.sdf
```

The exporter writes **one mesh per unique mesh datablock, not per object**.
About 3000 of the objects here are linked duplicates sharing their mesh
(trees, rocks, rubble, props), so this is ~712 files instead of ~3100, and
34 MB instead of several hundred. Per-object scale rides on `<mesh><scale>`.

It also **groups the seven parts of each human into one model** with one
link and seven visuals. A walking person you have to move by driving seven
poses in lockstep is a trap; one model, one pose.

Semantic properties come across as an SDF comment on each model, and
thermal temperature as a `<temperature>` in Kelvin on each visual. Add
`--models` to additionally write a standalone `models/<name>/` library.
Prototype objects (`SAR_Proto_*`) are hidden and never exported.

**Scatter is batched.** Gazebo charges per MODEL -- an SDF parse, an Ogre
scene node and a physics body each -- not per mesh file. An earlier version
of this exporter emitted one model per object: 2922 of them, 2494 being
scattered trees, bushes, rocks and rubble, and Gazebo crashed on load.
Scatter now merges into one model per (family, 50 m cell): one joined
visual mesh, cheap primitive collisions inside a single link.

| | one model per object | batched |
|---|---|---|
| models | 2922 | **517** |
| mesh collisions | 992 | **67** |
| world file | 2.5 MB | 0.9 MB |

The fidelity trade is deliberate: a scattered rock exports as an oriented
box rather than a convex hull, because ~1000 mesh collisions is what makes
the solver crawl and a box is an honest stand-in for a 40 cm rock the robot
must not drive through. Everything the robot interacts with on purpose --
terrain, roads, the bridge deck, buildings, vehicles, humans and the 77
dynamic props -- keeps its exact collision. `--no-batch` restores the old
behaviour if you want to see it fail.

Export produces regular models for all geometry, including 20 scripted
movers and 77 non-static physics props. Moving people retain their separate
body-part visuals and collision shapes. The carried stretcher appears once.

**Staged test worlds:**

| world | what it isolates |
|---|---|
| `stage1_one_mesh.sdf` | mesh format and resource path |
| `stage2_terrain.sdf` | terrain collision cost |
| `stage3_structures.sdf` | buildings, vehicles and static humans |
| `stage4_scatter.sdf` | scatter batches and scripted motion |
| `stage5_movers.sdf` | terrain and scripted movers |
| `dynamic_demo.sdf` | two moving people on a flat pad near the origin |

Older `stage5_actors_plain.sdf` / `stage5_actors_skin.sdf` files may remain
from previous exports; they are obsolete. Use `stage5_movers.sdf` instead.

The exporter builds timed loops from each Blender object's waypoint, speed,
dwell and formation-offset properties. `sar_motion.py` produces the route
configuration; `plugins/WaypointSystem.cc` interpolates it every simulation
step. Pause stops motion, and resetting simulation time restarts the routes.
The loop includes the initial pose at its end, avoiding a position jump.
Stretcher bearers retain their forward offsets through turns and loop wraps.

**Motion limitations:** these are prescribed moving collision obstacles.
They are static bodies repositioned by the plugin, so they resist contact
forces and keep following their route. They are suitable for demonstrating
obstacle detection and avoidance, not realistic human impact dynamics.
Limb poses are frozen; the whole person moves and turns without a walking
gait. The 77 loose props retain normal dynamic physics.

For manual launch, after building the plugin:

```bash
export GZ_SIM_RESOURCE_PATH="$PWD/gazebo_export${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
gz sim -r gazebo_export/military_world.sdf
```

`--no-actors` on the Blender exporter freezes scripted models. The normal
launcher keeps a moving master export and writes a separate frozen SDF for
`--static`, avoiding a full mesh re-export to switch modes.

To pick the motion controller by hand, without the launcher:

```bash
python3 sar_physics.py --controller 1 gazebo_export/*.sdf   # kinematic
python3 sar_physics.py --controller 2 gazebo_export/*.sdf   # contact-aware
```

It is idempotent -- running it repeatedly does not stack duplicate
`<inertial>`, `<member>` or `<dart>` elements -- so it is safe to re-run over
a world that has already been configured.

## 18. Gazebo compatibility

- Meshes export as glTF binary (`.glb`) by default. Harmonic loads it, it
  carries materials, and every Blender build has the exporter -- the
  Debian/Ubuntu Blender package ships with COLLADA compiled out, so a
  `.dae` pipeline fails on exactly the machines you least expect. Pass
  `--format dae` or `--format obj` if your toolchain needs it; the exporter
  falls back on its own if a format is unavailable.
- SDF 1.10, Gazebo Sim **Harmonic**, `gz-sim-*` system plugin names. No
  Gazebo Classic assumptions. **No Isaac Sim anywhere.**
- Z-up, right-handed, metric, Y-forward COLLADA export.
- No geometry nodes, no procedural shader textures, no volumetrics in any
  exported object -- nothing that dies at the mesh boundary.

## 19. Known limitations -- read these

1. **Thermal is metadata, not physics.** Blender has no radiometric path.
   Every relevant object carries `thermal_class` and `thermal_temp_c`, and
   `sar_thermal_table.csv` is the handoff to a Gazebo thermal camera. The
   `TPREV_*` materials only let you eyeball contrast in a grey viewport.
   Nothing in this scene claims LWIR sees through walls, rubble or snow:
   interior casualties are reachable only through openings, and
   `PERSON_014` is snow-*dusted*, not buried.
2. **Humans are proxy humanoids**, not rigged characters (section 3-6).
3. **Blender animation does not export.** Dynamic-human motion is portable
   only via `sar_dynamic_paths.json` / the generated `<actor>` blocks.
4. **Weather is a table, not a simulation.** Fog/rain/dust/snowfall need
   Gazebo `<scene><fog>` and sensor noise models configured from
   `sar_metadata.json`. Blender volumetrics stay in Blender.
5. **ROS 2 Humble + Gazebo Harmonic is not an upstream-supported pair.**
   Harmonic pairs with Jazzy; Humble's binary `ros_gz` targets Fortress. If
   you stay on Humble you are building `ros_gz` from source against
   Harmonic. The world itself is version-agnostic, so this is a bridge
   problem -- but budget for it before demo week.
6. **Prop physics is configured here, not validated here.** Mass and
   inertia are computed analytically and written into each model.sdf, but
   nothing in this pipeline runs a Gazebo step. Expect to tune
   `max_step_size`, contact `max_vel` and `min_depth` once you spawn
   them — a 4.5 kg cone on a mesh terrain is exactly the mass range where
   a soft contact solver jitters.
7. **Vegetation is low-poly and untextured.** Good LiDAR silhouettes and
   real occlusion; it will not win a render competition. That trade was
   made on purpose in favour of simulation rate.
8. **No texture maps.** Flat materials only. Feature-based visual SLAM will
   find geometry (walls, trunks, containers, rubble, the comms mast) but
   little surface texture. If you are testing a descriptor-based VO
   front-end, add textures before drawing conclusions.
9. **The base apron is genuinely flat** by design, so the
   `no_large_perfectly_flat_areas` check tolerates it.

## 20. How to regenerate

```bash
blender --background --python generate_military_sar_environment.py -- \
        --out /home/user/Documents/military_world --seed 20260915

# quick low-res iteration pass
blender --background --python generate_military_sar_environment.py -- --fast
```

Same seed -> byte-identical layout. Sub-seeds are derived per subsystem
(`sub_rng("rubble_scatter")` etc.) with a CRC32-stable hash, so re-tuning
rubble does not reshuffle where the casualties are, and `PYTHONHASHSEED`
cannot silently break reproducibility.

Regeneration deletes **only** the `SAR_GENERATED` subtree. A robot model,
imported assets, other scenes and your own collections are untouched. Run it
on a .blend that already has your UGV in it and the UGV survives.

## 21. How to move the humans

Edit `STATIC_HUMANS` / `DYNAMIC_HUMANS` near the top of the generator. One
dict per person:

```python
dict(pid="PERSON_002", x=-78.0, y=160.0, pose="LYING",
     cloth="MAT_HUMAN_DARK", th="HIGH", occ="VEGETATION_PARTIAL",
     status="INJURED_UNRESPONSIVE", zone="SAR_ZONE_B",
     sector="FOREST", diff="HARD", head=0.8)

dict(pid="PERSON_016", speed=1.25, dwell=(0.0, 0.0),
     path=[(-14,66), (-2,70), (10,74), (20,70), (8,64), (-14,66)])
```

`pose` is one of `STANDING`, `WALKING`, `SITTING`, `KNEELING`, `LYING`. `th` is HIGH/MEDIUM/LOW
thermal contrast. Z comes from the terrain automatically -- never hand-set
it. Ground truth, the JSON files and the SDF actors all regenerate from
these lists, so there is exactly one place to edit.

If you want a victim guaranteed occluded, add an entry to the `plan` list in
`build_occluders()`; it runs before the general scatter so the occluder is
certain rather than lucky.

## 22. How to change the weather

`WEATHER_PRESETS` in the generator, mirrored into `sar_metadata.json`:
`CLEAR`, `RAIN`, `FOG`, `DUST`, `SNOWFALL`, `NIGHT_CLEAR`. Each carries rain/dust/snowfall
intensity, wind, visibility and the lighting preset it implies. In Blender
they are hidden config holders in `LIGHTING`. In Gazebo, transcribe them
into `<scene><fog>` and your sensor noise models -- and for `RAIN`, drop
`friction_mu` by about 0.15 on sealed surfaces and 0.08 elsewhere.

Do not model `SNOWFALL` or `DUST` as total sensor failure. The objective is
robust navigation under degradation, not a scripted blackout.

## 23. How to change the lighting

Six presets: `DAY`, `CLOUDY`, `SUNSET`, `LOW_LIGHT`, `NIGHT`, `FOG`. One sun object per preset in
`LIGHTING`; `DAY` is active and the rest are hidden. Switch with
`sar_lighting.py` (embedded as a Blender text datablock -- open it and hit
Run Script), which also sets the world colour, strength and fog density.
Nothing is baked into textures.

`NIGHT` is the one that matters: the base floodlights stay on, everything
else goes near-dark, ambient air drops to 9 C and human-to-background
delta-T is at its largest. It is not pitch black, because the point is
fusion, not blinding the RGB camera.

## 24. How to build new SAR scenarios

1. **New casualty scenario** -- add to `STATIC_HUMANS` with the thermal
   contrast and occlusion you want to test, then add an occluder to
   `build_occluders()` if the occlusion must be guaranteed.
2. **New hazard** -- add to `NEGATIVES` (real carved void; the tile
   auto-refines to 0.50 m) or `CLIFFS`, and give it a `vis` of `SUBTLE` if
   you want it to be hard to see. Ground truth and metadata follow.
3. **New route decision** -- add two roads to `ROADS` between the same
   endpoints with different exposure, as `SAR_Track_Mountain_Risk` and
   `SAR_Track_Mountain_Safe` already do, then block one with debris.
4. **New decoy** -- add to `DECOYS`. `WARMCASE` is a thermal false positive,
   `MANNEQUIN` an RGB one, and placing them within a few metres of a real
   casualty is what makes classification hard rather than academic.
5. **New sector** -- add a centroid to `SECTORS` and a height function to
   `SECTOR_H`. Blending, classification, transitions and metadata are
   automatic; you do not touch the tile loop.
6. Re-run with a **new seed** for a fresh randomisation of the same design,
   or the same seed to reproduce a run exactly.

---

## Validation

Run automatically at the end of generation, against the generated field --
measured, not asserted.

**50/50 checks passed, 0 warning(s).**

- PASS `extent_300x300` -- 300 x 300 m
- PASS `start_pose_flat` -- 1.23 deg
- PASS `start_pose_6m_clear_of_hazards` -- 6 m radius
- PASS `start_faces_open_ground_25m` -- heading +Y
- PASS `route_SAR_Route_01_Disaster_no_water` -- 0 samples in water
- PASS `route_SAR_Route_01_Disaster_slope_under_32deg` -- max 6.1 deg at (22, 121)
- PASS `route_SAR_Route_02_Mountain_no_water` -- 0 samples in water
- PASS `route_SAR_Route_02_Mountain_slope_under_32deg` -- max 12.7 deg at (96, 211)
- PASS `route_SAR_Route_03_UrbanFlood_no_water` -- 0 samples in water
- PASS `route_SAR_Route_03_UrbanFlood_slope_under_32deg` -- max 11.9 deg at (48, 43)
- PASS `route_SAR_Route_04_Snow_no_water` -- 0 samples in water
- PASS `route_SAR_Route_04_Snow_slope_under_32deg` -- max 18.6 deg at (-88, 183)
- PASS `route_SAR_Route_05_AllTerrain_no_water` -- 0 samples in water
- PASS `route_SAR_Route_05_AllTerrain_slope_under_32deg` -- max 7.6 deg at (3, 146)
- PASS `humans_total_20_plus` -- 34
- PASS `static_humans_10_plus` -- 15
- PASS `dynamic_humans_10_plus` -- 19
- PASS `stationary_casualties_5_plus` -- 11
- PASS `occluded_humans_5_plus` -- 9
- PASS `thermal_targets_10_plus` -- 34
- PASS `multi_person_cluster_3_to_5` -- {"TRIAGE_01": 4}
- PASS `ditches_5_plus` -- 5
- PASS `holes_3_plus` -- 3
- PASS `steep_slopes_3_plus` -- 3
- PASS `rubble_zones_3_plus` -- 4
- PASS `cliffs_2_plus` -- 2
- PASS `water_boundaries_2_plus` -- 2
- PASS `blocked_paths_2_plus` -- 3
- PASS `buildings_3_to_6` -- 6
- PASS `vehicles_5_plus` -- 10
- PASS `human_heights_1p5_to_1p9` -- min 1.55 max 1.85
- PASS `door_width_realistic` -- 1.05 m
- PASS `door_height_realistic` -- 2.10 m
- PASS `road_widths_3_to_7m` -- min 3.0 max 6.0
- PASS `tree_heights_4_to_17m` -- min 4.3 max 14.8
- PASS `vehicle_sizes_realistic` -- 4.4 - 7.2 m
- PASS `terrain_types_6_plus` -- {"WATER": 1, "MUD": 4, "SOIL": 3, "GRASS": 6, "SNOW": 3, "GRAVEL": 4, "RUBBLE": 3, "ROCK": 4, "CONCRETE": 3, "SAND": 5}
- PASS `no_large_perfectly_flat_areas` -- 2 of 36 tiles with <0.15 m relief (base apron is expected)
- PASS `clearance_gates_narrow_and_wide` -- 0.80 / 1.00 / 1.40 m gates built
- PASS `spawn_clearance_6m_radius` -- clear
- PASS `building_footprints_clear_of_carriageways` -- clear
- PASS `urban_canyon_gap_over_5m` -- 7.53 m
- PASS `physics_reactive_props_30_plus` -- 77
- PASS `dynamic_props_have_positive_mass_and_inertia` -- bad: none
- PASS `dynamic_props_not_in_water_or_voids` -- bad: none
- PASS `prop_clusters_mixed_exposure` -- 2 of 9 clusters on a route corridor
- PASS `dynamic_obstacles_total_25_plus` -- 97
- PASS `motion_patterns_5_plus` -- CONVERGING, CROSSING, GROUP_CARRY, HEAD_ON, ORBIT, PATROL, ROAD_PATROL
- PASS `ground_truth_not_in_export` -- leaked: none
- PASS `no_prebuilt_occupancy_grid` -- no navigation map, no occupancy grid, no pose oracle in the scene

### Route traversability (sampled every 1.5 m off the real height field)

| route | length | max slope | in water | near neg. obstacle | terrain mix |
|---|---|---|---|---|---|
| `SAR_Route_01_Disaster` | 152 m | 6.1 deg | 0 | 0 | ROAD 66%, GRAVEL 26%, CONCRETE 5%, RUBBLE 3% |
| `SAR_Route_02_Mountain` | 343 m | 12.7 deg | 0 | 0 | GRAVEL 60%, ROAD 38%, ROCK 1%, CONCRETE 0% |
| `SAR_Route_03_UrbanFlood` | 168 m | 11.9 deg | 0 | 0 | ROAD 79%, CONCRETE 21% |
| `SAR_Route_04_Snow` | 341 m | 18.6 deg | 0 | 0 | DIRT 46%, ROAD 31%, GRAVEL 22%, CONCRETE 0% |
| `SAR_Route_05_AllTerrain` | 570 m | 7.6 deg | 0 | 0 | ROAD 70%, DIRT 12%, GRAVEL 12%, MUD 3% |

### The mission this is built to support

Robot spawns at `SAR_ROBOT_START` with no map, no pose oracle and no GNSS.
It initialises, starts SLAM, explores, classifies terrain, detects positive
and negative obstacles, plans safe routes, meets dynamic humans and yields,
finds a stationary casualty with RGB, confirms with thermal, localises it
with LiDAR/depth, transforms into the map frame, marks it, continues, hits a
blocked road, replans onto the bypass, evaluates rubble traversability,
chooses the safer line, finds the collapsed building, searches its accessible
volume, finds a second casualty, computes coverage, and drives home to the
dock.

Every one of those steps has geometry in this scene that makes it possible
and nothing in this scene that makes it free.
