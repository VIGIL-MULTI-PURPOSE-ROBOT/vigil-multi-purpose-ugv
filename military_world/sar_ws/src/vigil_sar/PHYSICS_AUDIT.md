# Physics audit of the VIGIL SAR UGV

Audit of the **existing** rover: same CAD geometry, same navigation stack, same SAR system, same
world. Only the physical properties were corrected — mass distribution, inertia, actuation,
contact and the limits the navigation stack is allowed to ask for.

**No physics was faked.** Nothing teleports, nothing sets a pose or a velocity directly, gravity is
on, friction is finite, motor torque is finite, and the wheels are the only thing that moves the
rover. Every metre it travels is a contact force.

Calibration lives in one file: `config/physics.yaml`. The launch reads it, feeds the
geometry-independent numbers into the robot description, and logs what the drivetrain can actually
do. Change a number there and the whole rover changes — nothing is hard-coded in two places.

---

## 1. Architecture as found

| Layer | File | Role |
|---|---|---|
| Geometry | `urdf/cad_geometry.xacro` | CAD export: link poses, wheel radii, meshes |
| Robot | `urdf/agri_ugv.urdf.xacro` | links, inertia, joints, suspension, contact |
| Sim plumbing | `urdf/simulation.xacro` | `gz_ros2_control` hardware + command interfaces |
| Controller | `config/controllers.yaml` | joint group controller for the 8 wheel joints |
| Motor | `scripts/drive.py` | `/cmd_vel` → per-wheel command |
| Navigation | `scripts/cliff_navigator.py`, `scripts/planner_core.py` | A* + path following |
| World | `scripts/build_sar_world.py` | military_world → SDF, physics profile |
| Engine | Gazebo Harmonic, gz-physics **DART** | ODE collision detector, Dantzig LCP solver |

## 2. Problems found

| # | Problem | Consequence |
|---|---|---|
| 1 | Wheel joints driven through a **velocity** command interface | DART treats a velocity command as a constraint and enforces it with *unlimited* force. The rover had infinite torque: it could not stall on a slope, could not be slowed by load, could not slip, and its acceleration was a software ramp rather than force ÷ mass. This was the single biggest physics cheat in the model. |
| 2 | Wheels modelled as **solid rubber cylinders** | 162 kg of a 291 kg rover was wheels (56 %). Unsprung mass exceeded sprung mass, the rover behaved like a flywheel on casters, and suspension response was meaningless. |
| 3 | Wheel inertia = solid cylinder | 3.37 kg·m² total; a real tyre-on-rim is about a third of that, but distributed *outward*. Both the magnitude and the distribution were wrong. |
| 4 | Wheel 4 (rear pair) mounted at z ≈ 0 in the CAD export | Its tyre bottom sat 0.35 m above the other six — far outside the 0.22 m of suspension travel — so the rover permanently drove on six wheels. This matched the flat-ground test: `L4: NO contact / R4: NO contact`. |
| 5 | `kp`, `kd`, `minDepth`, `maxVel` on every wheel | Gazebo-classic **ODE** parameters. This project runs **DART**, which silently ignores all four. The contact model looked tuned and was not. |
| 6 | Single friction coefficient, no direction | A tyre grips far less sideways than it rolls. Without `mu2` and `fdir1`, skid steering cost nothing and the rover cornered like it was on rails. |
| 7 | No rolling resistance anywhere | Released throttle meant coasting forever; top speed was set by the joint limit rather than by drag. |
| 8 | No battery in the mass budget | The heaviest single item in a 291 kg electric rover was missing, so the centre of mass was too high and too far forward. |
| 9 | Suspension spring references all zero | With the wheels at different CAD heights the springs pushed the chassis into a permanent lean. |
| 10 | Navigation asking for 50 m/s, 2 rad/s at any speed | Beyond anything the drivetrain or the tyres could deliver — the commands were fiction and the rover spent its time fighting them. |

## 3. Dimensions — unchanged (CAD is ground truth)

| | value | source |
|---|---|---|
| Wheelbase (wheel 1 → wheel 4) | 1.135 m | CAD |
| Track (wheel centreline to centreline) | 0.873 m | rocker ±0.300 + hub ±0.136 |
| Front wheel radius / width (×2) | 0.23463 m / 0.186 m | CAD |
| Rear wheel radius / width (×6) | 0.17655 m / 0.141 m | CAD |
| Chassis floor above ground | ≈ 0.31 m | CAD |
| Centre of mass above ground | 0.378 m | computed from the corrected budget |
| Static lateral tip-over angle | 49° | atan(0.436 / 0.378) |

The only geometric correction was wheel 4's mount height, which the CAD export had lost (item 4).

## 4. Mass budget — 291.46 kg total (unchanged), redistributed

| Item | before | after |
|---|---|---|
| Chassis structure | 26.40 | 26.40 |
| Payload (sensors, computer, mission) | 65.71 | 65.71 |
| **Battery** | — | **84.15** (computed as the remainder) |
| 8 × drive motor (unsprung) | — | 24.00 |
| 8 × wheel | **162.2** | **54.0** |
| Carriers, guides, rockers, sensors | 37.2 | 37.2 |
| **Total** | **291.46** | **291.46** |

The battery is the balancing item: the launch computes it as `robot.mass` minus everything else, so
the links always sum to the stated total no matter which part is edited.

Wheels are now 19 % of the rover instead of 56 %.

## 5. Wheel and tyre parameters

| | value | reasoning |
|---|---|---|
| Front / rear wheel mass | 9.0 / 6.0 kg | tyre + rim of these sizes, not solid rubber |
| Inertia model | 70 % of mass as a ring at 0.92 R, 30 % as a disc at 0.55 R | mass sits at the tread |
| Axle inertia, front wheel | 0.316 kg·m² | between a solid cylinder (0.248) and a thin ring (0.496) |
| `mu1` longitudinal | 0.90 | rubber on dry concrete |
| `mu2` lateral | 0.70 | a tyre always grips less sideways |
| `fdir1` | `1 0 0` | ties the friction ellipse to the rolling direction |
| Rolling resistance Crr | 0.030 | applied as joint friction Crr·N·r ⇒ 86 N of drag |
| Restitution | 0.0 | tyres do not bounce |

## 6. Motor model

One geared motor per wheel, a straight DC/BLDC torque–speed line:

```
wheel stall torque  τs   = 8.55 N·m × 5 (gear) × 0.90 (efficiency)  = 38.5 N·m
wheel no-load speed ωnl  = 10000 rpm / 5                            = 209 rad/s
available torque    τ(ω) = τs · (1 − |ω| / ωnl)
commanded torque         = 3.0 · (ω_target − ω_measured), capped by τ(ω) driving
                           and by 40 N·m braking
```

Each wheel produces `τ/r` with **its own** radius, so the two wheel sizes are summed properly:

| Derived quantity | value |
|---|---|
| Tractive force at 0 m/s | 1636 N |
| Rolling drag | 86 N |
| Peak acceleration | 5.3 m/s² (grip ceiling 8.8 → traction-limited, as it should be) |
| Braking | 5.8 m/s² (also under the grip ceiling, so the tyres never lock) |
| Top speed (force meets drag) | 36.9 m/s |
| Climb limit | 33.2° torque-limited (grip would allow 42°) |

The drivetrain is the limit on climbing, not grip — which is what you want: the rover stalls rather
than spins its wheels.

## 7. Suspension

| | value |
|---|---|
| Carrier spring / damper | 8000 N/m, 650 N·s/m (ζ ≈ 0.9) |
| Travel | +0.12 / −0.10 m |
| Rocker spring / damper | 3500 N·m/rad, 300 N·m·s/rad |
| Static load per wheel | 357 N ⇒ 45 mm compression |

Spring rest positions are computed per wheel from the CAD heights so all eight tyres meet a flat
plane under static load. Tyre-bottom spread: **0.086 m** (was 0.347 m), inside the travel.

## 8. Engine

| | value | reasoning |
|---|---|---|
| Gravity | 9.81 m/s² | Earth |
| Step size | 1 ms | the stiffest spring oscillates in 62 ms; 1 ms samples it 62× |
| Collision detector | ODE | robust on the world's mesh terrain |
| Solver | Dantzig (direct LCP) | 8 wheels in simultaneous contact — PGS drifts |

The step size and solver are now pinned by the launch unconditionally. They used to be overridden
by the world export's own `<dart>` block whenever `fast:=true` was passed, so the rover quietly ran
on a different solver in fast mode than in normal mode.

## 9. Corrections applied

1. Torque actuation end to end — effort command interface, effort controller, torque-producing
   motor model with a real torque–speed curve.
2. Realistic mass distribution — tyre-and-rim wheels, motors as unsprung mass, battery added low
   and slightly rear.
3. Ring-and-disc wheel inertia instead of a solid cylinder.
4. Wheel 4 mount height corrected so all eight wheels can reach the ground.
5. Directional friction (`mu1`/`mu2`/`fdir1`), ODE-only knobs deleted.
6. Rolling resistance as joint Coulomb friction.
7. Per-wheel spring references so the rover sits level.
8. Joint effort and velocity limits set to the motor's real stall torque and no-load speed.
9. Navigation limited to what the rover can do: 35 m/s cruise, cornering capped by lateral grip,
   commands ramped at the real acceleration and braking rates.
10. Engine configuration pinned so it cannot drift between launch modes.

## 10. Files changed

| FILE | CHANGE | REASON |
|---|---|---|
| `config/physics.yaml` | **new** — single calibration file: engine, mass budget, wheel, motor, suspension, derived limits | Physical constants were scattered across the xacro, the launch and three scripts, so no two agreed. One file now owns them and everything else reads it. |
| `config/controllers.yaml` | `velocity_controllers/JointGroupVelocityController` → `effort_controllers/JointGroupEffortController` | A velocity controller cannot express a motor. Torque in, motion out. |
| `config/sar_mission.yaml` | `cruise_speed` 50 → 35; governor table `[0,20,30,50]` → `[0,14,21,35]`; `brake_decel` 6.0 → 5.8; added `max_lateral_accel`, `max_accel`, `max_decel` | The rover's real top speed is 36.9 m/s, so 50 was an order the drivetrain could only refuse. The new numbers are the measured capability, and the added limits stop the navigator asking for a corner or a step the tyres cannot hold. |
| `urdf/cad_geometry.xacro` | wheel 4 (L and R) mount z: 0 → −0.262 m | The CAD export lost this offset, leaving the rear pair 0.35 m in the air. The rover drove on six wheels. |
| `urdf/agri_ugv.urdf.xacro` | wheel masses from config; new `wheel_inertia` ring+disc macro; battery link and joint; motor mass on each carrier; joint `effort`/`velocity` limits from the motor; joint friction = rolling resistance; `mu1`/`mu2`/`fdir1`; `kp`/`kd`/`minDepth`/`maxVel` deleted; per-wheel spring references | This is where almost every wrong physical property lived: solid-rubber wheels, no battery, no rolling resistance, isotropic friction, ODE knobs DART ignores, and springs that could not level the chassis. |
| `urdf/simulation.xacro` | `<command_interface name="velocity"/>` → `effort` | The root cause of the infinite-torque rover. DART enforces a velocity command with whatever force it takes and ignores the joint effort limit. |
| `scripts/drive.py` | rewritten: `/cmd_vel` + `/joint_states` → eight wheel **torques** through a geared motor curve, with drive and braking caps | The node used to command wheel velocities, which bypassed physics entirely. Now the wheels can slip, stall, be slowed by a slope, and coast to a stop. |
| `scripts/planner_core.py` | new `CommandShaper`: lateral-grip yaw cap and acceleration ramp | The physics of cornering belongs with the planning maths, not in a ROS node, so it can be tested without ROS. |
| `scripts/cliff_navigator.py` | uses `CommandShaper` on every published twist | At 35 m/s a 2 rad/s turn asks for 70 m/s² of lateral force; the tyres can hold 6.9. The rover now takes a wide line instead of sliding. |
| `launch/sim.launch.py` | loads `physics.yaml`, balances the mass budget through the battery, derives force/acceleration/top speed/climb over **both** wheel radii, warns when a configured limit exceeds what the drivetrain produces, passes the calibration into the xacro and into `drive.py`, pins engine step/detector/solver | One place computes the rover's capability and one place logs it, so a wishful number in a config file is now a warning at startup rather than a rover that will not do what it is told. |
| `launch/sar_mission.launch.py` | passes the physics parameter file through to the driving nodes | The motor model needs the calibration. |
| `test/test_sar_offline.py` | new `PhysicsConfigTests` (7) and `RobotModelTests` (9) | The offline half of the physics validation suite: limits match the drivetrain, grip is the ceiling and not the floor, step size resolves the suspension, navigation cannot ask for more than the rover has, cornering and acceleration are actually capped, mass budget balances, wheels are not solid rubber, inertia is ring-and-disc, every wheel can reach flat ground, joint limits are the motor's, rolling resistance sums to Crr·m·g, the effort interface is in place, no ODE-only knobs remain, and the centre of mass is low enough not to roll. |

**Startup log now reads:**

```
[vigil_sar] physics: 1 ms step, ode collision detector, dantzig solver
[vigil_sar] physics (config/physics.yaml): 291.5 kg (battery 84.2 kg balances the budget),
    wheels 54 kg, wheel torque 38.5 N.m x8 = 1636 N, drag 86 N -> 5.3 m/s2
    (grip ceiling 8.8), top speed 37 m/s, climb 33 deg torque / 42 deg grip
```

## Calibration status

**Derived, not measured.** The total mass and all geometry come from the CAD. The mass
*distribution*, the wheel inertia model, the motor curve, friction and rolling resistance are
engineering estimates sized to the stated targets. Nothing here has been checked against a real
rover, so the simulation is *physically consistent*, not *real-world accurate*. Measured values
replace these numbers one line at a time in `config/physics.yaml`.

## Still to do

* On-ground validation against the simulator: acceleration, braking distance, straight-line
  tracking, turning radius, in-place rotation, small/large obstacle, up/down hill, side slope,
  low/high friction, cliff, collision, suspension compression and rebound.
* Terrain and rock collision quality: several world meshes still use axis-aligned bounding boxes
  where a convex hull or a primitive fit would be both cheaper and correct.
* Dashboard physics panel (speed, acceleration, attitude, wheel speeds, motor state, slip estimate,
  suspension state) and a physics debug visualisation mode.
