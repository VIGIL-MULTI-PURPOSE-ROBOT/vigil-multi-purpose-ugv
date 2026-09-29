# Capabilities, simulation evidence and technical innovation

[Project overview](../README.md) · [Architecture](architecture.md) · [Validation evidence](validation.md)

This page answers three questions: what is implemented, what has been demonstrated, and what differentiates VIGIL from a conventional UGV. “Implemented” means that the software module and its interfaces exist in the repository. It does not automatically mean that the capability has passed a complete mission benchmark or been deployed on physical hardware.

## Capabilities implemented in software

### Rover and sensing

- Eight-wheel skid-steer rover simulation with articulated rockers and simulated wheel suspension.
- RGB, depth, LiDAR, IMU, wheel-state and transform interfaces; the SAR configuration also includes a simulated thermal camera.
- ROS 2 and Gazebo integration, controller configuration and wheel-command generation.

### Terrain perception and planning

- Depth observations are projected into a world-referenced terrain grid.
- Terrain is classified as safe, uneven, climbable, steep, obstacle, possible drop or high cliff.
- Slope, roughness, step and drop rules are related to the simulated rover geometry.
- Footprint-aware A* planning accounts for rover width, length, turning clearance, hazard inflation and safety margin.
- Route updating and recovery states exist for blocked paths, hazardous terrain, loss of progress and excessive tilt.

### Mapping, missions and operator interfaces

- Agriculture integrates wheel odometry, IMU, EKF, RGB-D/LiDAR inputs and RTAB-Map mapping.
- Agriculture contains crop-row route generation, row-driving logic, obstacle processing, autonomy supervision and dashboard controls.
- SAR contains terrain navigation, simulated thermal processing, persistent human IDs, search waypoints, mission states and a control-station dashboard.
- Dashboards display camera imagery, terrain classes, rover state, goals, routes, hazards, decisions and event history.
- Agriculture, rough terrain and SAR are separate ROS workspaces. They share a rover concept but are not one common autonomy package.

## Capabilities demonstrated in simulation

| Demonstration | Recorded result | Qualification |
|---|---|---|
| Default rough-terrain mission | A 2.9 m path was generated and the dashboard reported `GOAL_REACHED` | One observed Gazebo run using `/sim/ground_truth` as pose input |
| Terrain interpretation | Safe, uneven, climbable and hazardous regions appeared in the live terrain map and camera overlay | Demonstrates the running depth-geometry pipeline; no perception-accuracy benchmark was performed |
| Cliff observation | The `cliff_front` capture displayed `HIGH CLIFF` and an estimated drop | The goal was changed during the capture, so this is perception evidence rather than a controlled default-goal result |
| Eight-wheel comparison | Historical records report that the eight-wheel rover reached B while the four-wheel configuration became stuck | Applies only to the documented simulation setup and route |
| Agriculture integrated demo | Short curved drive, Dijkstra return, lighting and suspension-mode checks were recorded | Does not establish a complete 23-row field mission |
| Agricultural return | A recorded short return ended about 0.216 m from its target position | One historical test, not a statistical localization benchmark |
| SAR motion | Documented flat, slope, hill, obstacle and cliff motion scenarios passed | A complete urban SAR mission has not been run to completion |
| Thermal-search pipeline | Simulated thermal observations can be processed into detections, persistent IDs and map markers | Physical thermal-camera performance is not established |

The exact capture context and limitations are recorded in [validation.md](validation.md). Historical results must not be combined across revisions as though they were one controlled benchmark.

## What remains to be demonstrated

- Sensor-derived visual or visual-inertial localization controlling every mission without simulator pose input.
- Repeated A-to-B trials reporting attempts, successes, collisions and operator interventions.
- Controlled moving-obstacle tests and agricultural headland-turn avoidance.
- Complete 23-row agriculture and complete urban SAR missions.
- A representative mining-world mission with mining-specific criteria.
- Sensor-loss, stale-pose, no-route and bounded safe-stop behavior.
- Physical-rover operation and validation of the assumed mass, inertia, contact and suspension values.

## Technical innovation compared with a conventional UGV

| Conventional approach | VIGIL approach |
|---|---|
| GPS, teleoperation or predefined waypoints provide the main navigation reference | The target architecture combines onboard environmental perception, mapping/localization and autonomous goal navigation for GPS-denied operation |
| A 2-D map often represents only free and occupied space | The terrain grid represents slope, roughness, steps, drops, climbable ground, obstacles and cliffs |
| The planner may approximate the rover as a point or circular radius | Planning checks the oriented rover footprint, turning clearance and inflated hazard boundaries |
| Obstacles are simply blocked | Terrain can be safe, costly, climbable or forbidden according to vehicle limits |
| Mechanical design and path planning are treated separately | Perception thresholds and route costs use wheel size, clearance, footprint and slope limits from the rover model |
| The operator sees a camera stream and final command | The dashboard exposes terrain interpretation, routes, replans and decision states |
| A platform is commonly configured for one mission | The VIGIL concept is evaluated across agriculture, rough terrain and SAR, with mining inspection documented as the next application |

### Geometry-aware traversability

VIGIL evaluates whether the rover can physically traverse an observed surface. A small step can be treated as climbable, a rough or sloped area can receive additional cost, and a sufficiently large drop can become a hard hazard. This is more informative than a binary obstacle map.

### Vehicle-aware path planning

The planner checks the body footprint along the route rather than checking only the centre point. This reduces the risk of selecting a path where the rover centre is clear but its wheels or body overlap a cliff or obstacle.

### Co-design of mobility and autonomy

The eight-wheel platform and navigation rules are connected through physical parameters. This creates the chain:

> camera depth → terrain geometry → vehicle traversability → route cost → motion command

### Interpretable decisions

States such as `CLIFF AHEAD`, `PATH BLOCKED`, `REPLANNING`, `SAFE PATH FOUND`, `CLIMBING`, `RECOVERY` and `GOAL_REACHED` show the operator why behavior changed. This improves reviewability during a demonstration and provides a basis for later safety monitoring.

