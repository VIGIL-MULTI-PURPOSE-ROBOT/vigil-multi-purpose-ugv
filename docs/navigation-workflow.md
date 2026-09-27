# Navigation workflow

[Project overview](../README.md) · [Architecture](architecture.md) · [Run instructions](setup.md)

## Rough-terrain mission, from start to finish

1. **Bring up the simulation.** Gazebo loads the terrain and rover. ROS bridges connect camera, pose and other sensor streams; the wheel controller is activated.
2. **Acquire observations.** The mapper waits for depth, camera calibration and simulator pose. The navigator includes an initial settling period.
3. **Project and classify.** Depth pixels become world points; local surface geometry determines terrain classes and traversal costs.
4. **Accept the destination.** Default B comes from configuration. Launch overrides or the dashboard can set another goal in the world frame.
5. **Plan with the rover footprint.** A* evaluates route cost and clearance rather than treating the rover as a point.
6. **Track the route.** Navigation publishes a velocity request and the drive layer generates wheel commands.
7. **React to changes.** Updated terrain may cause replanning, climbing behavior, local avoidance or recovery. The dashboard displays the decision and event history.
8. **Declare arrival.** The navigator reports `GOAL_REACHED` according to its configured tolerance. The reported arrival distance and later settled distance may differ.

## Important behavior limits

The planner includes optimistic treatment of unobserved ground and fallback recovery behavior. Later implementation revisions can continue searching/reversing instead of terminating when a route is unavailable. Do not infer a certified emergency-stop policy from earlier documentation or a single successful video.

Validate camera dropout, stale pose, tilt events, blocked goals, unreachable routes and intervention behavior separately. A live visualization alone cannot establish collision-free operation.

## Presentation sequence

- Explain the camera and pose inputs before starting the demo.
- Show the depth overlay and traversability map together.
- Point out A, B, the planned route and the rover footprint.
- Show the navigation decision and goal event.
- State which measurements are simulator-provided and which are estimated.
- Close with recorded limitations and the next visual-localization milestone.

## Other missions

Agriculture defaults to a crop-row controller behind a dashboard start/stop gate; mapping and Nav2 also have their own roles. SAR adds thermal search after or alongside navigation. Refer to [detailed operations](operations.md) for their controls and to [architecture](architecture.md) for their distinct pose sources.
