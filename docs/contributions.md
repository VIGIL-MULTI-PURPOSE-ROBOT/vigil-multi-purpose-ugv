# Contributions and third-party attribution

[Project overview](../README.md)

## Team work represented in the repository

The repository contains rover model adaptation, ROS/Gazebo sensor and controller integration, depth-terrain processing, planning/control code, agricultural and SAR mission logic, dashboards, worlds and tests. The team reports AutoCAD/Fusion 360 mechanical design work. The available [CAD notes](../ros2_ws/CAD_UPDATE.md) describe inferred joints and physical assumptions; exact CAD fidelity is not claimed.

Use commit history and linked source modules when presenting individual contributions. No individual roles have been invented for this documentation update.

## External foundations

ROS 2, Gazebo Harmonic, ros2_control, robot_localization, RTAB-Map and Nav2 provide external middleware, simulation, estimation and navigation components. Describe the team's configuration/integration separately from authorship of those projects.

## Rocky terrain credit

This work is based on [“Rocky Terrain n4”](https://sketchfab.com/3d-models/rocky-terrain-n4-95939b2b45d14c4b8858180b5497398a) by [DarkPixel](https://sketchfab.com/darkpixel8), licensed under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). The terrain is scaled/adapted for simulation. The new rough-terrain captures depict this asset.

Original notices are preserved in the [rough-terrain package](../vigil_rough_terrain_ws/src/vigil_rough_terrain/ASSET_LICENSE.txt) and [SAR package](../military_world/sar_ws/src/vigil_sar/ASSET_LICENSE.txt).

The supplied README draft referred to `tdf_gazebo`. That attribution has not been carried over as the provenance of the current default rough-terrain world: the inspected package identifies the DarkPixel asset above. If another scenario later imports `tdf_gazebo`, document that dependency and its licence at that time.

## Licence boundaries

This documentation update does not choose a new licence for the team's code or CAD. Existing package metadata and third-party notices remain in place. A team-approved top-level licence is still needed before claiming a uniform open-source licence for the repository. Preserve third-party attribution independently of that decision.
