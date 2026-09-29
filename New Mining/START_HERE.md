# New Mining

Underground mine simulation: RGB-D visual SLAM, wheel/IMU fusion, gas/temperature/humidity sensing,
thermal imaging and a localhost dashboard with Point B navigation.

| Environment | Dashboard |
|---|---|
| ![VIGIL rover in the New Mining environment](cover.png) | ![New Mining dashboard](Images/Dashboards/dashboard.png) |

| What | Where |
|---|---|
| ROS 2 package `vigil_new_mining` | [`src/vigil_new_mining/`](src/vigil_new_mining) |
| Setup, licensed mine asset and validation scope | [`README.md`](README.md) |
| Start script | [`run.sh`](run.sh) |
| Environment and launcher images | [`Images/`](Images) |
| Runtime data flow and third-party asset notes | [`../docs/new-mining.md`](../docs/new-mining.md) |

Run: `./run.sh` from this folder (`ROS_DOMAIN_ID=74`), or VIGIL launcher option 5.
The mine model itself is a licensed third-party asset and is not in the repository.
