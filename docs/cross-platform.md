# Run VIGIL on Linux, macOS or Windows

[Project overview](../README.md) · [Native Ubuntu setup](setup.md)

VIGIL's ROS 2 and Gazebo dependencies are Linux applications. The portable
runtime packages them in a Docker container, allowing the rough-terrain demo to
run the same way on Linux, macOS and Windows without installing ROS on the host.

## Requirements

- Docker Desktop on Windows or macOS, or Docker Engine with the Compose plugin
  on Linux.
- At least 8 GB of memory available to Docker. Software rendering is used by
  default, so a host GPU is not required, but simulation speed depends on the
  host CPU.
- On Windows, use Linux containers. Both PowerShell and Command Prompt can run
  the commands below.

## Start

From the repository root:

```console
docker compose up --build
```

The first start downloads ROS and builds the VIGIL package. Later starts reuse
that image. When the service is healthy, open:

- <http://localhost:6080/?autoconnect=true&resize=scale> for the complete Gazebo
  desktop. Use the browser's full-screen mode for the largest view.
- <http://localhost:8080> for the VIGIL camera, terrain map, planned path and
  mission status dashboard.

Stop the application with `Ctrl+C`, then remove the stopped container:

```console
docker compose down
```

## Configuration

Compose reads these optional environment variables:

| Variable | Default | Purpose |
|---|---:|---|
| `VIGIL_DESKTOP_PORT` | `6080` | Host port used for the Gazebo browser desktop |
| `VIGIL_DESKTOP_SIZE` | `1600x900x24` | Virtual desktop width, height and colour depth |
| `VIGIL_DASHBOARD_PORT` | `8080` | Host port used for the web dashboard |
| `VIGIL_SCENARIO` | empty | Named test scenario, such as `cliff_front` |
| `VIGIL_CAMERA_WIDTH` | `960` | Dashboard camera width |
| `VIGIL_CAMERA_HEIGHT` | `720` | Dashboard camera height |
| `VIGIL_CAMERA_RATE` | `5` | Camera frames per second |
| `VIGIL_ROS_DOMAIN_ID` | `71` | ROS 2 domain used inside the container |

For example, in macOS/Linux shells:

```console
VIGIL_SCENARIO=cliff_front VIGIL_DASHBOARD_PORT=8081 docker compose up
```

In PowerShell:

```powershell
$env:VIGIL_SCENARIO = "cliff_front"
$env:VIGIL_DASHBOARD_PORT = "8081"
docker compose up
```

Then open <http://localhost:8081>.

## Scope and troubleshooting

The portable profile runs the rough-terrain simulation and the Gazebo desktop
through the browser. It uses CPU-based software rendering for consistent host
support; simulation speed therefore depends on the host CPU. Native Ubuntu
remains the supported path for direct GPU acceleration, hardware access and
development across all mission workspaces.

- If the container exits immediately, make sure Docker is using Linux
  containers and has enough memory.
- If the dashboard is slow, lower the camera size or rate.
- If port 8080 is occupied, set `VIGIL_DASHBOARD_PORT` to another port.
- If port 6080 is occupied, set `VIGIL_DESKTOP_PORT` to another port.
- View runtime logs with `docker compose logs -f vigil`.
- Rebuild after source changes with `docker compose up --build`.
