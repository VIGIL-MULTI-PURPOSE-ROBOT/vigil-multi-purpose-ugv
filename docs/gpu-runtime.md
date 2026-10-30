# Independent GPU runtime

This profile runs Rock Terrain entirely on the destination computer. No connection
to the original developer's computer is required. It targets native Linux with
an X11 or XWayland desktop and Docker Engine. It does not guarantee a frame rate:
CPU, GPU, driver, and terrain workload still determine performance.

Install Docker Engine, Python 3 and xauth on the destination machine. Install
the host graphics driver. For NVIDIA, also install and configure the
[NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
for Docker. `nvidia-smi` must work on that host. Intel/AMD use `/dev/dri`.

After obtaining the updated repository, stop any previous VIGIL simulation,
open a terminal in the repository, and run:

```bash
python3 docker/run_gpu.py
```

Or install the terminal menu once, then choose option 3:

```bash
bash vigil/setup_vigil.sh
vigil --gpu
```

Gazebo opens in a desktop window. Open http://localhost:8080 for the dashboard.
There is no browser desktop on port 6080 in this mode. Ctrl+C stops the container.
The first run builds the image; subsequent builds reuse cached layers.

The launcher passes a temporary X authentication cookie instead of disabling
display access control. Run it as the desktop user with Docker access, without
sudo. Startup prints the OpenGL renderer and rejects llvmpipe/softpipe software
rendering. On NVIDIA laptops it explicitly selects the NVIDIA runtime and PRIME
offload path. If NVIDIA device allocation fails, check Container Toolkit setup.

Windows/WSL and macOS are not supported by this native-Linux GPU profile.
Use the existing software profile there until an OS-specific graphics path is
configured and tested. Docker GPU compute support alone does not prove Gazebo
OpenGL acceleration. Only Rock Terrain is packaged in this image.

Validation: shell/Python syntax and launcher control-flow checks pass. Hardware
rendering and performance must still be tested on a destination GPU host.
