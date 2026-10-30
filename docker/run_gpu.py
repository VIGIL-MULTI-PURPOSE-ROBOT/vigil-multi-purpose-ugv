#!/usr/bin/env python3
"""Independent Linux GPU runtime; never connects to another simulation host."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import uuid

ROOT = Path(__file__).resolve().parent.parent


def main():
    if not sys.platform.startswith('linux') or 'microsoft' in os.uname().release.lower():
        raise RuntimeError('This profile requires native Linux with X11 or XWayland. WSL/macOS need a separate graphics configuration.')
    for tool in ('docker', 'xauth'):
        if not shutil.which(tool):
            raise RuntimeError(f'Install {tool} first.')
    display = os.environ.get('DISPLAY', '')
    if not display.startswith(':'):
        raise RuntimeError('Run from a local graphical desktop terminal with DISPLAY set (for example :0).')
    subprocess.run(['docker', 'info'], check=True, stdout=subprocess.DEVNULL)
    gpu = []
    if shutil.which('nvidia-smi') and subprocess.run(
            ['nvidia-smi'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
        gpu = ['--runtime', 'nvidia', '--gpus', 'all',
               '-e', 'NVIDIA_VISIBLE_DEVICES=all',
               '-e', 'NVIDIA_DRIVER_CAPABILITIES=all',
               '-e', '__NV_PRIME_RENDER_OFFLOAD=1',
               '-e', '__GLX_VENDOR_LIBRARY_NAME=nvidia',
               '-e', '__VK_LAYER_NV_optimus=NVIDIA_only']
        if Path('/dev/dri').exists():
            gpu += ['--device', '/dev/dri:/dev/dri']
    elif Path('/dev/dri').exists():
        gpu = ['--device', '/dev/dri:/dev/dri']
    else:
        raise RuntimeError('No usable GPU found. Install the host graphics driver first.')
    cookies = subprocess.check_output(['xauth', 'nlist', display], text=True)
    if not cookies.strip():
        raise RuntimeError('No display authentication cookie found. Run as your desktop user, without sudo.')
    with tempfile.TemporaryDirectory(prefix='vigil-display-') as temp:
        auth = Path(temp) / 'Xauthority'
        auth.touch(mode=0o600)
        # FamilyWild makes this cookie usable from the container hostname.
        cookies = '\n'.join('ffff' + line[4:] for line in cookies.splitlines()) + '\n'
        subprocess.run(['xauth', '-f', str(auth), 'nmerge', '-'], input=cookies, text=True, check=True)
        subprocess.run(['docker', 'build', '-t', 'vigil-rough-terrain:local', '.'], cwd=ROOT, check=True)
        name = 'vigil-gpu-' + uuid.uuid4().hex[:12]
        cmd = ['docker', 'run', '--rm', '--name', name, '--shm-size', '2g',
               '-p', '127.0.0.1:8080:8080', '-e', f'DISPLAY={display}',
               '-e', 'XAUTHORITY=/tmp/vigil.xauth', '-e', 'VIGIL_RENDER_MODE=hardware',
               '-e', 'QT_X11_NO_MITSHM=1',
               '-v', '/tmp/.X11-unix:/tmp/.X11-unix:ro',
               '-v', f'{auth}:/tmp/vigil.xauth:ro', *gpu,
               'vigil-rough-terrain:local']
        print('Gazebo will open as a desktop window. Dashboard: http://localhost:8080', flush=True)
        try:
            return subprocess.call(cmd)
        except KeyboardInterrupt:
            return 130
        finally:
            subprocess.run(['docker', 'stop', '--time', '15', name],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (RuntimeError, subprocess.CalledProcessError) as exc:
        print(f'VIGIL: {exc}', file=sys.stderr)
        sys.exit(1)
