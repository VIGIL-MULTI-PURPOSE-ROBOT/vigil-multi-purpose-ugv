#!/usr/bin/python3
"""Simulation speed (Gazebo real-time factor): measurement and the live 1x / 2x / 4x switch.

Simulation speed is EXECUTION speed only. Gazebo integrates the same 1 ms physics steps with the same
gravity, masses, friction and torques; at 4x it simply tries to do four simulated seconds per wall
second. Every node runs on simulation time (/clock), so sensor stamps, the navigator, the drive ramp
and the walkers all stay in step whatever the factor is. If the computer cannot keep up, Gazebo runs
as fast as it can: RtfMeter measures what was actually achieved and the dashboard shows it next to
the target.
"""
import subprocess
from collections import deque


class RtfMeter:
    """Measured real-time factor = simulated seconds / wall seconds over a sliding window."""

    def __init__(self, window=5.0):
        self.window = float(window)
        self.samples = deque()

    def add(self, wall, sim):
        self.samples.append((float(wall), float(sim)))
        while len(self.samples) > 2 and self.samples[-1][0] - self.samples[0][0] > self.window:
            self.samples.popleft()

    def rtf(self):
        if len(self.samples) < 2:
            return None
        (w0, s0), (w1, s1) = self.samples[0], self.samples[-1]
        if w1 - w0 < 1e-3 or s1 < s0:            # a world reset jumps sim time back
            return None
        return (s1 - s0) / (w1 - w0)


def set_physics_command(world, rtf, step):
    """gz service call that changes the target real-time factor (the step is sent unchanged: Gazebo
    only accepts both together)."""
    return ['gz', 'service', '-s', f'/world/{world}/set_physics', '--reqtype', 'gz.msgs.Physics',
            '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
            '--req', f'max_step_size: {float(step):g}, real_time_factor: {float(rtf):g}']


def set_simulation_speed(world, rtf, step, timeout=6.0):
    """Returns (ok, message)."""
    try:
        r = subprocess.run(set_physics_command(world, rtf, step), capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as e:
        return False, f'gz service failed: {e}'
    out = (r.stdout + r.stderr).strip()
    ok = r.returncode == 0 and 'true' in out.lower()
    return ok, out or f'exit {r.returncode}'
