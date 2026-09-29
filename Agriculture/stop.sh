#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
python3 - <<'PY'
import os,signal,time
from pathlib import Path
lock=Path('run/simulation.lock')
if not lock.exists():print('No tracked simulation is running.');raise SystemExit(0)
identity=lock.stat();holders=[]
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit() or int(proc.name)==os.getpid():continue
    try:
        if proc.stat().st_uid!=os.getuid() or (proc/'cwd').resolve()!=Path.cwd():continue
        for fd in (proc/'fd').iterdir():
            try:s=fd.stat()
            except OSError:continue
            if (s.st_dev,s.st_ino)==(identity.st_dev,identity.st_ino):holders.append(int(proc.name));break
    except OSError:continue
if not holders:print('Simulation is already stopped.');raise SystemExit(0)
for pid in holders:
    try:os.kill(pid,signal.SIGINT)
    except ProcessLookupError:pass
end=time.monotonic()+12
while time.monotonic()<end and any(Path('/proc',str(p)).exists() for p in holders):time.sleep(.2)
for pid in holders:
    try:
        if Path('/proc',str(pid),'cwd').resolve()==Path.cwd():os.kill(pid,signal.SIGTERM)
    except (ProcessLookupError,FileNotFoundError):pass
end=time.monotonic()+3
while time.monotonic()<end and any(Path('/proc',str(p)).exists() for p in holders):time.sleep(.2)
for pid in holders:
    try:
        if Path('/proc',str(pid),'cwd').resolve()==Path.cwd():os.kill(pid,signal.SIGKILL)
    except (ProcessLookupError,FileNotFoundError):pass
print('Stopped this workspace simulation, including orphaned children.')
PY
