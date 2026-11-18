#!/usr/bin/env bash
# Usage: ./kill_sim.sh [-n | -9]
# Standalone Linux cleanup; no ROS environment or DDS discovery required.
exec python3 - "$@" <<'PYTHON'
import argparse
import os
from pathlib import Path
import signal
import time

SCOPE = "simulation"
LAUNCHES = [('webots_simulation', 'diff_sim_launch.py'), ('webots_simulation', 'simulation_launch.py'), ('webots_simulation', 'localization_launch.py'), ('webots_simulation', 'slam_launch.py')]
parser = argparse.ArgumentParser(description=f"Stop this user's differential {SCOPE} session.")
mode = parser.add_mutually_exclusive_group()
mode.add_argument('-n', '--dry-run', action='store_true')
mode.add_argument('-9', '--force', action='store_true')
options = parser.parse_args()


def snapshot():
    processes = {}
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            if path.stat().st_uid != os.getuid():
                continue
            fields = (path / 'stat').read_text().rsplit(')', 1)[1].split()
            if fields[0] == 'Z':
                continue
            argv = (path / 'cmdline').read_bytes().decode(errors='replace').split('\0')[:-1]
            env = (path / 'environ').read_bytes().split(b'\0')
            tagged = f'RBS_DIFF_SESSION={SCOPE}'.encode() in env
            root = any(argv[i:i + 3] == ['launch', package, launch]
                       for package, launch in LAUNCHES for i in range(len(argv)))
            processes[int(path.name)] = (int(fields[1]), fields[19], argv, tagged or root)
        except (OSError, ValueError):
            continue
    return processes


# Never signal the stop command, its invoking shell, or any ancestor.
initial = snapshot()
excluded = {os.getpid()}
parent = os.getppid()
while parent in initial and parent not in excluded:
    excluded.add(parent)
    parent = initial[parent][0]
known = {}


def collect():
    processes = snapshot()
    selected = {pid for pid, data in processes.items()
                if pid not in excluded and (data[3] or known.get(pid) == data[1])}
    while True:
        children = {pid for pid, data in processes.items()
                    if pid not in excluded and data[0] in selected}
        if children <= selected:
            break
        selected |= children
    for pid in selected:
        known[pid] = processes[pid][1]
    return {pid: processes[pid] for pid in selected}


def send(processes, sig):
    for pid, data in processes.items():
        try:
            fd = os.pidfd_open(pid)
            try:
                fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
                if fields[19] == data[1]:
                    signal.pidfd_send_signal(fd, sig)
            finally:
                os.close(fd)
        except ProcessLookupError:
            pass
        except OSError as error:
            print(f'Cannot signal {pid}: {error}', flush=True)


remaining = collect()
for pid, data in sorted(remaining.items()):
    print(f'{pid}: {" ".join(data[2])[:200]}', flush=True)
if options.dry_run:
    print(f'Dry run: {len(remaining)} {SCOPE} processes; no signals sent.')
    raise SystemExit(0)

stages = [(signal.SIGKILL, 2)] if options.force else [
    (signal.SIGINT, 5), (signal.SIGTERM, 2), (signal.SIGKILL, 2)]
for sig, timeout in stages:
    if not remaining:
        break
    print(f'{sig.name}: {len(remaining)} {SCOPE} processes', flush=True)
    sent = set()
    deadline = time.monotonic() + timeout
    while remaining and time.monotonic() < deadline:
        fresh = {pid: data for pid, data in remaining.items() if (pid, data[1]) not in sent}
        send(fresh, sig)
        sent.update((pid, data[1]) for pid, data in fresh.items())
        time.sleep(0.1)
        remaining = collect()
if remaining:
    print(f'ERROR: remaining {SCOPE} PIDs: {sorted(remaining)}')
    raise SystemExit(1)
print(f'---- {SCOPE} shutdown done ! ------')
PYTHON
