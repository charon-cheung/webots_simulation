"""Exercise cleanup with isolated fake processes; never target real ROS sessions."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import uuid

import pytest


@pytest.mark.parametrize('force', [False, True])
def test_shutdown_scope_and_resistant_descendant(force):
    script = (Path(__file__).resolve().parents[1] / 'kill_sim.sh').read_text()
    code = script.split("<<'PYTHON'\n", 1)[1].rsplit('PYTHON', 1)[0]
    scope = 'shutdown-test-' + uuid.uuid4().hex
    # Override selection only in this test interpreter, keeping production signal logic.
    code = code.replace('SCOPE = "simulation"', f'SCOPE = {scope!r}')
    code = '\n'.join('LAUNCHES = []' if line.startswith('LAUNCHES = ') else line
                     for line in code.splitlines())
    worker = "import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); signal.signal(signal.SIGTERM,signal.SIG_IGN); print('ready',flush=True); time.sleep(60)"
    parent = """import os, subprocess, sys, time
child_env = os.environ.copy()
child_env.pop('RBS_DIFF_SESSION', None)
child = subprocess.Popen([sys.executable, '-c', sys.argv[1]], env=child_env, stdout=subprocess.PIPE, text=True)
child.stdout.readline()
print(child.pid, flush=True)
time.sleep(60)
"""
    env = dict(os.environ, RBS_DIFF_SESSION=scope)
    owned = subprocess.Popen([sys.executable, '-c', parent, worker], env=env,
                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    other = subprocess.Popen([sys.executable, '-c', worker],
                             env=dict(os.environ, RBS_DIFF_SESSION=scope + '-other'),
                             stdout=subprocess.PIPE, text=True)
    child_pid = int(owned.stdout.readline())
    other.stdout.readline()
    def run(*args):
        return subprocess.run([sys.executable, '-c', code, *args], text=True,
                              capture_output=True, timeout=15, check=True)
    try:
        preview = run('-n').stdout
        assert f'{owned.pid}:' in preview and f'{child_pid}:' in preview
        assert f'{other.pid}:' not in preview
        assert owned.poll() is None and other.poll() is None
        result = run(*(['-9'] if force else [])).stdout
        owned.wait(timeout=2)
        assert 'SIGKILL' in result  # Child ignores INT/TERM; escalation must work.
        stat = Path(f'/proc/{child_pid}/stat')
        assert not stat.exists() or stat.read_text().rsplit(')', 1)[1].split()[0] == 'Z'
        assert other.poll() is None
        assert '0 ' + scope + ' processes' in run('-n').stdout
        assert 'shutdown done' in run().stdout  # Idempotent when already stopped.
    finally:
        for process in (owned, other):
            if process.poll() is None:
                process.kill()
            process.wait(timeout=2)
            process.stdout.close()
        try:
            os.kill(child_pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
