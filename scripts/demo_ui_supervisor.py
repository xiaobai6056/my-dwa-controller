"""Own one Linux demo session and reap its ROS and Gazebo processes."""

import ctypes
import os
import signal
import socket
import subprocess
import sys
from contextlib import ExitStack
from pathlib import Path
from select import select
from shlex import join
from tempfile import TemporaryDirectory
from time import monotonic, sleep
from urllib.parse import urlsplit


PR_SET_PDEATHSIG = 1
PR_SET_CHILD_SUBREAPER = 36
STOP_SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
MOTION_SCRIPTS = ('v01_scenario.py', 'multi_motion.py')
START_FILE_ENV = 'MYDWA_DEMO_START_FILE'
libc = ctypes.CDLL(None, use_errno=True)


def process_control(option, value):
    """Apply one Linux prctl option."""
    if libc.prctl(option, value, 0, 0, 0) == -1:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error))


def protect_launch(supervisor_pid):
    """Let launch shut down if its supervisor unexpectedly disappears."""
    process_control(PR_SET_PDEATHSIG, signal.SIGINT)
    if os.getppid() != supervisor_pid:
        os._exit(130)


def reserve_endpoint(stack, key, label):
    """Reserve a demo endpoint until its supervisor finishes cleanup."""
    lock = stack.enter_context(socket.socket(socket.AF_UNIX))
    try:
        lock.bind(f'\0mydwa-ui.{os.getuid()}.{key}')
    except OSError as exc:
        raise RuntimeError(f'{label} is already in use by another MyDWA demo. '
                           'Wait for it to finish shutting down.') from exc


def check_gazebo(host, port):
    """Fail early if a Gazebo master already owns this address."""
    try:
        connection = socket.create_connection((host, port), timeout=.5)
    except ConnectionRefusedError:
        return
    except (OSError, TimeoutError) as exc:
        raise RuntimeError(f'Cannot check Gazebo master at {host}:{port}: {exc}') from exc
    connection.close()
    raise RuntimeError(f'Gazebo master {host}:{port} is already in use. '
                       'Close that simulation or choose another GAZEBO_MASTER_URI.')


def signal_group(group, signum):
    """Signal only the process group created specifically for this launch."""
    try:
        os.killpg(group, signum)
    except ProcessLookupError:
        pass


def adopted_children():
    """Return direct children that still need to exit or be reaped."""
    path = Path('/proc/thread-self/children')
    return [int(pid) for pid in path.read_text().split()]


def finish_children(group, signum, timeout):
    """Reap descendants of launch, including children adopted on launch exit."""
    deadline = monotonic() + timeout
    while True:
        try:
            pid, _ = os.waitpid(-1, os.WNOHANG)
        except ChildProcessError:
            return True
        if pid:
            continue
        signal_group(group, signum)
        # A descendant may have set its own PGID before launch died.
        for child in adopted_children():
            try:
                os.kill(child, signum)
            except ProcessLookupError:
                pass
        if monotonic() >= deadline:
            return False
        sleep(.05)


def run_launch(command, start_file, child_env):
    """Run one launch until interrupted, keeping cleanup inside this parent."""
    process_control(PR_SET_CHILD_SUBREAPER, 1)
    requested = None

    def stop(signum, _):
        nonlocal requested
        if requested is None:
            requested = signum

    for signum in STOP_SIGNALS:
        signal.signal(signum, stop)

    supervisor_pid = os.getpid()
    child = subprocess.Popen(
        command,
        stdin=subprocess.DEVNULL, env=child_env, start_new_session=True,
        preexec_fn=lambda: protect_launch(supervisor_pid))
    try:
        print('Arrange the demo windows, then press Enter to start. Ctrl+C to stop.',
              flush=True)
        input_fd = sys.stdin.fileno()
        while requested is None and child.poll() is None:
            if input_fd is None:
                sleep(.05)
                continue
            ready, _, _ = select([input_fd], [], [], .05)
            if ready:
                data = os.read(input_fd, 4096)
                if b'\n' in data and requested is None:
                    start_file.touch()
                    print('Start confirmed. Waiting for the demo to be ready.', flush=True)
                    input_fd = None
                elif not data:
                    print('Terminal input closed. The demo will wait for Ctrl+C.', flush=True)
                    input_fd = None
    finally:
        if child.poll() is None:
            try:
                child.send_signal(signal.SIGINT)
                child.wait(timeout=17)
            except subprocess.TimeoutExpired:
                signal_group(child.pid, signal.SIGTERM)
                try:
                    child.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    signal_group(child.pid, signal.SIGKILL)
                    child.wait(timeout=2)
        if not finish_children(child.pid, signal.SIGTERM, 1):
            if not finish_children(child.pid, signal.SIGKILL, 2):
                raise RuntimeError('Some MyDWA processes did not exit after SIGKILL.')
    if requested is not None:
        return 128 + requested
    return child.returncode if child.returncode >= 0 else 128 - child.returncode


def wait_for_motion(command):
    """Gate only the scenario motion process and preserve its launch arguments."""
    if len(command) > 2 and command[1] == '-u' and Path(command[2]).name in MOTION_SCRIPTS:
        start_file = Path(os.environ[START_FILE_ENV])
        while not start_file.is_file():
            sleep(.05)
    os.execvp(command[0], command)


def run_recording(run, launch_args):
    """Start a recording launch and pass the confirmation to its motion process."""
    start_file = Path(run) / 'start'
    child_env = os.environ.copy()
    child_env[START_FILE_ENV] = str(start_file)
    # The capture stack comes from external launch files. Pass through other Python nodes.
    prefix = join([sys.executable, str(Path(__file__).resolve()), '--motion'])
    command = ['ros2', 'launch', '--noninteractive', '--launch-prefix', prefix,
               '--launch-prefix-filter', r'^python[0-9.]*$', *launch_args]
    return run_launch(command, start_file, child_env)


def main():
    """Check endpoint availability and start the supervised launch."""
    args = sys.argv[1:]
    if args[:1] == ['--motion']:
        wait_for_motion(args[1:])
    if args[:1] == ['--recording']:
        return run_recording(args[1], args[2:])
    path, scene = args
    master = urlsplit(os.environ['GAZEBO_MASTER_URI'])
    host, port = master.hostname, master.port or 11345
    if master.scheme != 'http' or host is None:
        raise RuntimeError('GAZEBO_MASTER_URI must be an HTTP URL with a hostname.')
    domain = int(os.environ['ROS_DOMAIN_ID'])
    with ExitStack() as stack:
        reserve_endpoint(stack, f'ros.{domain}', f'ROS domain {domain}')
        reserve_endpoint(stack, f'gazebo.{port}', f'Gazebo port {port}')
        check_gazebo(host, port)
        with TemporaryDirectory(prefix='mydwa-ui.') as state:
            command = ['ros2', 'launch', '--noninteractive', path, f'scene:={scene}',
                       f'run:={state}', 'wait_for_input:=true']
            child_env = dict(os.environ, TMPDIR=state, ROS_LOG_DIR=f'{state}/ros',
                             GAZEBO_LOG_PATH=f'{state}/gazebo',
                             YOLO_CONFIG_DIR=f'{state}/ultralytics')
            print(f'MyDWA {scene}', flush=True)
            return run_launch(command, Path(state) / 'start', child_env)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as exc:
        print(f'MyDWA demo: {exc}', file=sys.stderr)
        sys.exit(1)
