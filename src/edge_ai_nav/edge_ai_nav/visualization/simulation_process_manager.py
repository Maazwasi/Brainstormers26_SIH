"""Own and supervise one dashboard-launched SWARMX simulation process group."""
from collections import deque
import os
import signal
import subprocess
import threading
import time


class SimulationProcessManager:
    STATES = {'STOPPED', 'STARTING', 'RUNNING', 'STOPPING', 'ERROR', 'EXTERNAL'}

    def __init__(self, command, workspace, readiness_probe, logger,
                 startup_timeout=75.0, detect_external=True):
        self.command = tuple(command)
        self.workspace = workspace
        self.readiness_probe = readiness_probe
        self.logger = logger
        self.startup_timeout = float(startup_timeout)
        self.lock = threading.RLock()
        self.process = None
        self.state = 'EXTERNAL' if detect_external and self._external_running() else 'STOPPED'
        self.started_at = None
        self.last_error = None
        self.last_logs = deque(maxlen=40)
        self.transition = None

    @staticmethod
    def _all_cmdlines():
        for entry in os.scandir('/proc'):
            if not entry.name.isdigit():
                continue
            try:
                with open(f'/proc/{entry.name}/cmdline', 'rb') as stream:
                    value = stream.read().replace(b'\0', b' ').decode(errors='replace')
                if value:
                    yield int(entry.name), value
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue

    def _external_running(self):
        return any('gz-sim-main' in command and
                   'warehouse_sih_v2_large.sdf' in command
                   for _, command in self._all_cmdlines())

    def _group_has(self, text):
        process = self.process
        if process is None:
            return False
        pgid = process.pid
        for pid, command in self._all_cmdlines():
            try:
                if os.getpgid(pid) == pgid and text in command:
                    return True
            except (PermissionError, ProcessLookupError):
                continue
        return False

    def _ready(self):
        return (self._group_has('warehouse_sih_v2_large.sdf') and
                self._group_has('local_waypoint_controller') and
                bool(self.readiness_probe()))

    def _log(self, message, error=False):
        line = f'[SIM] {message}'
        (self.logger.error if error else self.logger.info)(line)

    def _set_error(self, message):
        with self.lock:
            self.state = 'ERROR'
            self.last_error = str(message)[:240]
        self._log(f'ERROR: {self.last_error}', error=True)

    def _read_output(self, process):
        try:
            for line in process.stdout:
                clean = line.rstrip()
                if clean:
                    self.last_logs.append(clean[-500:])
        except (ValueError, OSError):
            pass

    def _watch_start(self, process):
        deadline = time.monotonic() + self.startup_timeout
        while time.monotonic() < deadline:
            code = process.poll()
            if code is not None:
                tail = self.last_logs[-1] if self.last_logs else f'exit code {code}'
                return self._set_error(f'launch exited before ready: {tail}')
            if self._ready():
                with self.lock:
                    if self.process is process and self.state == 'STARTING':
                        self.state = 'RUNNING'
                        self.last_error = None
                self._log('RUNNING')
                break
            time.sleep(0.5)
        else:
            self._terminate(process)
            return self._set_error('startup readiness timeout')

        code = process.wait()
        with self.lock:
            expected = self.state in ('STOPPING', 'STOPPED')
            if self.process is process:
                self.process = None
            if not expected:
                self.state = 'ERROR'
                self.last_error = f'launch exited unexpectedly (code {code})'
        if not expected:
            self._log(f'ERROR: launch exited unexpectedly (code {code})', error=True)

    def _launch(self):
        environment = os.environ.copy()
        environment['ROS_LOG_DIR'] = '/tmp/swarmx_dashboard_launch_logs'
        os.makedirs(environment['ROS_LOG_DIR'], exist_ok=True)
        self._log('launching SWARMX')
        try:
            process = subprocess.Popen(
                self.command, cwd=self.workspace, env=environment,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, start_new_session=True)
        except OSError as error:
            self._set_error(f'cannot start launch: {error}')
            return
        with self.lock:
            self.process = process
            self.started_at = time.strftime('%Y-%m-%dT%H:%M:%S%z')
            self.last_error = None
        self._log(f'PID={process.pid}')
        threading.Thread(target=self._read_output, args=(process,), daemon=True).start()
        threading.Thread(target=self._watch_start, args=(process,), daemon=True).start()

    def start(self):
        self._log('START requested')
        with self.lock:
            self._reconcile_locked()
            if self.state in ('STARTING', 'RUNNING'):
                return self.status()
            if self.state == 'STOPPING':
                return self.status()
            if self.state == 'EXTERNAL' or (self.process is None and self._external_running()):
                self.state = 'EXTERNAL'
                self.last_error = 'SWARMX is running outside dashboard ownership'
                return self.status()
            self.state = 'STARTING'
            self.last_error = None
            self.last_logs.clear()
            self._launch()
            return self.status()

    @staticmethod
    def _signal_group(process, sig):
        try:
            os.killpg(process.pid, sig)
            return True
        except ProcessLookupError:
            return False

    def _terminate(self, process):
        if process is None or process.poll() is not None:
            return
        self._signal_group(process, signal.SIGINT)
        self._log('SIGINT sent')
        try:
            process.wait(timeout=8.0)
            return
        except subprocess.TimeoutExpired:
            pass
        self._signal_group(process, signal.SIGTERM)
        self._log('SIGTERM sent')
        try:
            process.wait(timeout=4.0)
            return
        except subprocess.TimeoutExpired:
            pass
        self._signal_group(process, signal.SIGKILL)
        self._log('SIGKILL sent')
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            self._log('process group did not exit after SIGKILL', error=True)

    def _finish_stop(self, process, restart=False):
        self._terminate(process)
        with self.lock:
            if self.process is process:
                self.process = None
            self.state = 'STOPPED'
            self.started_at = None
            self.last_error = None
        self._log('STOPPED')
        if restart:
            time.sleep(0.75)
            self.start()

    def stop(self):
        self._log('STOP requested')
        with self.lock:
            self._reconcile_locked()
            if self.state == 'EXTERNAL':
                self.last_error = 'Cannot stop a simulation not started by this dashboard'
                return self.status()
            if self.state in ('STOPPED', 'STOPPING') or self.process is None:
                self.state = 'STOPPED' if self.process is None else self.state
                return self.status()
            self.state = 'STOPPING'
            process = self.process
        threading.Thread(target=self._finish_stop, args=(process,), daemon=True).start()
        return self.status()

    def restart(self):
        self._log('RESTART requested')
        with self.lock:
            self._reconcile_locked()
            if self.state == 'EXTERNAL':
                self.last_error = 'Cannot restart a simulation not started by this dashboard'
                return self.status()
            if self.state in ('STARTING', 'STOPPING'):
                return self.status()
            if self.process is None:
                return self.start()
            self.state = 'STOPPING'
            process = self.process
        threading.Thread(target=self._finish_stop, args=(process, True), daemon=True).start()
        return self.status()

    def _reconcile_locked(self):
        if self.process is not None and self.process.poll() is not None:
            code = self.process.returncode
            was_stopping = self.state == 'STOPPING'
            self.process = None
            self.state = 'STOPPED' if was_stopping else 'ERROR'
            if not was_stopping:
                self.last_error = f'launch exited unexpectedly (code {code})'
        elif self.process is None and self.state == 'EXTERNAL' and not self._external_running():
            self.state = 'STOPPED'
            self.last_error = None

    def status(self):
        with self.lock:
            self._reconcile_locked()
            return {
                'state': self.state,
                'pid': self.process.pid if self.process is not None else None,
                'started_at': self.started_at,
                'last_error': self.last_error,
                'last_logs': list(self.last_logs)[-12:],
            }
