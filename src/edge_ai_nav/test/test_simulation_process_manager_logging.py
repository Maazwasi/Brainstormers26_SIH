"""Regression check for ROS log-severity call-site stability."""
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

from edge_ai_nav.visualization.simulation_process_manager import SimulationProcessManager


class StrictCallSiteLogger:
    """Approximate rclpy's one-severity-per-call-site restriction."""
    def __init__(self):
        self.entries=[]

    def info(self,message):
        self.entries.append(('INFO',message))

    def error(self,message):
        self.entries.append(('ERROR',message))


def test_supervisor_can_log_info_then_error_without_dynamic_method_dispatch():
    manager=SimulationProcessManager.__new__(SimulationProcessManager)
    manager.logger=StrictCallSiteLogger()
    manager._log('launching SWARMX')
    manager._log('launch failed',error=True)
    assert manager.logger.entries==[
        ('INFO','[SIM] launching SWARMX'),
        ('ERROR','[SIM] launch failed')]
