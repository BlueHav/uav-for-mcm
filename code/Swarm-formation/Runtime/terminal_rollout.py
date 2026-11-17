"""Process-worker entry: no ROS callbacks, no command ownership."""
from types import SimpleNamespace
import numpy as np
from terminal_coordinator import TerminalCoordinator


def build_rollout(job):
    helper=TerminalCoordinator.__new__(TerminalCoordinator)
    helper.trees=job.get('trees',np.empty((0,5)))
    slots=np.asarray(job['landing_slots'],float)
    helper.model=SimpleNamespace(ends=np.asarray(job['ends']),landing_slots=lambda group:slots[group])
    return helper.build(job['group'],job['states'],job['reference'],job['run'],job['serial'],
                        job['stamp'],job['reserved'],job['cursor'])
