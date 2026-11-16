"""Limit owned-process parallelism to the CPU quota, with control isolation."""
import os
from pathlib import Path


def choose_layout(allowed,cpu_max,topology):
    quota,period=cpu_max.split()
    if quota=='max':return None
    capacity=int(quota)//int(period)
    # Leave one quota CPU for ROS launch/monitoring and unrelated container work.
    budget=capacity-1
    if budget<7:return None
    physical={}
    for cpu in sorted(allowed):physical.setdefault(topology(cpu),cpu)
    selected=list(physical.values())[:budget]
    if len(selected)<7:return None
    return dict(control=selected[:3],planning=selected[3:],quota_cores=capacity)


def detect_layout():
    def topology(cpu):
        root=Path('/sys/devices/system/cpu/cpu%d/topology'%cpu)
        return ((root/'physical_package_id').read_text().strip(),(root/'core_id').read_text().strip())
    try:return choose_layout(os.sched_getaffinity(0),Path('/sys/fs/cgroup/cpu.max').read_text(),topology)
    except (AttributeError,FileNotFoundError):return None


def child_command(name,command,layout):
    args=['/bin/bash','-c',command]
    if layout is None:return args
    role='control' if name=='controller' or name.startswith('executor') else 'planning'
    return ['taskset','-c',','.join(str(c) for c in layout[role])]+args
