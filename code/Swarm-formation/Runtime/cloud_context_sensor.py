"""Bounded immutable terrain crops, selected by each aircraft's latest pose."""
from collections import OrderedDict
import numpy as np
from cloud_config import CONTEXT_SPAN,CONTEXT_RESOLUTION
from cloud_context import context_cloud

class TerrainContextCache:
    def __init__(self,world,slope,span=CONTEXT_SPAN,resolution=CONTEXT_RESOLUTION,capacity=10):
        if capacity<1:raise ValueError('terrain cache capacity must be positive')
        self.world=world;self.slope=slope;self.span=span;self.resolution=resolution;self.capacity=capacity
        self.entries=OrderedDict()

    def sample(self,position):
        position=np.asarray(position,float)
        if position.shape!=(3,) or not np.isfinite(position).all():raise ValueError('invalid terrain sensor pose')
        # context_cloud uses this exact 2 m grid quantization. Geometry and its
        # unknown boundaries are invariant until the grid center changes.
        center=tuple(np.round(position[:2]/self.resolution)*self.resolution)
        if center in self.entries:
            self.entries.move_to_end(center);return self.entries[center]
        points=context_cloud(self.world,position,self.slope,span=self.span,resolution=self.resolution)
        points.setflags(write=False);self.entries[center]=points
        while len(self.entries)>self.capacity:self.entries.popitem(last=False)
        return points

def context_observations(states,cache,clock,count=10,ids=None):
    """Do not snapshot all poses before earlier members' expensive crops.

    A cached surface never refreshes the pose time. Both checks use the actual
    sampled pose header, including a second check after a cache miss finishes.
    Only the requested identities (the group leaders) are sampled; every other
    aircraft reuses its leader's crop through sensor_channels.group_context_topic.
    """
    for identity in (range(count) if ids is None else ids):
        state=states.get(identity)
        if state is None:continue
        stamp,position=state
        if not np.isfinite(stamp) or not -.1<=clock()-stamp<=.3:continue
        position=np.asarray(position,float)
        if position.shape!=(3,) or not np.isfinite(position).all():continue
        points=cache.sample(position)
        if not -.1<=clock()-stamp<=.3:continue
        yield identity,stamp,points
