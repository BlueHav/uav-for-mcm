"""Live-position map display: 100 m dense and 1 km sparse; not radar measurements."""
import json
import threading
from pathlib import Path
from portable_paths import PROJECT
import numpy as np
import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from display_sampling import world_height
from terrain_guidance import terrain_direction
from group_display_grid import DENSE_RADIUS_M, DISPLAY_RADIUS_M, group_display_grid
from point_mission_core import PointMission


class TerrainDisplay:
    def __init__(self):
        rospy.init_node('terrain_overview_display')
        root = Path(rospy.get_param('~root', str(PROJECT)))
        model = PointMission(rospy.get_param('~point', str(root/'config/point.json')))
        self.config = model.config
        self.world = model.world
        self.states = {}; self.lock = threading.RLock()
        self.near = rospy.Publisher('/sensors/raw/swarm10/display/terrain_near', PointCloud2, queue_size=1, latch=True)
        self.far = rospy.Publisher('/sensors/raw/swarm10/display/terrain_far', PointCloud2, queue_size=1, latch=True)
        self.status = rospy.Publisher('/sensors/raw/swarm10/display/status', String, queue_size=1, latch=True)
        self.guidance = [rospy.Publisher('/swarm10/group_%d/terrain_guidance'%g, String, queue_size=1, latch=True) for g in range(2)]
        for i in range(10):
            rospy.Subscriber('/drone_%d/mission/status'%i, String, self.state, callback_args=i, queue_size=1)
        rospy.Timer(rospy.Duration(3.), self.tick)

    def state(self, msg, i):
        try:
            s = json.loads(msg.data)
            p = np.asarray(s['position'], float)
            if p.shape == (3,) and np.isfinite(p).all():
                with self.lock: self.states[i] = (float(s['stamp']), p)
        except (KeyError, ValueError, TypeError): pass

    def tick(self, _):
        now = rospy.Time.now().to_sec()
        with self.lock: current = {i:p for i,(stamp,p) in self.states.items() if -.1 <= now-stamp <= 2.}
        leader_ids = [i for i in (2,7) if i in current]
        if not leader_ids:
            header = Header(stamp=rospy.Time.now(), frame_id='world')
            self.near.publish(point_cloud2.create_cloud_xyz32(header, []))
            self.far.publish(point_cloud2.create_cloud_xyz32(header, []))
            self.status.publish(String(json.dumps(dict(stamp=now,available=False,reason='No fresh group leader states',source_kind='map_terrain_sample_not_radar',display_only=True))))
            return
        focus = np.array([current[i][:2] for i in leader_ids])
        source = 'live_group_leaders'
        xy, dense = group_display_grid(focus)
        z = world_height(self.world, xy)
        # This optional advisory is explicitly disconnected from control.
        context = np.c_[xy[dense & np.isfinite(z)], z[dense & np.isfinite(z)]]
        for g,pub in enumerate(self.guidance):
            if 2+5*g not in current:
                hint = dict(available=False, reason='actual group states missing/stale')
            else:
                center = current[2+5*g]
                goal = self.config['points']['landing']
                hint = terrain_direction(center, [goal['x'],goal['y'],self.config['cruise_altitude']], context,
                                         1 if g==0 else -1, cruise_z=self.config['cruise_altitude'])
            hint.update(stamp=now, group=g, leader_id=2+5*g, advisory_only=True, controller_connected=False, goal_reference='landing')
            pub.publish(String(json.dumps(hint)))
        valid = np.isfinite(z)
        points = np.c_[xy, z].astype(np.float32)
        header = Header(stamp=rospy.Time.now(), frame_id='world')
        self.near.publish(point_cloud2.create_cloud_xyz32(header, points[dense & valid]))
        self.far.publish(point_cloud2.create_cloud_xyz32(header, points[~dense & valid]))
        self.status.publish(String(json.dumps(dict(stamp=now,center_source=source, center=focus.mean(0).tolist(),
            diameter_m=2*DISPLAY_RADIUS_M, dense_radius_m=DENSE_RADIUS_M, sparse_radius_m=DISPLAY_RADIUS_M, near_step_m=2, far_step_m=20,
            source_kind="map_terrain_sample_not_radar", leader_ids=leader_ids,
            near_points=int(np.sum(dense & valid)), far_points=int(np.sum(~dense & valid)),
            missing_dem_points=int(np.sum(~valid)), display_only=True))))


if __name__ == '__main__':
    TerrainDisplay()
    rospy.spin()
