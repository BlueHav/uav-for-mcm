#!/usr/bin/env python3
"""按 0.1 秒间隔记录并发布最近 100 个位置。"""
from collections import deque
from copy import deepcopy

import rospy
import rospkg
from threading import RLock
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker


class OdomPaths:
    def __init__(self):
        self.lock = RLock()
        self.subscribers = []
        self.mesh_pub = rospy.Publisher('character_0/mesh', Marker, queue_size=1)
        self.mesh = Marker()
        self.mesh.header.frame_id = 'world'
        self.mesh.ns = 'mesh'
        self.mesh.type = Marker.MESH_RESOURCE
        self.mesh.mesh_resource = 'file://' + rospkg.RosPack().get_path('so3_quadrotor_simulator') + '/config/uav.dae'
        self.mesh.scale.x = self.mesh.scale.y = self.mesh.scale.z = 2.0
        self.mesh.color.r = self.mesh.color.a = 1.0
        self.history = {name: deque(maxlen=100) for name in ('character_0', 'drone_0')}
        self.last_stamp = {}
        self.publishers_by_name = {}
        for name in self.history:
            self.publishers_by_name[name] = rospy.Publisher(f'{name}/path', Path, queue_size=1)
            self.subscribers.append(rospy.Subscriber(f'{name}/odom', Odometry,
                lambda msg, name=name: self.on_odom(name, msg), queue_size=1))
        self.subscribers.append(rospy.Subscriber('/respawn', Bool, self.reset, queue_size=1))

    def reset(self, msg):
        with self.lock:
            if msg.data:
                self.last_stamp.clear()
                for name, history in self.history.items():
                    history.clear()
                    path = Path()
                    path.header.frame_id = 'world'
                    path.header.stamp = rospy.Time.now()
                    self.publishers_by_name[name].publish(path)

    def on_odom(self, name, msg):
        with self.lock:
            if name == 'character_0':
                self.mesh.header.stamp = deepcopy(msg.header.stamp)
                self.mesh.pose = deepcopy(msg.pose.pose)
                self.mesh.pose.position.z += 1.0
                self.mesh_pub.publish(self.mesh)
            stamp = msg.header.stamp.to_nsec()
            previous = self.last_stamp.get(name)
            if previous is not None:
                if stamp < previous:
                    self.history[name].clear()
                elif stamp - previous < 100_000_000:
                    return
            self.last_stamp[name] = stamp
            pose = PoseStamped()
            pose.header.stamp = deepcopy(msg.header.stamp)
            pose.header.frame_id = 'world'
            pose.pose = deepcopy(msg.pose.pose)
            if name == 'character_0':
                pose.pose.position.z += 1.0
            self.history[name].append(pose)
            path = Path()
            path.header = deepcopy(pose.header)
            path.poses = list(self.history[name])
            self.publishers_by_name[name].publish(path)


def main():
    rospy.init_node('odom_paths')
    node = OdomPaths()
    rospy.spin()

if __name__ == '__main__':
    main()
