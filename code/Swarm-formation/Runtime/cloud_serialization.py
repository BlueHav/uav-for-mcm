"""Vectorized, wire-identical replacement for ROS create_cloud_xyz32.

Avoid per-point Python struct packing in display processes. The immutable
payload owns a copy; neither caller mutation nor asynchronous ROS publication
can change an already constructed message.
"""
import numpy as np
from sensor_msgs.msg import PointCloud2,PointField


def xyz32(header,points):
    points=np.asarray(points,dtype='<f4')
    if points.shape==(0,):points=points.reshape(0,3)
    if points.ndim!=2 or points.shape[1]!=3:raise ValueError('Expected Nx3 points')
    fields=[PointField('x',0,PointField.FLOAT32,1),
            PointField('y',4,PointField.FLOAT32,1),
            PointField('z',8,PointField.FLOAT32,1)]
    return PointCloud2(header=header,height=1,width=len(points),fields=fields,
                       is_bigendian=False,point_step=12,row_step=12*len(points),
                       data=points.tobytes(order='C'),is_dense=False)
