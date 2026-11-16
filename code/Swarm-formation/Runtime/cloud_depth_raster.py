"""Lazy server-only wrapper for depth-only triangle rasterization.

Build on the server only:
g++ -O3 -std=c++11 -shared -fPIC cloud_depth_raster.cpp -o cloud_depth_raster_native.so
"""
import ctypes
from pathlib import Path
import weakref

import numpy as np


def _library():
    # A distinct basename prevents Python treating the C ABI library as this
    # Python module's extension-module implementation.
    path = Path(__file__).with_name('cloud_depth_raster_native.so')
    library = ctypes.CDLL(str(path))
    library.cloud_depth_raster_abi.argtypes = []
    library.cloud_depth_raster_abi.restype = ctypes.c_int
    if library.cloud_depth_raster_abi() != 1:
        raise RuntimeError('cloud depth raster library ABI mismatch')
    pointer = ctypes.c_void_p
    library.cloud_depth_raster_create.argtypes = [pointer, ctypes.c_int64, pointer, ctypes.c_int64]
    library.cloud_depth_raster_create.restype = pointer
    library.cloud_depth_raster_destroy.argtypes = [pointer]
    library.cloud_depth_raster_destroy.restype = None
    library.cloud_depth_raster_render.argtypes = [pointer, pointer, pointer, ctypes.c_int, ctypes.c_int] + [ctypes.c_double] * 6 + [pointer]
    library.cloud_depth_raster_render.restype = ctypes.c_int
    return library


class RasterScene:
    def __init__(self, vertices, triangles):
        vertices = np.asarray(vertices, dtype=np.float32)
        indices = np.asarray(triangles)
        if (vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) < 3
                or len(vertices) > 10000000 or not np.isfinite(vertices).all()
                or indices.ndim != 2 or indices.shape[1] != 3 or len(indices) < 1
                or len(indices) > 20000000 or not np.issubdtype(indices.dtype, np.integer)
                or np.any(indices < 0) or np.any(indices >= len(vertices))):
            raise ValueError('finite Nx3 vertices and valid integer Mx3 triangle indices required')
        vertices = np.ascontiguousarray(vertices)
        indices = np.ascontiguousarray(indices, dtype=np.uint32)
        self._library = _library()
        self._pointer = self._library.cloud_depth_raster_create(vertices.ctypes.data, len(vertices), indices.ctypes.data, len(indices))
        if not self._pointer:
            raise RuntimeError('native raster mesh creation failed')
        self._finalizer = weakref.finalize(self, self._library.cloud_depth_raster_destroy, self._pointer)

    def render(self, position, rotation, width, height, fx, fy, cx, cy, near_clip=.05, far_clip=80.):
        from cloud_depth_geometry import _rotation, _vector
        position = np.ascontiguousarray(_vector(position, 'position'))
        rotation = np.ascontiguousarray(_rotation(rotation))
        if (not isinstance(width, (int, np.integer)) or not isinstance(height, (int, np.integer))
                or not 1 <= width <= 8192 or not 1 <= height <= 8192
                or not np.isfinite([fx, fy, cx, cy, near_clip, far_clip]).all()
                or min(fx, fy) <= 0 or not 0 <= cx <= width or not 0 <= cy <= height
                or not 0 < near_clip < far_clip):
            raise ValueError('invalid raster image dimensions, intrinsics, or clip planes')
        # A new output for each camera retains paired front/up/down frames safely.
        output = np.empty((height, width), dtype=np.float32)
        status = self._library.cloud_depth_raster_render(self._pointer, position.ctypes.data, rotation.ctypes.data,
            int(width), int(height), fx, fy, cx, cy, near_clip, far_clip, output.ctypes.data)
        if status:
            raise RuntimeError('native depth rasterization rejected inputs: %d' % status)
        return output
