"""CPU-only, view-limited synthetic depth from the versioned world height field.

Terrain triangles and building boxes follow HeightFieldMesh/WorldGeometry.cs.
Trees use conservative analytic cylinders/ellipsoids (Unity's faceted primitive
meshes lie inside them). Roads/model assets require an exported geometry cache
and are rejected instead of silently disappearing. Layer-10 pads are excluded.

Open3D is imported only when a scene is built. Runtime execution belongs on the
server. Each worker owns one immutable acceleration structure, bounded threads,
and a reusable camera ray cache. No global map is returned to the planner.
"""
import numpy as np
from types import MappingProxyType

MAX_DEPTH = 80.
NEAR_CLIP = .05
UNKNOWN_DEPTH = .04  # RollingLocalMap ignores <= .05, preserving unknown voxels.
# All vertices of Assets/SourceModels/TrackerDroneMesh.json, FLU=(z,-x,y).
# Original source_sha256: b4bdfc3ef148fbd2282bb7496599ed907fb472318d3b78f7d22168c62eb09635.
# Centered symmetric bounds deliberately enclose the asymmetric vertical mesh.
PEER_HALF_EXTENTS = np.array([.2463711053133011, .24637119472026825,
                              .0970769003033638], dtype=np.float32)
_UP = np.array([[0., 0., -1.], [0., 1., 0.], [1., 0., 0.]], dtype=np.float32)
_DOWN = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]], dtype=np.float32)


def _vector(value, name, size=3):
    result = np.asarray(value, dtype=np.float32)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError('%s must be a finite %d-vector' % (name, size))
    return result


def _rotation(value):
    result = np.asarray(value, dtype=np.float32)
    if (result.shape != (3, 3) or not np.isfinite(result).all()
            or not np.allclose(result.T @ result, np.eye(3), atol=2e-4, rtol=0.)
            or not np.isclose(np.linalg.det(result), 1., atol=2e-4, rtol=0.)):
        raise ValueError('rotation must be a finite proper orthonormal 3x3 matrix')
    return result


class PreparedPeers(tuple):
    """One owned, validated pose snapshot shared by a synchronous triplet."""
    def __new__(cls,peers):
        prepared=[]
        for peer in peers:
            center=_vector(peer['position'],'peer position').copy()
            rotation=_rotation(peer['rotation']).copy()
            half=_vector(peer.get('half_extents',PEER_HALF_EXTENTS),'peer half_extents').copy()
            if np.any(half<=0.):raise ValueError('peer half extents must be positive')
            for value in (center,rotation,half):value.flags.writeable=False
            prepared.append(MappingProxyType(dict(position=center,rotation=rotation,half_extents=half)))
        return super().__new__(cls,prepared)


def camera_rotation(rotation, camera):
    """FLU camera-to-world rotation; vertical cameras do not inherit body tilt."""
    rotation = _rotation(rotation)
    if camera == 'front':
        return rotation.copy()
    if camera == 'up':
        return _UP.copy()
    if camera == 'down':
        return _DOWN.copy()
    raise ValueError('camera must be front, up, or down')


def _number(obj, key, default=None):
    value = float(obj[key] if default is None else obj.get(key, default))
    if not np.isfinite(value):
        raise ValueError('%s must be finite' % key)
    return value


def _box_mesh(center, half):
    vertices = np.array([[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
                         [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]], np.float32)
    vertices = vertices * np.asarray(half, np.float32) + np.asarray(center, np.float32)
    faces = np.array([[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
                      [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5],
                      [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]], np.uint32)
    return vertices, faces


class CloudDepthScene:
    def __init__(self, world, width=480, height=270, nthreads=1, backend='open3d'):
        self.width, self.height, self.nthreads = int(width), int(height), int(nthreads)
        if backend not in ('open3d', 'raster'):
            raise ValueError('geometry backend must be open3d or raster')
        self.backend = backend
        if min(self.width, self.height, self.nthreads) < 1 or self.nthreads > 16:
            raise ValueError('positive image dimensions and 1..16 CPU threads required')
        bounds = world['bounds']
        self.bounds = np.array([[_number(bounds, 'x_min'), _number(bounds, 'x_max')],
                                [_number(bounds, 'y_min'), _number(bounds, 'y_max')]], np.float32)
        if np.any(self.bounds[:, 0] >= self.bounds[:, 1]):
            raise ValueError('world footprint must have positive width and height')
        if world.get('roads') or world.get('plazas'):
            raise ValueError('roads/plazas require an explicit exported geometry cache')
        terrain = world.get('terrain')
        if not terrain:
            raise ValueError('a complete finite terrain height field is required')
        columns, rows = int(terrain['width']), int(terrain['height'])
        heights = np.asarray(terrain['heights'], np.float32)
        if min(columns, rows) < 2 or heights.shape != (columns * rows,) or not np.isfinite(heights).all():
            raise ValueError('invalid or missing terrain samples')
        xs = np.linspace(*self.bounds[0], columns, dtype=np.float32)
        ys = np.linspace(self.bounds[1, 1], self.bounds[1, 0], rows, dtype=np.float32)
        xx, yy = np.meshgrid(xs, ys)
        vertices = [np.column_stack((xx.ravel(), yy.ravel(), heights))]
        i = (np.arange(rows - 1)[:, None] * columns + np.arange(columns - 1)).ravel()
        # ROS->Unity is a handedness conversion. Reverse Unity's i,b,d and b,e,d
        # so Open3D normals point outwards/up in right-handed world FLU.
        triangles = [np.concatenate((np.column_stack((i, i + columns, i + 1)),
                                     np.column_stack((i + 1, i + columns, i + columns + 1)))).astype(np.uint32)]
        offset = len(vertices[0])
        self.trees = []
        self.building_count = 0

        def add_box(center, half):
            nonlocal offset
            v, t = _box_mesh(center, half)
            vertices.append(v); triangles.append(t + offset); offset += len(v)

        for obj in world.get('objects', []):
            x, y, base = (_number(obj, key, 0.) for key in ('x', 'y', 'base_z'))
            tall = _number(obj, 'height')
            if tall <= 0.:
                raise ValueError('object height must be positive')
            if obj.get('kind') == 'tree':
                radius = _number(obj, 'radius')
                if radius <= 0.:
                    raise ValueError('tree radius must be positive')
                self.trees.append((np.array([x, y, base], np.float32), tall, radius))
            else:
                if obj.get('model'):
                    raise ValueError('unsupported model geometry: %s' % obj['model'])
                if obj.get('kind') != 'building':
                    raise ValueError('unsupported world object kind: %s' % obj.get('kind'))
                wide, deep, bottom = _number(obj, 'width'), _number(obj, 'depth'), _number(obj, 'bottom_z', 0.)
                if min(wide, deep, base + tall - bottom) <= 0.:
                    raise ValueError('building extents must be positive')
                # Unity primitive buildings deliberately do not apply obj.yaw.
                add_box([x, y, (bottom + base + tall) * .5], [wide * .5, deep * .5, (base + tall - bottom) * .5])
                add_box([x, y, base + tall + .15], [wide * .5 + .2, deep * .5 + .2, .15])
                self.building_count += 1

        v = np.ascontiguousarray(np.concatenate(vertices), dtype=np.float32)
        t = np.ascontiguousarray(np.concatenate(triangles), dtype=np.uint32)
        self.triangle_count = len(t)
        if backend == 'raster':
            from cloud_depth_raster import RasterScene
            self._raster = RasterScene(v, t)
        else:
            import open3d as o3d
            self._o3d = o3d
            self._device = o3d.core.Device('CPU:0')
            self._scene = o3d.t.geometry.RaycastingScene(nthreads=self.nthreads, device=self._device)
            self._scene.add_triangles(o3d.core.Tensor(v, device=self._device), o3d.core.Tensor(t, device=self._device))
        self._rays = {}
        self._intrinsics = {}
        xx, yy = np.meshgrid(np.arange(self.width, dtype=np.float32) + .5,
                             np.arange(self.height, dtype=np.float32) + .5)
        for camera in ('front', 'up', 'down'):
            focal = 240. if camera == 'front' else 77.94228634059948
            fx, fy = focal * self.width / 480., focal * self.height / 270.
            cx, cy = self.width * .5, self.height * .5
            self._rays[camera] = np.stack((np.ones_like(xx), -(xx - cx) / fx, -(yy - cy) / fy), axis=-1)
            self._intrinsics[camera] = (fx, fy, cx, cy)
        self.unknown_mask = np.zeros((self.height, self.width), bool)
        self.stats = {}
        self._world_rays = {
            'up': np.einsum('...j,ij->...i', self._rays['up'], _UP),
            'down': np.einsum('...j,ij->...i', self._rays['down'], _DOWN)}
        self._front_rotation = None
        # Triangle inequality bounds every rotated ray, including wide cameras.
        # Compute once; a render far inside the map needs no full-image exit scan.
        self._ray_abs_bounds = {name:np.max(np.abs(rays),axis=(0,1)).astype(np.float64)
                                for name,rays in self._rays.items()}
        if backend == 'open3d':
            self._ray_buffer = np.empty((self.width * self.height, 6), np.float32)
            # CPU Tensor.from_numpy shares this private worker-owned buffer. A render
            # completes synchronously before another camera reuses it.
            self._ray_tensor = o3d.core.Tensor.from_numpy(self._ray_buffer)

    def _static_depth(self, position, directions, rotation=None, camera=None):
        if self.backend == 'raster':
            fx, fy, cx, cy = self._intrinsics[camera]
            image = self._raster.render(position, rotation, self.width, self.height, fx, fy, cx, cy)
            return image, np.zeros(image.shape, bool)
        rays = self._ray_buffer
        directions = directions.reshape(-1, 3)
        rays[:, 3:] = directions
        rays[:, :3] = position + directions * NEAR_CLIP
        cast = self._scene.cast_rays(self._ray_tensor, nthreads=self.nthreads)
        result = cast['t_hit'].numpy()
        result += NEAR_CLIP
        front = np.einsum('ij,ij->i', directions, cast['primitive_normals'].numpy()) < -1e-7
        valid = np.isfinite(result) & (result <= MAX_DEPTH)
        active = np.flatnonzero(valid & ~front)
        unresolved = np.zeros(len(rays), bool)
        if not len(active):
            result[~valid] = np.inf
            return result.reshape(self.height, self.width), unresolved.reshape(self.height, self.width)
        advance = np.full(len(rays), NEAR_CLIP, np.float32)
        advance[active] = result[active] + .0001
        rays[active, :3] = position + directions[active] * advance[active, None]
        result[~valid | ~front] = np.inf
        # Normally one pass. A bounded retry implements Unity backface culling
        # when the camera lies inside an object or below a surface.
        for _ in range(31):
            cast = self._scene.cast_rays(self._o3d.core.Tensor.from_numpy(rays[active]), nthreads=self.nthreads)
            depth = cast['t_hit'].numpy() + advance[active]
            normals = cast['primitive_normals'].numpy()
            front = np.einsum('ij,ij->i', directions[active], normals) < -1e-7
            valid = np.isfinite(depth) & (depth <= MAX_DEPTH)
            accepted = valid & front
            result[active[accepted]] = depth[accepted]
            retry = valid & ~front
            active = active[retry]
            if not len(active):
                break
            advance[active] = depth[retry] + .0001
            rays[active, :3] = position + directions[active] * advance[active, None]
        else:
            unresolved[active] = True
        return result.reshape(self.height, self.width), unresolved.reshape(self.height, self.width)

    def _roi(self, position, cam_rotation, camera, center, rotation, half):
        signs = np.array([[-1, -1, -1], [-1, -1, 1], [-1, 1, -1], [-1, 1, 1],
                          [1, -1, -1], [1, -1, 1], [1, 1, -1], [1, 1, 1]], np.float32)
        corners = (signs * half) @ rotation.T + center
        local = (corners - position) @ cam_rotation
        if np.max(local[:, 0]) < NEAR_CLIP or np.min(local[:, 0]) > MAX_DEPTH:
            return slice(0, 0), slice(0, 0)
        fx, fy, cx, cy = self._intrinsics[camera]
        # A box crossing the near plane may still be wholly outside a side of
        # the frustum. This is common for peers beside vertical cameras; testing
        # every image ray for every such peer wastes most of the CPU budget.
        for axis, extent in ((1, cx / fx), (2, cy / fy)):
            for sign in (-1., 1.):
                if np.all(local[:, 0] * extent + sign * local[:, axis] < 0.):
                    return slice(0, 0), slice(0, 0)
        if np.min(local[:, 0]) <= NEAR_CLIP:
            return slice(0, self.height), slice(0, self.width)
        u = cx - fx * local[:, 1] / local[:, 0] - .5
        v = cy - fy * local[:, 2] / local[:, 0] - .5
        return (slice(int(np.clip(np.floor(v.min()), 0, self.height)), int(np.clip(np.ceil(v.max()) + 1, 0, self.height))),
                slice(int(np.clip(np.floor(u.min()), 0, self.width)), int(np.clip(np.ceil(u.max()) + 1, 0, self.width))))

    @staticmethod
    def _box_depth(position, directions, center, rotation, half):
        origin = (position - center) @ rotation
        local = np.einsum('...j,jk->...k', directions, rotation)
        parallel = np.abs(local) < 1e-12
        safe = np.where(parallel, 1., local)
        a, b = (-half - origin) / safe, (half - origin) / safe
        lo, hi = np.minimum(a, b), np.maximum(a, b)
        outside = parallel & (np.abs(origin) > half)
        lo = np.where(parallel, -np.inf, lo); hi = np.where(parallel, np.inf, hi)
        entry, leave = lo.max(axis=-1), hi.min(axis=-1)
        depth = np.where(entry >= NEAR_CLIP, entry, leave)
        good = ~outside.any(axis=-1) & (leave >= np.maximum(entry, NEAR_CLIP)) & (depth <= MAX_DEPTH)
        return np.where(good, depth, np.inf).astype(np.float32)

    def _tree_depth(self, position, directions, base, tall, radius):
        # Circumscribed smooth primitives intentionally never leave holes between
        # Unity's sphere/cylinder facets. Current production world has zero trees.
        center = base + [0, 0, tall * .7]
        scale = np.array([radius, radius, tall * .3])
        origin, ray = (position - center) / scale, directions / scale
        a = np.einsum('...i,...i->...', ray, ray)
        b = np.einsum('...i,i->...', ray, origin)
        c = np.dot(origin, origin) - 1.
        disc = b * b - a * c
        entry = (-b - np.sqrt(np.maximum(disc, 0.))) / a
        canopy = np.where((disc >= 0.) & (entry >= NEAR_CLIP), entry, np.inf)
        origin = position - base
        a = np.sum(directions[..., :2] ** 2, axis=-1)
        b = np.einsum('...i,i->...', directions[..., :2], origin[:2])
        disc = b * b - a * (np.dot(origin[:2], origin[:2]) - .275 ** 2)
        entry = (-b - np.sqrt(np.maximum(disc, 0.))) / np.maximum(a, 1e-30)
        z = origin[2] + entry * directions[..., 2]
        trunk = np.where((a > 1e-20) & (disc >= 0.) & (entry >= NEAR_CLIP) & (z >= 0.) & (z <= tall * .55), entry, np.inf)
        for zcap, normal in ((0., -1.), (tall * .55, 1.)):
            dz = directions[..., 2]
            entry = (zcap - origin[2]) / np.where(np.abs(dz) > 1e-12, dz, 1.)
            xy = origin[:2] + entry[..., None] * directions[..., :2]
            hit = (dz * normal < -1e-12) & (entry >= NEAR_CLIP) & (np.sum(xy * xy, axis=-1) <= .275 ** 2)
            trunk = np.minimum(trunk, np.where(hit, entry, np.inf))
        return np.minimum(canopy, trunk).astype(np.float32)

    def _view_inside_bounds(self, position, rotation, camera):
        reach = MAX_DEPTH * (np.abs(rotation[:2]).astype(np.float64) @ self._ray_abs_bounds[camera])
        p = np.asarray(position[:2],np.float64)
        # Conservative roundoff allowance for float32 rays, positions and bounds.
        # Uncertain/boundary cases always retain the original per-pixel test.
        margin = 64*np.finfo(np.float32).eps*(np.abs(p)+np.max(np.abs(self.bounds),axis=1)+reach+1.)
        return bool(np.all(p-reach-margin>self.bounds[:,0]) and np.all(p+reach+margin<self.bounds[:,1]))

    def _boundary_unknown(self, position, directions, depth):
        exit_depth = np.full(depth.shape, np.inf, np.float32)
        for axis in (0, 1):
            direction = directions[..., axis]
            bound = np.where(direction > 0., self.bounds[axis, 1], self.bounds[axis, 0])
            crossing = (bound - position[axis]) / np.where(np.abs(direction) > 1e-12, direction, 1.)
            exit_depth = np.minimum(exit_depth, np.where(np.abs(direction) > 1e-12, crossing, np.inf))
        outside = np.any((position[:2] < self.bounds[:, 0]) | (position[:2] > self.bounds[:, 1]))
        return (exit_depth < np.minimum(depth, MAX_DEPTH)) | outside

    def render(self, position, rotation, camera, peers=None):
        position = _vector(position, 'position')
        cam_rotation = camera_rotation(rotation, camera)
        if camera == 'front' and (self._front_rotation is None or not np.array_equal(cam_rotation, self._front_rotation)):
            self._world_rays['front'] = np.einsum('...j,ij->...i', self._rays['front'], cam_rotation)
            self._front_rotation = cam_rotation.copy()
        directions = self._world_rays[camera]
        depth, unresolved = self._static_depth(position, directions, cam_rotation, camera)
        for base, tall, radius in self.trees:
            center = base + [0, 0, tall * .5]
            half = np.array([max(radius, .275), max(radius, .275), tall * .5])
            roi = self._roi(position, cam_rotation, camera, center, np.eye(3), half)
            if depth[roi].size:
                depth[roi] = np.minimum(depth[roi], self._tree_depth(position, directions[roi], base, tall, radius))
        peer_mask = np.zeros(depth.shape, bool)
        peer_tested_pixels = 0
        prepared=peers if isinstance(peers,PreparedPeers) else PreparedPeers(() if peers is None else peers)
        for peer in prepared:
            center=peer['position'];peer_rotation=peer['rotation'];half=peer['half_extents']
            roi = self._roi(position, cam_rotation, camera, center, peer_rotation, half)
            if depth[roi].size:
                peer_tested_pixels += depth[roi].size
                candidate = self._box_depth(position, directions[roi], center, peer_rotation, half)
                peer_mask[roi] |= candidate < depth[roi]
                depth[roi] = np.minimum(depth[roi], candidate)
        self.unknown_mask = (unresolved if self._view_inside_bounds(position,cam_rotation,camera)
                             else unresolved | self._boundary_unknown(position,directions,depth))
        self.peer_mask = peer_mask & ~self.unknown_mask
        hit = np.isfinite(depth) & (depth <= MAX_DEPTH) & ~self.unknown_mask
        image = np.minimum(depth, MAX_DEPTH).astype(np.float32)
        image[self.unknown_mask] = UNKNOWN_DEPTH
        self.stats = {'unknown_pixels': int(self.unknown_mask.sum()),
                      'unknown_fraction': float(self.unknown_mask.mean()),
                      'hit_pixels': int(hit.sum()),
                      'sky_pixels': int((~hit & ~self.unknown_mask).sum()),
                      'peer_pixels': int((peer_mask & ~self.unknown_mask).sum()),
                      'peer_tested_pixels': int(peer_tested_pixels),
                      'triangles': self.triangle_count,
                      'buildings': self.building_count, 'trees': len(self.trees),
                      'tree_geometry': 'conservative_analytic_primitives',
                      'backend': self.backend,
                      'nthreads': self.nthreads}
        return image
