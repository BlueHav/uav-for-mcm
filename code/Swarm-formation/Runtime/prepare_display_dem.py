"""Prepare a display-only DEM cache; preserve current Unity terrain inside its bounds."""
import hashlib
import json
import sys
from pathlib import Path
import numpy as np
import rasterio
from rasterio.transform import from_origin
from rasterio.warp import reproject, Resampling

root = Path(sys.argv[1] if len(sys.argv) > 1 else ('__UAV_PROJECT__'.replace("__UAV_PROJECT__", __import__("os").environ["UAV_PROJECT_ROOT"])))
c = json.loads((root/'point.json').read_text())
mp = Path(c['paths']['map'])
if not mp.is_absolute():
    mp = root/mp
meta = json.loads((mp/c['paths']['map_files']['metadata']).read_text())
world = json.loads((mp/c['paths']['map_files']['world']).read_text())
b = world['bounds']; step = 100.
# Cover every possible display center in the existing simulation map, plus 10 km.
x0 = np.floor((b['x_min']-10000)/step)*step
y0 = np.floor((b['y_min']-10000)/step)*step
nx = int(np.ceil((b['x_max']+10000-x0)/step))+1
ny = int(np.ceil((b['y_max']+10000-y0)/step))+1
east = meta['bounds_utm'][0]-b['x_min']
north = meta['bounds_utm'][1]-b['y_min']
dst = np.full((ny, nx), np.nan, np.float32)
source = Path(c['paths']['source_dem'])
with rasterio.open(source) as src:
    reproject(rasterio.band(src, 1), dst, src_transform=src.transform, src_crs=src.crs,
              src_nodata=src.nodata, dst_transform=from_origin(x0+east-step/2, y0+(ny-1)*step+north+step/2, step, step),
              dst_crs=meta['crs'], dst_nodata=np.nan, resampling=Resampling.bilinear)
dst = dst[::-1]-meta['source_datum_offset']
out = root/'display'; out.mkdir(exist_ok=True)
np.savez_compressed(out/'dem_cache.npz', heights=dst, x0=x0, y0=y0, step=step)
record = dict(scope='display only, never sensor data', source=str(source), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
              world_sha256=hashlib.sha256((mp/c['paths']['map_files']['world']).read_bytes()).hexdigest(),
              metadata_sha256=hashlib.sha256((mp/c['paths']['map_files']['metadata']).read_bytes()).hexdigest(),
              cache_sha256=hashlib.sha256((out/'dem_cache.npz').read_bytes()).hexdigest(),
              cached_bounds=[x0, y0, x0+(nx-1)*step, y0+(ny-1)*step], step_m=step, valid_fraction=float(np.isfinite(dst).mean()),
              source_datum_offset=meta['source_datum_offset'], note='Interpolation increases display density, not survey resolution.')
(out/'dem_metadata.json').write_text(json.dumps(record, indent=2))
print(json.dumps(record))
