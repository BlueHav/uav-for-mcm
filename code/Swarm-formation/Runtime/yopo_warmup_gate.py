"""Bound cold model loading to one member per owned simulation directory."""
from contextlib import contextmanager
import fcntl
import os
from pathlib import Path
import stat

@contextmanager
def model_warmup_slot(path=None):
    if path is None:
        from portable_paths import SCENE
        path=SCENE/'Runtime'/'.yopo-warmup.lock'
    # Keep the file after unlocking: unlinking could let a third process lock
    # a new inode while an existing waiter still owns the old one.
    descriptor=os.open(str(Path(path)),os.O_CREAT|os.O_RDWR|os.O_CLOEXEC|os.O_NOFOLLOW,0o600)
    try:
        info=os.fstat(descriptor)
        if not stat.S_ISREG(info.st_mode) or info.st_uid!=os.geteuid():raise RuntimeError('Invalid model warmup lock owner/type')
        # Blocking flock releases the Python GIL. Only the loader thread waits;
        # map, sensor and controller timers continue, with their original limits.
        fcntl.flock(descriptor,fcntl.LOCK_EX)
        try:yield
        finally:fcntl.flock(descriptor,fcntl.LOCK_UN)
    finally:os.close(descriptor)
