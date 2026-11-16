"""Spatial reuse of advisory optimizer seeds, never flight permission."""
import numpy as np


def usable_hint(intent,position,now,token,run,world_hash,static=False):
    if not isinstance(intent,dict) or intent.get('token')!=token:return False
    try:
        age=now-intent['stamp']
        if not np.isfinite(age) or age<-.05:return False
        if age<=.7:return True
        identity=intent.get('seed_identity',{})
        if (not static or not isinstance(world_hash,str) or not world_hash
                or identity.get('run_id')!=run or identity.get('epoch')!=2
                or identity.get('world_hash')!=world_hash):return False
        seed=np.asarray(intent['position'],float);current=np.asarray(position,float)
        return bool(seed.shape==(3,) and current.shape==(3,) and np.isfinite(seed).all()
                    and np.isfinite(current).all() and np.linalg.norm(current-seed)<=10.)
    except (KeyError,TypeError,ValueError):return False
