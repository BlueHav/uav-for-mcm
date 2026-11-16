"""Bounded sensor scheduling and depth validation. No ROS/planner imports."""
import math
import threading
import time
import numpy as np
from depth_transport import (HEADER, RAW_LENGTH, Frame, Lz4Decoder, ProtocolError,
                             decode_header, decode_payload, read_exact)


class LatestScheduler:
    """One immutable latest value per declared stream; rotating fair dispatch."""
    def __init__(self, thresholds):
        self.keys = list(thresholds)
        self.thresholds = dict(thresholds)
        self.lock = threading.RLock()
        self.slots = {}
        self.sequences = {}
        self.cursor = 0
        self.stats = {k: dict(received=0, published=0, dropped=0, replaced=0,
                             invalid=0, expired=0, last_stamp=None, last_sequence=None,
                             last_receive_mono=None, last_publish_mono=None,
                             max_publish_ms=0., reason=None) for k in self.keys}

    def reject(self, key, reason):
        with self.lock:
            s=self.stats[key];s['dropped']+=1;s['invalid']+=1;s['reason']=reason

    def submit(self, key, value, stamp, now, mono, sequence=None):
        with self.lock:
            s=self.stats[key]
            s['received']+=1
            if not math.isfinite(stamp) or not -.05 <= now-stamp <= self.thresholds[key]:
                self.reject(key,'stale_or_future');return False
            if sequence is not None and sequence <= self.sequences.get(key,-1):
                self.reject(key,'sequence');return False
            if sequence is None and s['last_stamp'] is not None and stamp<=s['last_stamp']:
                self.reject(key,'stamp_order');return False
            if key in self.slots:
                s['replaced']+=1;s['dropped']+=1
            if sequence is not None:self.sequences[key]=sequence
            s.update(last_stamp=stamp,last_sequence=sequence,last_receive_mono=mono,reason=None)
            self.slots[key]=(value,stamp,mono)
            return True

    def take(self, now, mono):
        with self.lock:
            for _ in self.keys:
                key=self.keys[self.cursor];self.cursor=(self.cursor+1)%len(self.keys)
                item=self.slots.pop(key,None)
                if item is None:continue
                value,stamp,received=item
                if not -.05 <= now-stamp <= self.thresholds[key] or mono-received > self.thresholds[key]:
                    s=self.stats[key];s['expired']+=1;s['dropped']+=1;s['reason']='dispatch_expired'
                    continue
                return key,value
            return None

    def published(self,key,mono,duration_ms):
        with self.lock:
            s=self.stats[key];s['published']+=1;s['last_publish_mono']=mono
            s['max_publish_ms']=max(s['max_publish_ms'],duration_ms)

    def clear(self,kind):
        with self.lock:
            for key in self.keys:
                if key[0]!=kind:continue
                if key in self.slots:
                    self.slots.pop(key);self.stats[key]['dropped']+=1
                self.sequences.pop(key,None)
                self.stats[key].update(last_stamp=None,last_sequence=None,last_receive_mono=None,
                                       last_publish_mono=None,reason='source_disconnected')

    def status(self,now,mono):
        with self.lock:
            out=[]
            for key in self.keys:
                value=dict(self.stats[key]);stamp=value['last_stamp'];recv=value['last_receive_mono']
                age=None if stamp is None else now-stamp
                receive_age=None if recv is None else mono-recv
                value.update(stream=list(key),queue=int(key in self.slots),capacity=1,
                             age_s=age,receive_age_s=receive_age,
                             fresh=age is not None and -.05<=age<=self.thresholds[key]
                             and receive_age<=self.thresholds[key])
                if key[0]=='depth':value.update(drone_id=key[1],camera=('front','up','down')[key[2]])
                elif key[0] in ('ground','local_ground'):value.update(drone_id=key[1])
                else:value.update(display_only=True,source_kind='map_terrain_sample_not_radar')
                out.append(value)
            return out


class IsolatedFrameReader:
    """Consume a complete bounded frame before rejecting one damaged stream.

    Invalid framing/shape still closes the connection: its next boundary cannot
    be trusted. CRC, numeric, age and sequence errors preserve other streams.
    """
    def __init__(self):
        self.header_buffer=bytearray(HEADER.size)
        self.payload_buffer=bytearray(RAW_LENGTH)
        self.decoder=Lz4Decoder()
        self.sequences={}

    def read(self,sock,now=None):
        read_exact(sock,self.header_buffer,HEADER.size)
        try:h=decode_header(self.header_buffer)
        except ProtocolError as exc:
            # A bounded v1 payload still has a trustworthy next boundary even
            # when a pose, stamp, ID or reserved field is invalid.
            parts=HEADER.unpack(self.header_buffer)
            magic,version,codec,drone,camera=parts[:5]
            width,height,raw,size=parts[8:12]
            if magic!=b'YDLZ' or version!=1 or (width,height,raw)!=(480,270,RAW_LENGTH) or not 0<size<=RAW_LENGTH:
                raise
            read_exact(sock,self.payload_buffer,size)
            key=('depth',drone,camera) if drone<10 and camera<3 else None
            return None,key,str(exc)
        read_exact(sock,self.payload_buffer,h.payload_length)
        key=('depth',h.drone,h.camera)
        age=(time.time() if now is None else now() if callable(now) else now)-(h.sec+h.nsec*1e-9)
        if not -.05 <= age <= .6:return None,key,'stale_or_future'
        if h.sequence<=self.sequences.get(key,-1):return None,key,'sequence'
        payload=bytes(memoryview(self.payload_buffer)[:h.payload_length])
        try:raw=decode_payload(h,payload,self.decoder)
        except ProtocolError as exc:return None,key,str(exc)
        values=np.frombuffer(raw,dtype='<f4')
        if not np.isfinite(values).all() or np.any(values<0.):return None,key,'depth_values'
        self.sequences[key]=h.sequence
        return Frame(h,raw),key,None
