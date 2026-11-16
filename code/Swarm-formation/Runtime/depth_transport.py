"""Bounded lossless depth wire protocol. No ROS or planner imports."""
import ctypes
import math
import struct
import time
import zlib
from dataclasses import dataclass

HEADER = struct.Struct('<4sBBBBQ8I7d24x')
RAW_LENGTH = 480 * 270 * 4

class ProtocolError(ValueError): pass

@dataclass(frozen=True)
class FrameHeader:
    codec: int
    drone: int
    camera: int
    sequence: int
    sec: int
    nsec: int
    width: int
    height: int
    raw_length: int
    payload_length: int
    crc: int
    pose: tuple

@dataclass(frozen=True)
class Frame:
    header: FrameHeader
    data: bytes

def decode_header(data):
    if len(data) != HEADER.size: raise ProtocolError('header length')
    magic, version, codec, drone, camera, seq, sec, nsec, w, h, raw, size, crc, reserved, *pose = HEADER.unpack(data)
    if magic != b'YDLZ' or version != 1: raise ProtocolError('magic/version')
    if codec not in (0,1) or drone >= 10 or camera >= 3: raise ProtocolError('stream/codec')
    if (w,h,raw) != (480,270,RAW_LENGTH): raise ProtocolError('image shape')
    if not 0 < size <= raw or (codec == 0 and size != raw): raise ProtocolError('payload length')
    if nsec >= 1000000000 or reserved or any(data[104:128]): raise ProtocolError('timestamp/reserved')
    if not all(math.isfinite(x) for x in pose) or abs(sum(x*x for x in pose[3:])-1.) > .01:
        raise ProtocolError('pose')
    return FrameHeader(codec,drone,camera,seq,sec,nsec,w,h,raw,size,crc,tuple(pose))

class Lz4Decoder:
    def __init__(self):
        self.lib = ctypes.CDLL('liblz4.so.1')
        self.lib.LZ4_decompress_safe.argtypes = [ctypes.c_void_p,ctypes.c_void_p,ctypes.c_int,ctypes.c_int]
        self.lib.LZ4_decompress_safe.restype = ctypes.c_int
        self.output = ctypes.create_string_buffer(RAW_LENGTH)
    def decode(self, payload):
        size = self.lib.LZ4_decompress_safe(payload,self.output,len(payload),RAW_LENGTH)
        if size != RAW_LENGTH: raise ProtocolError('LZ4 decoded length')
        return self.output.raw

def decode_payload(header,payload,decoder=None):
    if len(payload) != header.payload_length: raise ProtocolError('truncated payload')
    raw = payload if header.codec == 0 else (decoder or Lz4Decoder()).decode(payload)
    if zlib.crc32(raw) != header.crc: raise ProtocolError('CRC')
    return raw

def read_exact(sock,buffer,size):
    view=memoryview(buffer)[:size]
    while view:
        received=sock.recv_into(view)
        if not received: raise EOFError('depth connection closed')
        view=view[received:]

class FrameReader:
    """One reader per connection. Only valid, fresh frames advance sequence."""
    def __init__(self):
        self.header_buffer=bytearray(HEADER.size)
        self.payload_buffer=bytearray(RAW_LENGTH)
        self.decoder=Lz4Decoder()
        self.sequences={}
    def read(self,sock,now=None):
        read_exact(sock,self.header_buffer,HEADER.size)
        header=decode_header(self.header_buffer)
        read_exact(sock,self.payload_buffer,header.payload_length)
        age=(time.time() if now is None else now)-(header.sec+header.nsec*1e-9)
        if age > .6 or age < -.05: raise ProtocolError('stale/future capture')
        key=(header.drone,header.camera)
        if header.sequence <= self.sequences.get(key,-1): raise ProtocolError('sequence')
        # Immutable bytes own the ROS message until asynchronous publication finishes.
        payload=bytes(memoryview(self.payload_buffer)[:header.payload_length])
        raw=decode_payload(header,payload,self.decoder)
        self.sequences[key]=header.sequence
        return Frame(header,raw)
