"""Linux display IPC; never imports ROS, the planner, or native map libraries.

Only bounded array copies happen in the producer. Protocol 5 keeps ndarray
payloads out of the pickle traversal. A single mmap slot replaces older views;
nonblocking flock means a stopped/dead reader cannot stall the controller.
The tiny append-only tmpfs journal preserves every path sample independently.
"""
import collections
import fcntl
import json
import mmap
import os
import pickle
import struct
import subprocess
import sys
import tempfile
from pathlib import Path


class SnapshotMailbox:
    HEADER=struct.Struct('<QII')

    def __init__(self,path,create=False,capacity=8*1024*1024):
        self.file=open(path,'w+b' if create else 'r+b',buffering=0)
        if create:
            os.fchmod(self.file.fileno(),0o600)
            # Reserve before the mission starts: a sparse mmap can SIGBUS the
            # flight process if tmpfs later runs out of space during a copy.
            os.posix_fallocate(self.file.fileno(),0,capacity)
        self.memory=mmap.mmap(self.file.fileno(),0)
        self.capacity=len(self.memory);self.sequence=0;self.seen=0

    def put(self,view):
        buffers=[]
        metadata=pickle.dumps(view,protocol=5,buffer_callback=buffers.append)
        raw=[buffer.raw() for buffer in buffers]
        lengths=struct.pack('<%dQ'%len(raw),*[len(buffer) for buffer in raw])
        size=self.HEADER.size+len(lengths)+len(metadata)+sum(map(len,raw))
        if size>self.capacity:return False
        try:fcntl.flock(self.file,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:return False
        try:
            offset=self.HEADER.size
            for data in [lengths,metadata]+raw:
                self.memory[offset:offset+len(data)]=data;offset+=len(data)
            self.sequence+=1
            self.HEADER.pack_into(self.memory,0,self.sequence,len(metadata),len(raw))
            return True
        finally:fcntl.flock(self.file,fcntl.LOCK_UN)

    def take(self):
        try:fcntl.flock(self.file,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:return None
        try:
            sequence,size,count=self.HEADER.unpack_from(self.memory)
            if sequence==self.seen or sequence==0:return None
            offset=self.HEADER.size
            lengths=struct.unpack_from('<%dQ'%count,self.memory,offset);offset+=8*count
            metadata=self.memory[offset:offset+size];offset+=size
            buffers=[]
            for length in lengths:
                buffers.append(self.memory[offset:offset+length]);offset+=length
            self.seen=sequence
        finally:fcntl.flock(self.file,fcntl.LOCK_UN)
        return pickle.loads(metadata,buffers=buffers)

    def close(self):self.memory.close();self.file.close()
    def __enter__(self):return self
    def __exit__(self,*args):self.close()


class PathJournalReader:
    def __init__(self,path,unlimited=False):
        self.file=open(path,'rb');self.run=None
        self.entries=collections.deque(maxlen=None if unlimited else 400)

    def read_through(self,sequence):
        while True:
            offset=self.file.tell();line=self.file.readline()
            if not line:return self.entries
            if not line.endswith(b'\n'):
                self.file.seek(offset);return self.entries
            entry=json.loads(line)
            if entry['sequence']>sequence:
                self.file.seek(offset);return self.entries
            if entry['run']!=self.run:self.entries.clear();self.run=entry['run']
            self.entries.append(entry)

    def close(self):self.file.close()


class DisplayProcess:
    def __init__(self,drone_id,count,worker_command=None):
        # tmpfs avoids filesystem I/O stalls on the flight process. Files are
        # mode 0600 inside a private 0700 directory, never remote/untrusted IPC.
        self.directory=tempfile.TemporaryDirectory(prefix='yopo-display-',dir='/dev/shm')
        self.root=self.directory.name
        self.mailbox=SnapshotMailbox(self.root+'/view',create=True)
        self.journal=open(self.root+'/path','ab',buffering=0)
        os.fchmod(self.journal.fileno(),0o600)
        self.sequence=0;self.accepted=0;self.dropped=0;self.error=None
        command=worker_command or [sys.executable,str(Path(__file__).with_name('visual_worker.py')),
                                    self.root,str(drone_id),str(count),str(os.getpid())]
        self.process=subprocess.Popen(command,close_fds=True)

    def submit(self,view):
        try:
            self.sequence+=1;view.sequence=self.sequence
            entry=dict(sequence=self.sequence,stamp=view.stamp,run=view.run_id,p=view.p.tolist())
            data=(json.dumps(entry,separators=(',',':'))+'\n').encode()
            if self.journal.write(data)!=len(data):raise OSError('Short display path journal write')
            if self.process.poll() is not None:
                self.error='display process exited';self.dropped+=1;return False
            accepted=self.mailbox.put(view)
            if accepted:self.accepted+=1
            else:self.dropped+=1
            return accepted
        except Exception as exc:
            # Visualization must never change the mission's motion decision.
            self.error=repr(exc);self.dropped+=1;return False

    def status(self):
        return dict(pid=self.process.pid,alive=self.process.poll() is None,
                    accepted=self.accepted,dropped=self.dropped,error=self.error)

    def close(self):
        if self.process.poll() is None:
            self.process.kill();self.process.wait(timeout=2)
        self.journal.close();self.mailbox.close();self.directory.cleanup()

    def __enter__(self):return self
    def __exit__(self,*args):self.close()
