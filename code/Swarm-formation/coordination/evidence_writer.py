# Bounded reliable evidence sink. Disk and serialization never own the state lock.
import csv,json,queue,threading,time
from pathlib import Path
class EvidenceWriter:
 def __init__(self,base,capacity=512,write_hook=None):
  self.base=Path(base);self.queue=queue.Queue(maxsize=capacity);self.error=None;self.rejected=0;self.submitted=0;self.written=0;self.max_pending=0;self.closing=False;self.write_hook=write_hook;self.submit_lock=threading.Lock();self.accepted_by_kind={k:0 for k in ('telemetry','csv0','csv1')};self.written_by_kind={k:0 for k in self.accepted_by_kind}
  self.log=(self.base/'telemetry.jsonl').open('w',buffering=1);self.raw={i:(self.base/('trajectory-drone-%d.csv'%i)).open('w',newline='',buffering=1) for i in (0,1)};self.csv={i:csv.writer(f) for i,f in self.raw.items()}
  for w in self.csv.values():w.writerow(['stamp','x','y','z','vx','vy','vz'])
  self.thread=threading.Thread(target=self.run,name='evidence_writer',daemon=True);self.thread.start()
 def submit(self,kind,payload):
  # Payload ownership transfers to this queue; callers never mutate it again.
  with self.submit_lock:
   if self.closing or self.error:self.rejected+=1;return False
   try:self.queue.put_nowait((kind,payload));self.submitted+=1;self.accepted_by_kind[kind]+=1;self.max_pending=max(self.max_pending,self.queue.qsize());return True
   except queue.Full:self.error='evidence_queue_overflow';self.rejected+=1;return False
 def run(self):
  io_failed=False
  while True:
   item=self.queue.get()
   try:
    if item is None:break
    if io_failed:self.rejected+=1;continue
    kind,payload=item
    try:
     if self.write_hook:self.write_hook(kind,payload)
     if kind=='telemetry':self.log.write(json.dumps(payload,allow_nan=True)+'\n')
     else:self.csv[int(kind[-1])].writerow(payload)
     self.written+=1;self.written_by_kind[kind]+=1
    except BaseException as exc:self.error='evidence_write_error:'+repr(exc);self.rejected+=1;io_failed=True
   finally:self.queue.task_done()
  try:
   self.log.flush();self.log.close()
   for f in self.raw.values():f.flush();f.close()
  except BaseException as exc:self.error='evidence_flush_error:'+repr(exc)
 def close(self,timeout=30):
  with self.submit_lock:self.closing=True
  deadline=time.monotonic()+timeout
  while True:
   try:self.queue.put(None,timeout=.1);break
   except queue.Full:
    if time.monotonic()>deadline:self.error='evidence_close_timeout';return self.stats()
  self.thread.join(max(0.,deadline-time.monotonic()))
  if self.thread.is_alive():self.error='evidence_close_timeout'
  return self.stats()
 def stats(self):return dict(error=self.error,submitted=self.submitted,written=self.written,rejected=self.rejected,max_pending=self.max_pending,pending=self.queue.qsize(),writer_alive=self.thread.is_alive(),accepted_by_kind=dict(self.accepted_by_kind),written_by_kind=dict(self.written_by_kind))
