import threading,time,functools
class ProducerGate:
 def __init__(self):self.condition=threading.Condition();self.closed=False;self.active=0;self.local=threading.local()
 def wrap(self,function):
  @functools.wraps(function)
  def guarded(*args,**kw):
   with self.condition:
    if self.closed:return
    self.active+=1
   self.local.depth=getattr(self.local,'depth',0)+1
   try:return function(*args,**kw)
   finally:
    self.local.depth-=1
    with self.condition:self.active-=1;self.condition.notify_all()
  return guarded
 def current_accepted(self):return getattr(self.local,'depth',0)>0
 def close(self):
  with self.condition:self.closed=True
 def wait(self,timeout):
  deadline=time.monotonic()+timeout
  with self.condition:
   while self.active:
    left=deadline-time.monotonic()
    if left<=0:return False
    self.condition.wait(left)
  return True
