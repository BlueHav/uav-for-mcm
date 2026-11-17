"""Bounded latest-measurement mailbox; consuming never clears a newer arrival."""
from collections import deque

class LatestMeasurement:
 def __init__(self):self._items=deque(maxlen=1)
 def put(self,value):self._items.append(value)
 def take(self):
  try:return self._items.popleft()
  except IndexError:return None
