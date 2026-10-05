from collections import deque


class DedupCache:
    def __init__(self,max_items=4096): self.max_items=max_items;self._order=deque();self._seen=set()
    def accept(self,key):
        if key in self._seen:return False
        self._seen.add(key);self._order.append(key)
        if len(self._order)>self.max_items:self._seen.discard(self._order.popleft())
        return True
