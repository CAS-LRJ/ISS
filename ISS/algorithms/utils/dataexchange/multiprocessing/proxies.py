from collections import deque

class DequeProxy(object):
    def __init__(self, *args, **kwargs):
        self.deque = deque(*args, **kwargs)
    def __len__(self):
        return self.deque.__len__()
    def __getitem__(self, *args, **kwargs):
        return self.deque.__getitem__(*args, **kwargs)
        pass
    def appendleft(self, x):
        self.deque.appendleft(x)
    def append(self, x):
        self.deque.append(x)
    def pop(self):
        return self.deque.pop()
    def popleft(self):
        return self.deque.popleft()
