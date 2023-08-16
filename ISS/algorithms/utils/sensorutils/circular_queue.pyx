from queue import Queue

class CircularQueue:
    def __init__(self, capacity=100):
        self.capacity = capacity
        self.queue = Queue(maxsize=capacity)

    def isEmpty(self):
        return self.queue.empty()
    
    def isFull(self):
        return self.queue.full()

    def enqueue(self, data):
        if self.isFull():
            print("Queue is full")
        else:
            self.queue.put(data)
    
    def dequeue(self, isWaitingStall, timeout):
        if self.isEmpty():
            print("Queue is empty")
        else:
            self.queue.get(isWaitingStall, timeout)

    def peek(self):
        if self.isEmpty():
            print("Queue is empty")
        else:
            return self.queue[0]