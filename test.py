import multiprocessing
import rospy
import time
from std_msgs.msg import String

# def worker(init_num):
#     for i in range(10):
#         init_num += 1
#     return init_num

# tasks = [i for i in range(20)]
# with multiprocessing.Pool() as p:
#     results = p.map(worker, tasks)

#     for res in results:
#         print(res)

def worker(init_num, num):
    for i in range(10):
        init_num += 1
    if num == 5:
        time.sleep(5)
    return init_num

class TestNode:
    def __init__(self) -> None:
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_callback)
        self.pub = rospy.Publisher("test", String, queue_size=10)        
        self.num = 0
    
    def timer_callback(self, event):
        tasks = [(i, self.num) for i in range(20)]
        with multiprocessing.Pool() as p:
            results = p.starmap(worker, tasks)
        
        pub_msg = String()
        for result in results:
            pub_msg.data += str(result) + ", "
        self.pub.publish(pub_msg)
    
if __name__ == "__main__":
    rospy.init_node("test_node")
    node = TestNode()
    rospy.spin()