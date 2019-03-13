#!/usr/bin/env python  
import rospy

from std_msgs.msg import Int16
from project1_solution.msg import TwoInts

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'The numbers are %d, %d',data.a, data.b)
    
def listener():
    rospy.init_node('listener',anonymous=True)
    rospy.Subscriber('two_ints',TwoInts,callback)
    rospy.spin()
    
if __name__ == '__main__':
    listener()
