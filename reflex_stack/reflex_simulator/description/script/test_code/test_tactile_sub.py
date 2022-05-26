#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from reflex_msgs.msg import Hand

def callback(data):
    global pressureFinger 
    pressureFinger = [data.finger[0].pressure , data.finger[1].pressure, data.finger[2].pressure]
    rospy.loginfo(pressureFinger)
    
def listener():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/reflex_takktile/hand_state", Hand, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
    

if __name__ == '__main__':
    listener()
