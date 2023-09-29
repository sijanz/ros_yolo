#!/usr/bin/env python

import rospy


def ros_yolo():
    
    rospy.init_node('ros_yolo', anonymous=True)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        
        # TODO: add code here
        
        rate.sleep()
        
    return

if __name__ == '__main__':
    try:
        ros_yolo()
    except rospy.ROSInterruptException:
        pass