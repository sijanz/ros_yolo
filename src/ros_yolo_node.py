#!/usr/bin/env python

import rospy


YOLO_CONFIDENCE_THRESHOLD = 0.8


class YoloSegment:
    def __init__(self, name, segment_x, segment_y):
        self.name = name
        self.segment_x = segment_x
        self.segment_y = segment_y
  
        
class YoloPose:
    def __init__(self, x_values, y_values):
        self.x_values = x_values
        self.y_values = y_values


def ros_yolo():
    
    rospy.init_node('ros_yolo', anonymous=True)
    rate = rospy.Rate(10)
    print(YOLO_CONFIDENCE_THRESHOLD)
    
    while not rospy.is_shutdown():
        
        # TODO: add code here
        
        rate.sleep()
        
    return

if __name__ == '__main__':
    try:
        ros_yolo()
    except rospy.ROSInterruptException:
        pass
