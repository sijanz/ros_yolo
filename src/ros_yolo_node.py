#!/usr/bin/env python

"""
Software License Agreement (BSD 3-Clause License)

Copyright (c) 2023, Simon Janzon
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above
   copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided
   with the distribution.
3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import rospy
import cv2
import json
from ultralytics import YOLO
from ros_yolo.msg import Pose, Poses, Segment, Segments
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


RECEIVED_IMAGE = None


class YoloSegment:
    def __init__(self, name, segment_x, segment_y):
        self.name = name
        self.segment_x = segment_x
        self.segment_y = segment_y
  
        
class YoloPose:
    def __init__(self, x_values, y_values):
        self.x_values = x_values
        self.y_values = y_values
        
        
def image_callback(data):
    bridge = CvBridge()
    global RECEIVED_IMAGE
    RECEIVED_IMAGE = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')


def ros_yolo():
    rospy.init_node('ros_yolo', anonymous=True)
    rate = rospy.Rate(10)
    
    # create YOLO model by loading the path via launch parameter
    yolo_path = rospy.get_param('/ros_yolo/yolo_path')
    
    # DEBUG
    print('yolo path: ', str(yolo_path))
    
    yolo_model = YOLO(yolo_path)
    
    confidence_threshold = rospy.get_param('/ros_yolo/confidence_threshold')
    
    # DEBUG
    print('yolo path: ', yolo_path)
    print('confidence threshold: ', confidence_threshold)
    
    pub = None
    
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    
    if "seg" in yolo_path:
        pub = rospy.Publisher('/yolo_segments', Segments, queue_size=10)
    elif "pose" in yolo_path:
        pub = rospy.Publisher('/yolo_pose', Poses, queue_size=10)
    
    while not rospy.is_shutdown():
        
        if RECEIVED_IMAGE is None:
            continue
        
        # get image from callback and convert it
        img = RECEIVED_IMAGE
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        # infere YOLO model
        results = yolo_model(img)
        
        data = json.loads(results[0].tojson())
        
        msg = []

        for object in data:
            
            print(object['name'])
            print(object['confidence'])
            
            if float(object['confidence']) > confidence_threshold:
                detected_object = Segment(object['name'], object['segments']['x'], object['segments']['y'])
                msg.append(detected_object)
        
        rate.sleep()
        
    return

if __name__ == '__main__':
    try:
        ros_yolo()
    except rospy.ROSInterruptException:
        pass
