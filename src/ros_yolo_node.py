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
