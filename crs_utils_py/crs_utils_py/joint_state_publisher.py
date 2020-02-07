#!/usr/bin/env python3

'''
@file joint_state_publisher.py
@date Jan 30, 2020
@copyright Copyright (c) 2020, Southwest Research Institute

Software License Agreement (BSD License)
Copyright (c) 2020, Southwest Research Institute

Redistribution and use in source and binary forms, with or without modification, are permitted provided 
that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the 
following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and 
the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or 
promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import rclpy
import sys
from rclpy.node import Node
from sensor_msgs.msg import JointState

NODE_NAME = 'joint_state_publisher'
JOINT_STATE_TOPIC = 'joint_states'
JOINT_NAMES_PARAM = 'joint_names'
JOINT_INIT_VALS_PARAM = 'joint_init_vals'
JOINT_STATE_SRC_TOPICS= 'src_topics'

class JointStatePub(Node):
    
    def __init__(self):
        super().__init__(NODE_NAME)
        self.js_pub_ = self.create_publisher(JointState, JOINT_STATE_TOPIC, 10)
        
        self.joints_dict_ = {}
        self.input_topics_names_ = []
        self.input_js_subs_ = []
        self.publish_timer_ = None
        
        param_names = [JOINT_NAMES_PARAM, JOINT_INIT_VALS_PARAM, JOINT_STATE_SRC_TOPICS]
        for pname in param_names:
            self.declare_parameter(pname, None)
            param = self.get_parameter(pname)
            if param.type_ is param.Type.NOT_SET:
               self.get_logger().error('Failed to find parameter \'%s\'' % (pname)) 
               sys.exit(-1)
        
        try:
            joint_names = self.get_parameter(JOINT_NAMES_PARAM).value
            joint_init_vals = self.get_parameter(JOINT_INIT_VALS_PARAM).value
            joint_init_vals = [float(i) for i in joint_init_vals]
            self.joints_dict_ = dict(zip(joint_names, joint_init_vals))
            
            self.input_topics_names_ = self.get_parameter(JOINT_STATE_SRC_TOPICS).value
        except rclpy.exceptions.ParameterNotDeclaredException as e:
            self.get_logger().error('Failed to read one or more parameters: %s' % str(e))
            sys.exit(-1)
            
        # creating subscribers
        for tn in self.input_topics_names_:
            sub = self.create_subscription(JointState, tn, self.src_joint_state_callback, 1)
            self.input_js_subs_.append(sub)
            
        # create publish timer
        PUBLISH_TIMER_PERIOD = 0.1
        self.publish_timer_ = self.create_timer(PUBLISH_TIMER_PERIOD, self.publish_state_timer_callback)
        
    def publish_state_timer_callback(self):
        js = JointState()
        
        for k, v in self.joints_dict_.items():
            js.name.append(k)
            js.position.append(v)
            js.velocity.append(0.0)
            js.effort.append(0.0)
            
        self.js_pub_.publish(js)
        
    def src_joint_state_callback(self, msg: JointState):
        
        for i, jn in enumerate(msg.name):
            self.joints_dict_[jn] = msg.position[i]
                
def main(args = None):
    rclpy.init(args = sys.argv)
    node = JointStatePub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
    
            
        
            
        