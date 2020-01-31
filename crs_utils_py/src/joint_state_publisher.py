#!/usr/bin/env python3

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
    
    
            
        
            
        