#!/usr/bin/env python3

import rospy
import json
import os
import time
from imeta_y1_msg.msg import ArmState

def ArmStateCallBack(msg: ArmState):
    start_time = time.time()  # 开始计时
    
    # 仅保存 position 和 velocity
    data = {
        'position': list(msg.joint_position),
        'velocity': list(msg.joint_velocity)
    }
    
    with open(output_file, 'a') as f:
        f.write(json.dumps(data) + '\n')
        
    end_time = time.time()  # 结束计时

    loop_duration = end_time - start_time
    rospy.loginfo(f"Loop duration: {loop_duration:.6f} seconds")  # 可改为 rospy.loginfo 查看

if __name__ == '__main__':
    rospy.init_node('record_trajectory', anonymous=True)
    # topic_name = rospy.get_param('~topic', '/joint_states')
    topic_name = "/y1/arm_joint_state"
    # output_file = rospy.get_param('~output', 'joint_states.jsonl')
    output_file = "data/arm_state.jsonl"
    os.makedirs(os.path.dirname(output_file), exist_ok=True)  # 自动创建 data 目录

    rospy.Subscriber(topic_name, ArmState, ArmStateCallBack)
    rospy.loginfo(f"Subscribing {topic_name}, saving to {output_file}")
    rospy.spin()
