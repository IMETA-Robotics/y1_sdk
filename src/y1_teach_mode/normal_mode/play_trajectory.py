#!/usr/bin/env python3

import rospy
import json
from imeta_y1_msg.msg import ArmJointPositionControl

def playback_trajectory(jsonl_file, control_pub):
    with open(jsonl_file, 'r') as f:
        data_lines = [json.loads(line) for line in f]

    if not data_lines:
        rospy.logerr("No data found in the file.")
        return

    data_len  = len(data_lines)
    print("data size : ", data_len)
    idx = 1  # 第一个点已经发过了
    msg = ArmJointPositionControl()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"
    msg.arm_joint_position = data_lines[0]['position'][0:6]
    msg.arm_joint_velocity = 0.4
    input("enter anything to start recording")
    
    rospy.sleep(3) # TODO: 为什么要等待一会，第一个点才可以发送成功？

    control_pub.publish(msg)
    rospy.loginfo("Publish start position, sleeping for 3 seconds go to start position.")
    rospy.sleep(3)  # 给机械臂3秒时间移动到位
    print("Playback started.")
    
    rate = rospy.Rate(25)
    while not rospy.is_shutdown() and idx < data_len:
        msg.header.stamp = rospy.Time.now()
        msg.arm_joint_position = data_lines[idx]['position'][0:6]
        msg.arm_joint_velocity = 0.4
        msg.gripper = data_lines[idx]['position'][6]

        control_pub.publish(msg)
        idx += 1
        print("index: ", idx)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('play_trajectory', anonymous=True)

    jsonl_file = "/home/zxf/IMETA_LAB/Y1/data/arm_state_clothse_25hz.jsonl"
    
    pub = rospy.Publisher('/y1/arm_joint_position_control', ArmJointPositionControl, queue_size=1)

    rospy.loginfo(f"Preparing to play back trajectory from {jsonl_file}...")
    playback_trajectory(jsonl_file, pub)
