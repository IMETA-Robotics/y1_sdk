#!/usr/bin/env python3

import rospy
import json
from imeta_y1_msg.msg import ArmJointState

def playback_trajectory(jsonl_file, control_pub):
    with open(jsonl_file, 'r') as f:
        data_lines = [json.loads(line) for line in f]

    if not data_lines:
        rospy.logerr("No data found in the file.")
        return
    
    rospy.sleep(3) # TODO: 为什么要等待一会，第一个点才可以发送成功？
    data_len  = len(data_lines)
    print("data size : ", data_len)
    input("Press key [Enter] to start play trajectory.")
    
    # gripper joint position control
    gripper = [i / 3 for i in range(240)]
    gripper += [x / 3 for x in reversed(range(240))]
    gripper_len = len(gripper)
    gripper_index = 0

    total_counts = 3
    count = 0
    while count < total_counts :
        idx = 1  # 第一个点已经发过了
        msg = ArmJointState()
        msg.header.stamp = rospy.Time.now()
        joint_position = list(data_lines[0]['position'][0:6])
        joint_position.append(gripper[gripper_index])
        msg.joint_position = joint_position
        msg.joint_velocity = data_lines[0]['velocity'][0:6]
        control_pub.publish(msg)
        print(f"{count+1}th times playback_trajectory")
        rospy.loginfo("Publish start position, sleeping for 3 seconds go to start position.")
        rospy.sleep(3)  # 给机械臂3秒时间移动到位
        
        rate = rospy.Rate(400)  # high frequency
        while not rospy.is_shutdown() and idx < data_len:
            msg.header.stamp = rospy.Time.now()
            if gripper_index >= gripper_len:
                gripper_index = 0

            joint_position = list(data_lines[idx]['position'][0:6])
            joint_position.append(gripper[gripper_index])
            msg.joint_position = joint_position
            msg.joint_velocity = data_lines[idx]['velocity'][0:6]

            control_pub.publish(msg)
            idx += 1
            gripper_index += 1
            print("index: ", idx)
            rate.sleep()
        count += 1

if __name__ == '__main__':
    rospy.init_node('play_trajectory', anonymous=True)

    jsonl_file = "/home/zxf/IMETA_LAB/Y1/data/arm_state_400hz.jsonl"
    
    pub = rospy.Publisher('/master_arm_right/joint_states', ArmJointState, queue_size=1)

    rospy.loginfo(f"Preparing to play back trajectory from {jsonl_file}...")
    playback_trajectory(jsonl_file, pub)