#!/usr/bin/env python3
import rospy
import json
from imeta_y1_msg.msg import ArmJointPositionControl

def playback_trajectory(jsonl_file):
    with open(jsonl_file, 'r') as f:
        data_lines = [json.loads(line) for line in f]

    if not data_lines:
        rospy.logerr("No data found in the file.")
        return

    idx = 1  # 第一个点已经发过了
    msg = ArmJointPositionControl()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"
    msg.arm_joint_position = data_lines[0]['position']
    msg.arm_joint_velocity = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    
    print("joint_velocity: ", msg.arm_joint_velocity)

    pub.publish(msg)
    rospy.loginfo("Publish start position, sleeping for 3 seconds go to start position.")
    rospy.sleep(10.0)  # 给机械臂3秒时间移动到位
    print("Playback started.")
    
    rate = rospy.Rate(200)  # 200Hz
    while not rospy.is_shutdown() and idx < len(data_lines):
        msg.arm_joint_position = data_lines[idx]['position']
        msg.arm_joint_velocity = data_lines[idx]['velocity']
        
        pub.publish(msg)
        idx += 1
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('play_trajectory', anonymous=True)

    jsonl_file = "/home/ubuntu/IMETA_LAB/Y1/data/arm_state.jsonl"
    
    pub = rospy.Publisher('/y1_controller/arm_joint_position_control', ArmJointPositionControl, queue_size=1)

    rospy.loginfo(f"Preparing to play back trajectory from {jsonl_file}...")
    playback_trajectory(jsonl_file)

    rospy.spin()