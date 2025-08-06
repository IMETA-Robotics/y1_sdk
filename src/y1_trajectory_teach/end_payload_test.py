#!/usr/bin/env python3

import rospy
import json
from imeta_y1_msg.msg import ArmJointPositionControl

def end_payload_test(control_pub):
  
    start_position = [0.0, 0.0, 0.1, 0.0, 0.0, 0.0]
    end_position = [0.0, -3.14, 3.14, 0.0, 0.0, 0.0]
    rospy.sleep(3) # TODO: 为什么要等待一会，第一个点才可以发送成功？
    
    msg = ArmJointPositionControl()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"
    msg.arm_joint_position = start_position
    msg.arm_joint_velocity = 0.8
    control_pub.publish(msg)
    rospy.loginfo("Publish start position, sleeping for 3 seconds go to start position.")
    rospy.sleep(3)  # 给机械臂3秒时间移动到位
    
    print("End payload test start.")
    for i in range(10):
      print(f"{i+1} times")
      print("Go to end positon.")
      msg.header.stamp = rospy.Time.now()
      msg.arm_joint_position = end_position
      msg.arm_joint_velocity = 0.8
      control_pub.publish(msg)
      rospy.sleep(3)
      
      print("Return to start positon.")
      msg.header.stamp = rospy.Time.now()
      msg.arm_joint_position = start_position
      msg.arm_joint_velocity = 0.8
      control_pub.publish(msg)
      rospy.sleep(3)
      

if __name__ == '__main__':
    rospy.init_node('end_payload_test', anonymous=True)

    pub = rospy.Publisher('/y1/arm_joint_position_control', ArmJointPositionControl, queue_size=1)

    end_payload_test(pub)