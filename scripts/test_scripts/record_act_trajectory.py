#!/usr/bin/env python3
import rospy, json, os
from y1_msg.msg import ArmJointPositionControl

latest = None
def state_callback(msg):
    global latest
    latest = msg

if __name__ == "__main__":
    rospy.init_node('record_trajectory', anonymous=True)
    os.makedirs("data", exist_ok=True)
    rospy.Subscriber("/y1/arm_joint_position_control",
                     ArmJointPositionControl, state_callback, queue_size=1)
    
    input("Press key [Enter] to start recording")

    rate = rospy.Rate(20)
    with open("data/arm_state_act_20hz.jsonl", 'a') as f:
        while not rospy.is_shutdown():
            if latest is not None:
                data = {
                    'position': list(latest.arm_joint_position),
                    'velocity': latest.arm_joint_velocity,
                    "gripper": latest.gripper,
                }
                f.write(json.dumps(data) + '\n')
                rospy.loginfo("Saved snapshot")
            rate.sleep()
