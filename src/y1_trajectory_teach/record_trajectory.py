#!/usr/bin/env python3
import rospy, json, os
from imeta_y1_msg.msg import ArmJointState

latest = None
def state_callback(msg):
    global latest
    latest = msg

if __name__ == "__main__":
    rospy.init_node('record_trajectory', anonymous=True)
    os.makedirs("data", exist_ok=True)
    rospy.Subscriber("/puppet_arm_right/joint_states",
                     ArmJointState, state_callback, queue_size=1)

    rate = rospy.Rate(30)  # 20Hz
    with open("data/arm_state_30hz.jsonl", 'a') as f:
        while not rospy.is_shutdown():
            if latest is not None:
                data = {
                    'position': list(latest.joint_position),
                    'velocity': list(latest.joint_velocity)
                }
                f.write(json.dumps(data) + '\n')
                rospy.loginfo("Saved snapshot")
            rate.sleep()
