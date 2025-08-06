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
    rospy.Subscriber("/master_arm_right/joint_states",
                     ArmJointState, state_callback, queue_size=1)

    rate = rospy.Rate(200)  # 200Hz
    with open("data/arm_state_200hz.jsonl", 'a') as f:
        while not rospy.is_shutdown():
            if latest is not None:
                data = {
                    'position': list(latest.joint_position),
                    'velocity': list(latest.joint_velocity),
                    'effort': list(latest.joint_effort),
                }
                f.write(json.dumps(data) + '\n')
                rospy.loginfo("Saved snapshot")
            rate.sleep()
