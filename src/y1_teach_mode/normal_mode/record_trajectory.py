#!/usr/bin/env python3
import rospy, json, os
from y1_msg.msg import ArmJointState

latest = None
def state_callback(msg):
    global latest
    latest = msg

if __name__ == "__main__":
    rospy.init_node('record_trajectory', anonymous=True)
    os.makedirs("data", exist_ok=True)
    rospy.Subscriber("/master_arm_right/joint_states",
                     ArmJointState, state_callback, queue_size=1)
    
    input("press key [Enter] to start record trajectory.")

    rate = rospy.Rate(25)
    with open("data/arm_state_25hz.jsonl", 'a') as f:
        while not rospy.is_shutdown():
            if latest is not None:
                data = {
                    'position': list(latest.joint_position),
                    'velocity': list(latest.joint_velocity),
                    'effort': list(latest.joint_effort),
                    "end_pose": list(latest.end_pose),
                }
                f.write(json.dumps(data) + '\n')
                rospy.loginfo("Saved snapshot")
            rate.sleep()
