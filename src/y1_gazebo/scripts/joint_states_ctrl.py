#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import math

# 关节列表
joint_names = [
    "joint1", "joint2", "joint3", "joint4",
    "joint5", "joint6", "joint7", "joint8"
]

# Publisher 字典
publishers = {}

# 缓存上一次的关节位置
last_positions = {}

def joint_state_callback(msg):
    # 将 JointState 转为 dict
    joint_positions = {name: pos for name, pos in zip(msg.name, msg.position)}

    for joint_name in joint_names:
        position = 0.0

        if joint_name == "joint8":
            # joint8 = joint7（可根据需要改为 -joint7）
            if "joint7" in joint_positions:
                position = joint_positions["joint7"]
            else:
                position = 0.0
        else:
            if joint_name in joint_positions:
                position = joint_positions[joint_name]
            else:
                continue  # 没有该关节数据

        # 仅在关节位置变化时发布
        last_pos = last_positions.get(joint_name, None)
        if last_pos is None or abs(position - last_pos) > 1e-5:
            cmd = Float64()
            cmd.data = position
            publishers[joint_name].publish(cmd)
            rospy.loginfo(f"发布 {joint_name} 位置: {position:.6f}")
            last_positions[joint_name] = position


def main():
    rospy.init_node("joint_states_ctrl")

    # 初始化 publishers
    for name in joint_names:
        topic = f"/gazebo/{name}_position_controller/command"
        publishers[name] = rospy.Publisher(topic, Float64, queue_size=10)

    # 订阅 joint_states
    rospy.Subscriber("/joint_states", JointState, joint_state_callback)

    rospy.loginfo("joint_states_ctrl 节点已启动")
    rospy.spin()


if __name__ == "__main__":
    main()
