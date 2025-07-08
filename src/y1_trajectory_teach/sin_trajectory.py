import math
import rospy
from imeta_y1_msg.msg import ArmJointPositionControl

def generate_sinusoidal_trajectory(joint_count, amplitude, frequency, phase_shift, duration, rate):
    """
    为每个关节生成正弦运动轨迹。

    参数说明：
    - joint_count: 关节数量（例如：6）
    - amplitude: 每个关节的正弦波幅度（列表，可为每个关节单独指定）
    - frequency: 正弦波频率（Hz）
    - phase_shift: 每个关节的相位偏移（列表，可为每个关节指定）
    - duration: 轨迹持续时间（秒）
    - rate: 发布频率（Hz）
    """
    trajectory = []
    total_points = int(duration * rate)
    for i in range(total_points):
        t = i / rate  # 当前时间
        positions = [
            amplitude[j] * math.sin(2 * math.pi * frequency * t) + phase_shift[j]
            for j in range(joint_count)
        ]
        trajectory.append(positions)
    return trajectory

if __name__ == '__main__':
    rospy.init_node('sin_trajectory', anonymous=True)

    pub = rospy.Publisher('/y1/arm_joint_position_control', ArmJointPositionControl, queue_size=1)

    # 设置参数
    joint_count = 6
    amplitude = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]  # 每个关节的幅度
    frequency = 1/4  # 频率 (Hz)
    phase_shift = [0.0, -math.pi/2, math.pi/2, 0, 0, 0]  # 每个关节的相位偏移
    duration = 10.0  # 轨迹持续时间
    rate_hz = 200  # 发送频率

    # 生成轨迹
    trajectory = generate_sinusoidal_trajectory(joint_count, amplitude, frequency, phase_shift, duration, rate_hz)
    rospy.sleep(2.0)

    msg = ArmJointPositionControl()
    msg.header.frame_id = "base_link"
    msg.header.stamp = rospy.Time.now()
    msg.arm_joint_position = trajectory[0]
    msg.arm_joint_velocity = [0.8] * joint_count
    pub.publish(msg)

    rospy.loginfo("Publish start position, sleeping for 3 seconds go to start position.")
    rospy.sleep(3.0)  # 给机械臂3秒时间移动到位
    
    rate = rospy.Rate(rate_hz)  # 设置循环频率
    idx = 1
    print("trajectory length: ", len(trajectory))
    while not rospy.is_shutdown() and idx < len(trajectory):
        msg.header.stamp = rospy.Time.now()
        msg.arm_joint_position = trajectory[0]
        msg.arm_joint_position[2] = trajectory[idx][2]
        
        # 如果需要固定速度，可以设定统一的速度值或再次计算
        msg.arm_joint_velocity = [0.8] * joint_count
        print("idx: ", idx)
        
        pub.publish(msg)
        idx += 1
        rate.sleep()