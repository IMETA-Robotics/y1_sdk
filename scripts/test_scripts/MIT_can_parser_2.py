import matplotlib.pyplot as plt
import re

# 定义 uint_to_float 函数
def uint_to_float(x_int, x_min, x_max, bits):
    span = x_max - x_min
    return float(x_int) * span / (2**bits - 1) + x_min

# 参数范围
pmax = 6.28
vmax = 10
tmax = 30

# 读取本地txt文件
with open('/home/zxf/IMETA_LAB/Y1/J4_can.txt', 'r') as f:
    lines = f.readlines()

# 初始化结果列表
cmd_pos, cmd_vel, cmd_torque = [], [], []
fbk_pos, fbk_vel, fbk_torque = [], [], []

# 正则表达式提取报文数据 (单行)
pattern_cmd = re.compile(r'can0\s+004\s+\[8\]\s+((?:[\dA-F]{2}\s+){7}[\dA-F]{2})', re.IGNORECASE)
pattern_fbk = re.compile(r'can0\s+014\s+\[8\]\s+((?:[\dA-F]{2}\s+){7}[\dA-F]{2})', re.IGNORECASE)

cmd_data, fbk_data = None, None

for line in lines:
    match_cmd = pattern_cmd.search(line)
    match_fbk = pattern_fbk.search(line)

    if match_cmd:
        cmd_data = [int(b, 16) for b in match_cmd.group(1).strip().split()]
        if fbk_data:  # 有反馈数据时配对解析
            # 下发解析
            pos_tmp = (cmd_data[0] << 8) | cmd_data[1]
            vel_tmp = (cmd_data[2] << 4) | (cmd_data[3] >> 4)
            tor_tmp = ((cmd_data[6] & 0xF) << 8) | cmd_data[7]

            cmd_pos.append(uint_to_float(pos_tmp, -pmax, pmax, 16))
            cmd_vel.append(uint_to_float(vel_tmp, -vmax, vmax, 12))
            cmd_torque.append(uint_to_float(tor_tmp, -tmax, tmax, 12))

            # 反馈解析
            p_int = (fbk_data[1] << 8) | fbk_data[2]
            v_int = (fbk_data[3] << 4) | (fbk_data[4] >> 4)
            t_int = ((fbk_data[4] & 0xF) << 8) | fbk_data[5]

            fbk_pos.append(uint_to_float(p_int, -pmax, pmax, 16))
            fbk_vel.append(uint_to_float(v_int, -vmax, vmax, 12))
            fbk_torque.append(uint_to_float(t_int, -tmax, tmax, 12))

            # 清空，准备下一对
            cmd_data, fbk_data = None, None

    elif match_fbk:
        fbk_data = [int(b, 16) for b in match_fbk.group(1).strip().split()]

# 作图
x = list(range(len(cmd_pos)))

plt.figure()
plt.plot(x, cmd_pos, label='Cmd Pos')
plt.plot(x, fbk_pos, label='Fbk Pos')
plt.ylabel('Position [rad]')
plt.legend()
plt.grid()

plt.figure()
plt.plot(x, cmd_vel, label='Cmd Vel')
plt.plot(x, fbk_vel, label='Fbk Vel')
plt.ylabel('Velocity [rad/s]')
plt.legend()
plt.grid()

plt.figure()
plt.plot(x, cmd_torque, label='Cmd Torque')
plt.plot(x, fbk_torque, label='Fbk Torque')
plt.ylabel('Torque [Nm]')
plt.legend()
plt.grid()

plt.show()

