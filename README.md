### 依赖项
ubuntu 20.04  ros-noetic版本

sudo apt update
sudo apt install can-utils net-tools iproute2
sudp apt install ros-noetic-

### CAN接口设置
# ONLY ONE ARM
1. 设置CAN(同一个系统只需要设置一次)
cd scripts/can_scripts/
bash set_only_one_can.sh

2. 启动CAN(之后只需要启动can, 单臂默认为can0)
bash start_can0.sh

这个终端会1s检查一次can是否还存在，如果不存在则会尝试重启can0。比如USB2CAN拔掉再插上会自动重新连上CAN。

也可以将这个终端杀掉，CAN不会断掉，只是若USB2CAN拔掉再插上不会自动重新连接。


# 多条机械臂
核心是设置好rules文件,有几个机械臂就设置几行数据,依次在pc插入每条机械臂的USB2CAN模块,每个设置完再插入下一个, 循环一下1 ,2 步骤: 
1. 设置CAN
cd scripts/can_scripts/
bash search.sh
2. 修改idVendor, idProduct, serial, SYMLINK
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e88", ATTRS{idProduct}=="4603", ATTRS{serial}=="00000000050C", SYMLINK+="imeta_y1_can0"
3. 设置完所有USB2CAN模块后,执行以下脚本:
bash set_rules.sh
4. 启动CAN(有几个机械臂启动几个), example:
bash start_can0.sh
bash start_can1.sh

### 运行驱动 
1. 启动ROS驱动
source devel/setup.bash
roslaunch y1_controller single_arm_controller.launch

使用rostopic list查看话题, 会存在以下话题：
/humanoid/can_driver/motor_states     电机状态反馈
/humanoid/control/control_command     电机控制指令

2. 发布电机控制指令

source devel/setup.bash
cd src/drivers/can_driver/scripts/
bash publish_control_command.sh

在publish_control_command.sh脚本里修改自己的控制参数。

问题:
1. 夹爪是否需要支持 力钜控制？

2. 是否将末端重量传进SDK里，一起做动力学建模？

3. 只给电机下发一帧位置控制，看看效果？

4. 示教器或末端夹具装上后，需要对J6进行零位标定，因此开放J6零位标定和所有关节使能和失能的接口。

5. rviz上可视化关节位置信息，end pose信息。

6. 增加通过/joint_state_publisher_gui控制机械臂。

7. 增加位置控制 和 end pose控制的两个demo。

8. 增加机械臂回零位的接口。

9. 增加停止机械臂，让机械臂会以一个恒定阻尼落下

10. 驱动被杀死后，是不是不需要对电机失能，这样会更安全？
