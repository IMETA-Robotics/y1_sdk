### 依赖项
ubuntu 20.04  ros-noetic版本

### CAN接口设置
1. 设置CAN
bash set_can.sh


2. 启动CAN
bash start_can.sh

这个终端会1s检查一次can是否还存在，如果不存在则会尝试重启can0。比如USB2CAN拔掉再插上会自动重新连上CAN。

也可以将这个终端杀掉，CAN不会断掉，只是若USB2CAN拔掉再插上不会自动重新连接。


### 运行驱动 
1. 启动ROS驱动
source devel/setup.bash
roslaunch imeta_y1_controller single_arm_controller.launch

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
