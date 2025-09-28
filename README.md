### 依赖项
  - Ubuntu 20.04 LTS
  - ROS Noetic

  ```sh
  sudo apt update
  sudo apt install can-utils net-tools iproute2 libgoogle-glog-dev
  sudo apt install ros-noetic-kdl-parser liborocos-kdl-dev ros-noetic-urdf ros-noetic-trac-ik
  ```

## Set single arm Can
### config can device(同一台电脑环境没变化，只需配置一次)
  如果只有一个臂:
  ```sh
  cd y1_ros/can_scripts/
  bash set_only_one_can.sh
  ```
### 启动can（每次电脑重启 或 拔插can线时）
  ```sh
  bash can_scripts/start_can0.sh
  ```

## 多臂设置参考一下飞书文档
Refer [C++ SDK 和 ROS1使用说明](https://p05rcnnjwft.feishu.cn/wiki/NfQbwougEi7YSAkW330cW1elnBb)

## Use y1_ros(基于python sdk封装的ros1, 推荐入门使用, 不再需要自己调用接口写程序)

### build
  ```sh
  cd y1_sdk/
  bash build.sh
  ```

### 单臂使用（末端：夹爪示教器2合1）
  Note: launch中默认参数can接口为can0， arm_end_type 为3（可根据实际情况修改）
  ```sh
  source devel/setup.bash
  ```

  单臂重力补偿模式(采集数据时可用):
  ```sh
  roslaunch y1_controller one_master.launch
  ```

  单臂控制模式(关节位置或末端位姿控制时可用):
  ```sh
  roslaunch y1_controller single_arm_control.launch
  ```

### 双臂使用 （末端：夹爪示教器2合1）
  Note: launch中默认参数can接口为can0、can2， arm_end_type 为3（可根据实际情况修改）
  ```sh
  source devel/setup.bash
  ```
  双臂重力补偿模式(采集数据时可用):
  ```sh
  roslaunch y1_controller two_master.launch
  ```
  双臂控制模式(关节位置或末端位姿控制时可用):
  ```sh
  roslaunch y1_controller two_arm_control.launch
  ```

### 一主一从使用（两条臂，一个末端是示教器， 一个末端是夹爪）
  Note: launch中默认参数can接口为can0、can1， 
  主臂arm_end_type为2, 从臂arm_end_type为1 （可根据实际情况修改）
  ```sh
  source devel/setup.bash
  ```
  主从摇操模式(采集数据时可用):
  ```sh
  roslaunch y1_controller one_master_slave.launch
  ```
  单个从臂控制模式(关节位置或末端位姿控制时可用):
  ```sh
  roslaunch y1_controller single_arm_control.launch
  ```

### 两主两从使用 （四条臂，两个末端是示教器， 两个末端是夹爪）
  Note: launch中默认参数, 右主can0, 右从can1, 左主can2, 左从can3， 
  两个主臂arm_end_type为2, 两个从臂arm_end_type为1 （可根据实际情况修改）
  ```sh
  source devel/setup.bash
  ```
  主从摇操模式(采集数据时可用):
  ```sh
  roslaunch y1_controller two_master_slave.launch
  ```
  双从臂控制模式(关节位置或末端位姿控制时可用):
  ```sh
  roslaunch y1_controller two_arm_control.launch
  ```

## 以下为单臂rostopic控制示例（双臂是分为两个话题，分别下发控制指令）
### joint position control

#### go to joint position [0.6, -0.6, 0.6, 0.5, 0.4, 0]
  ```sh
  source devel/setup.bash
  bash control_scripts/joint_position_control.sh
  ```

#### go zero position [0, 0, 0, 0, 0, 0]
  ```sh
  source devel/setup.bash
  bash control_scripts/go_zero_position.sh 
  ```

### end pose control

#### go to end pose [0.0535, -0.0476, 0.3963, -0.3829, -1.0915, 2.5349]
  ```sh
  source devel/setup.bash
  bash control_scripts/end_pose_control.sh
  ```

## 话题数据类型说明 （详细变量含义请查看msg文件内注释）

| msg                     | Description                       |
| ----------------------- | --------------------------------- |
| ArmJointState           | 机械臂关节信息反馈， 如速度、位置、力矩。|  
| ArmStatus               | 机械臂关节状态，如温度、电流           |
| ArmJointPositionControl | 关节位置控制                        |
| ArmEndPoseControl       | 末端位姿控制                        |

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
