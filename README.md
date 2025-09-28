# Software Dependency

  - Ubuntu 22.04 LTS
  - ROS2 Humble

  ```sh
  sudo apt update
  sudo apt install can-utils net-tools iproute2 libgoogle-glog-dev libnlopt-cxx-dev  
  sudo apt install ros-humble-kdl-parser liborocos-kdl-dev 
  ```

## Set single arm Can
### config can device(同一台电脑环境没变化，只需配置一次)
  如果只有一个臂:
  ```sh
  cd scripts/can_scripts/
  bash set_only_one_can.sh
  ```
### 启动can（每次电脑重启 或 拔插can线时）
  ```sh
  bash scripts/can_scripts/start_can0.sh
  ```

## 多臂设置参考一下飞书文档
Refer [C++ SDK 和 ROS2使用说明](https://nxjux7a2aq.feishu.cn/wiki/SSsqwir8ZiIKtMkpJbCcErNCnIc)

## Use ros2 driver(基于c++ sdk封装的ros2, 推荐使用, 不再需要自己调用接口写程序)

### build
  ```sh
  cd y1_sdk/
  bash build.sh
  ```

### 单臂使用（末端：夹爪示教器2合1）
  Note: launch中默认参数can接口为can0， arm_end_type 为3（可根据实际情况修改）
  ```sh
  source install/setup.bash
  ```

  单臂重力补偿模式(采集数据时可用):
  ```sh
  ros2 launch y1_controller one_master.launch.py
  ```

  单臂控制模式(关节位置或末端位姿控制时可用):
  ```sh
  ros2 launch y1_controller single_arm_control.launch.py
  ```

### 双臂使用 （末端：夹爪示教器2合1）
  Note: launch中默认参数can接口为can0、can2， arm_end_type 为3（可根据实际情况修改）
  ```sh
  source install/setup.bash
  ```
  双臂重力补偿模式(采集数据时可用):
  ```sh
  ros2 launch y1_controller two_master.launch.py
  ```
  双臂控制模式(关节位置或末端位姿控制时可用):
  ```sh
  ros2 launch y1_controller two_arm_control.launch.py
  ```

### 一主一从使用（两条臂，一个末端是示教器， 一个末端是夹爪）
  Note: launch中默认参数can接口为can0、can1， 
  主臂arm_end_type为2, 从臂arm_end_type为1 （可根据实际情况修改）
  ```sh
  source install/setup.bash
  ```
  主从摇操模式(采集数据时可用):
  ```sh
  ros2 launch y1_controller one_master_slave.launch.py
  ```
  单个从臂控制模式(关节位置或末端位姿控制时可用):
  ```sh
  ros2 launch y1_controller single_arm_control.launch.py
  ```

### 两主两从使用 （四条臂，两个末端是示教器， 两个末端是夹爪）
  Note: launch中默认参数, 右主can0, 右从can1, 左主can2, 左从can3， 
  两个主臂arm_end_type为2, 两个从臂arm_end_type为1 （可根据实际情况修改）
  ```sh
  source install/setup.bash
  ```
  主从摇操模式(采集数据时可用):
  ```sh
  ros2 launch y1_controller two_master_slave.launch.py
  ```
  双从臂控制模式(关节位置或末端位姿控制时可用):
  ```sh
  ros2 launch y1_controller two_arm_control.launch.py
  ```

## 以下为单臂rostopic控制示例（双臂是分为两个话题，分别下发控制指令）
### joint position control

#### go to joint position [0.6, -0.6, 0.6, 0.5, 0.4, 0]
  ```sh
  source install/setup.bash
  bash scripts/control_scripts/joint_position_control.sh
  ```

#### go zero position [0, 0, 0, 0, 0, 0]
  ```sh
  source install/setup.bash
  bash scripts/control_scripts/go_zero_position.sh
  ```

### end pose control

#### go to end pose [0.0535, -0.0476, 0.3963, -0.3829, -1.0915, 2.5349]
  ```sh
  source install/setup.bash
  bash scripts/control_scripts/end_pose_control.sh
  ```

## 话题数据类型说明 （详细变量含义请查看msg文件内注释）

| msg                     | Description                       |
| ----------------------- | --------------------------------- |
| ArmJointState           | 机械臂关节信息反馈， 如速度、位置、力矩。|  
| ArmStatus               | 机械臂关节状态，如温度、电流           |
| ArmJointPositionControl | 关节位置控制                        |
| ArmEndPoseControl       | 末端位姿控制                        |