### 依赖项
ubuntu 22.04  ros2 humble版本

```sh
sudo apt update
sudo apt install can-utils net-tools iproute2 libgoogle-glog-dev libnlopt-cxx-dev  
sudo apt install ros-humble-kdl-parser liborocos-kdl-dev 
```

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
source install/setup.bash
ros2 launch y1_controller single_arm_control.launch.py

2. 发布电机控制指令
source install/setup.bash
bash scripts/control_scripts/joint_position_control.sh

go_zero:
bash scripts/control_scripts/go_zero_position.sh