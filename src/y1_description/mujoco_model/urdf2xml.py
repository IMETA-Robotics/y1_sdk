import mujoco

# 从URDF文件加载模型
model = mujoco.MjModel.from_xml_path("/home/ubuntu/IMETA_LAB/y1_sdk/src/y1_description/mujoco_model/y1_no_gripper.urdf")

# 将模型保存为MuJoCo XML文件
mujoco.mj_saveLastXML("your_robot.xml", model)