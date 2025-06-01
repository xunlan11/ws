## 机器人软件工程学：turtlebot机器人

### 编译：
- rm -rf build devel
- catkin build
- source ~/ws/catkin_ws/devel/setup.bash

## 克隆kobuki桌面环境
- git clone https://github.com/yujinrobot/kobuki_desktop.git
- rm -rf kobuki_desktop/kobuki_qtestsuite
- git clone --single-branch https://github.com/yujinrobot/kobuki.git
- mv kobuki/kobuki_description kobuki/kobuki_bumper2pc \
   kobuki/kobuki_node kobuki/kobuki_keyop \
   kobuki/kobuki_safety_controller ./
- rm -rf kobuki
- mkdir -p ~/repos/
- cd ~/repos
- git clone https://github.com/yujinrobot/yujin_ocs.git
- cp -r yujin_ocs/yocs_cmd_vel_mux/yujin_ocs/yocs_velocity_smoother yujin_ocs/yocs_controllers ~/catkin_ws/src/

### turtlebot：
- 仿真：roslaunch turtlebot_gazebo turtlebot_world.launch
- 启动底盘：roslaunch turtlebot_bringup minimal.launch
- 键盘控制：roslaunch turtlebot_teleop keyboard_teleop.launch

### 语音：
- 不连续输入，持续输出
- 语音识别：roslaunch robot_voice iat_publish.launch
- 语音控制：roslaunch robot_voice voice_control.launch

### 视觉：
- 注意发布的主题，于Template_Matching.py中修改
- 电脑摄像头（右侧拨钮开启）：roslaunch usb_cam usb_cam-test.launch
- 机器人摄像头：roslaunch freenect_launch freenect-registered-xyzrgb.launch
- 模版匹配：rosrun robot_view Template_Matching.py

### 建图：
- 启动底盘：roslaunch turtlebot_bringup minimal.launch
- 启动GMAPPING建图：roslaunch turtlebot_navigation gmapping_demo.launch
- 启动RViz可视化工具：roslaunch turtlebot_rviz_launchers view_navigation.launch
- 保存地图：rosrun map_server map_saver -f ~/my_maps/my_map
- GMAPPING演示：roslaunch turtlebot_gazebo gmapping_demo.launch

### 导航
- echo "export TURTLEBOT_MAP_FILE=$HOME/my_maps/my_map.yaml" >> ~/.bashrc
- source ~/.bashrc
- 启动底盘（世界时间）：roslaunch turtlebot_bringup minimal.launch use_sim_time:=False 
- 启动amcl定位：roslaunch turtlebot_navigation amcl_demo.launch 
- 启动rviz可视化工具（输出于当前终端）：roslaunch turtlebot_rviz_launchers view_navigation.launch --screen 
- rosrun robot_map navigation.py

### 机械臂
- 检查硬件连接：ls /dev/ttyUSB0
- 设置USB权限：
  - sudo dmesg -c
  - sudo chmod 666 /dev/ttyUSB0
    - udev规则文件路径： /etc/udev/rules.d/70-ttyusb.rules
    - 规则内容： KERNEL=="ttyUSB[0-9]*", MODE="0666"
    - "重启系统"使规则生效
- 启动控制器：roslaunch my_dynamixel start_tilt_controller.launch
- rostopic
- 测试话题：rostopic pub -1 /tilt_controller/command std_msgs/Float64 -- 1.5
