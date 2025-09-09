# 创建 catkin 工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 克隆项目
git clone https://github.com/qfwang23/LO.git

# 解析依赖并编译
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslauch LO lo.launch
