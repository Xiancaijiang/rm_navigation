# RM 导航系统

一个基于ROS2的RoboMaster机器人导航系统，包含先进的定位、建图和路径规划功能。

## 系统架构


## 主要模块

### 1. 定位模块
- **FAST_LIO**: 轻量快速的激光雷达-惯性里程计
- **Point-LIO**: 基于点云的激光雷达-惯性里程计
- **ICP配准**: 用于地图匹配的点云配准

### 2. 感知模块  
- **地面分割**: 实时地面平面检测
- **点云处理**: 点云滤波和变换
- **IMU滤波**: 用于IMU数据融合的互补滤波器

### 3. 导航模块
- **全局规划器**: 基于A*的全局路径规划
- **局部规划器**: 带动态避障的TEB局部规划器
- **代价地图管理**: 多层代价地图集成

### 4. 仿真模块
- **Gazebo仿真**: 高保真机器人环境仿真
- **RViz可视化**: 导航数据实时可视化

## 安装指南

### 系统依赖
```bash
sudo apt install ros-humble-desktop
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-tf2-*
```

### 项目安装
```bash
# 克隆仓库
git clone https://github.com/Xiancaijiang/RM_SIMULATION.git
cd RM_SIMULATION

# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 编译
colcon build
source install/setup.bash
```

## 系统启动

### 仿真模式
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
world:=RMUL \          # 选择地图 (RMUL/RMUC)
mode:=mapping \        # 运行模式 (mapping/nav)
lio:=fastlio \         # LIO算法 (fastlio/pointlio)
lio_rviz:=False \      # 是否显示LIO点云
nav_rviz:=True \       # 是否显示导航可视化
localization:=icp \    # 定位方式 (icp/amcl/slam_toolbox)
use_sim_time:=True     # 是否使用仿真时间
```

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
world:=RMUL \
mode:=nav \
lio:=fastlio \
localization:=slam_toolbox \
lio_rviz:=False \
nav_rviz:=True
```

### 真实模式
```bash
ros2 launch rm_nav_bringup bringup_real.launch.py \
world:=YOUR_WORLD_NAME \
mode:=mapping  \
lio:=fastlio \
lio_rviz:=False \
nav_rviz:=True
```
```bash
ros2 launch rm_nav_bringup bringup_real.launch.py \
world:=YOUR_WORLD_NAME \
mode:=nav \
lio:=fastlio \
localization:=slam_toolbox \
lio_rviz:=False \
nav_rviz:=True
```
### 启动参数说明

| 参数名称 | 默认值 | 可选值 | 说明 |
|----------|--------|--------|------|
| world | RMUL | RMUL/RMUC | 选择地图文件 |
| mode | mapping | mapping/nav | 运行模式：建图/导航 |
| lio | fastlio | fastlio/pointlio | LIO算法选择 |
| lio_rviz | False | True/False | 是否显示LIO点云 |
| nav_rviz | True | True/False | 是否显示导航可视化 |
| localization | icp | icp/amcl/slam_toolbox | 定位方式选择 |
| use_sim_time | True | True/False | 是否使用仿真时间 |

## 性能优化建议

### 1. 定位优化
- **FAST_LIO参数优化** (`config/fastlio_params.yaml`)
  - 降低`filter_size_surf`和`filter_size_map`值可提高精度但增加计算量
  - 调整`max_iteration`平衡精度和实时性
  - 根据场景调整`cube_side_length`优化内存使用
  - 关闭`runtime_pos_log_enable`和`pcd_save.pcd_save_en`可减少I/O开销

- **ICP配准优化** (`config/icp_registration_sim.yaml`)
  - 根据场景复杂度调整`rough_leaf_size`和`refine_leaf_size`
  - 优化`thresh`阈值提高配准精度
  - 调整`yaw_offset`和`yaw_resolution`提高旋转估计精度

### 2. 感知优化
- **点云处理**
  - 根据场景复杂度调整点云降采样率
  - 优化地面分割参数减少误分割
  - 调整IMU互补滤波器权重提高姿态估计精度

### 3. 导航优化
- **全局规划**
  - 调整代价权重平衡路径长度和安全性
  - 优化地图分辨率提高规划精度

- **局部规划**
  - 调整TEB参数优化轨迹平滑度
  - 优化动态避障参数提高响应速度

### 4. 系统级优化
- 根据硬件性能调整线程池大小
- 优化消息队列长度平衡延迟和内存使用
- 关闭不必要的可视化输出（如`lio_rviz`）减少资源占用

## 代码结构

```
rm_navigation/
├── rm_localization/        # 定位模块
│   ├── FAST_LIO/           # FAST_LIO实现
│   ├── point_lio/          # Point-LIO实现
│   └── icp_registration/   # ICP配准
├── rm_perception/          # 感知模块
│   ├── ground_segmentation/ # 地面分割
│   └── imu_filter/         # IMU滤波
├── rm_navigation/          # 导航模块
│   ├── global_planner/     # 全局规划
│   └── local_planner/      # 局部规划
└── rm_simulation/          # 仿真模块
    ├── gazebo/             # Gazebo仿真
    └── rviz/               # RViz可视化
```

## 致谢
- 感谢Chen的开源代码: https://github.com/LihanChen2004/pb_rm_simulation
