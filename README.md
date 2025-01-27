# RM Navigation System

A comprehensive ROS2-based navigation system for RoboMaster robots, featuring advanced localization, mapping, and path planning capabilities.

## System Architecture

![System Architecture](docs/architecture.png) (Note: Add architecture diagram later)

## Key Modules

### 1. Localization
- **FAST_LIO**: Lightweight and fast LiDAR-inertial odometry
- **Point-LIO**: Point cloud based LiDAR-inertial odometry  
- **ICP Registration**: Point cloud registration for map matching

### 2. Perception
- **Ground Segmentation**: Real-time ground plane detection
- **Point Cloud Processing**: Point cloud filtering and transformation
- **IMU Filtering**: Complementary filter for IMU data fusion

### 3. Navigation
- **Global Planner**: A* based global path planning
- **Local Planner**: TEB local planner with dynamic obstacle avoidance
- **Costmap Management**: Multi-layer costmap integration

### 4. Simulation
- **Gazebo Simulation**: High-fidelity robot and environment simulation
- **RViz Visualization**: Real-time visualization of navigation data

## Installation

```bash
# Clone repository
git clone https://github.com/Xiancaijiang/RM_SIMULATION.git
cd RM_SIMULATION

# Build with colcon
colcon build
source install/setup.bash
```

## Running the System

### Simulation Mode
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py
```

### Real Robot Mode  
```bash
ros2 launch rm_nav_bringup bringup_real.launch.py
```

## Performance Optimization

1. **Localization Optimization**
   - Tune FAST_LIO parameters in `config/fastlio_params.yaml`
   - Adjust ICP registration thresholds in `config/icp.yaml`
   - Optimize IMU filtering parameters

2. **Perception Optimization**  
   - Configure ground segmentation parameters
   - Optimize point cloud downsampling rates
   - Tune IMU complementary filter weights

3. **Navigation Optimization**
   - Adjust global planner cost weights
   - Tune local planner trajectory parameters  
   - Optimize costmap layer configurations

## Acknowledgments
- Thanks to Chen's open source code: https://github.com/LihanChen2004/pb_rm_simulation
