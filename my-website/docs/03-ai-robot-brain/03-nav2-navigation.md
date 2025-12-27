---
sidebar_label: "Nav2 Navigation"
sidebar_position: 3
---

# Nav2 for Bipedal Path Planning

:::tip Learning Objective
Configure the ROS 2 Navigation2 stack for bipedal robot path planning and autonomous navigation.
:::

## What is Nav2?

**Nav2** (Navigation2) is the ROS 2 navigation stack that provides:

```mermaid
graph TB
    subgraph "Nav2 Architecture"
        BT[Behavior Tree] --> PLAN[Planner Server]
        BT --> CTRL[Controller Server]
        BT --> REC[Recovery Server]
        
        PLAN --> GLOBAL[Global Costmap]
        CTRL --> LOCAL[Local Costmap]
        
        MAP[Map Server] --> GLOBAL
        SENSORS[Sensor Data] --> LOCAL
        
        CTRL --> CMD[/cmd_vel]
    end
```

| Component | Function |
|-----------|----------|
| **Planner** | Global path from A to B |
| **Controller** | Follow path, avoid obstacles |
| **Recovery** | Handle stuck situations |
| **Costmaps** | Obstacle representation |

## Installation

```bash
# Install Nav2
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup

# Install dependencies for bipedal robots
sudo apt install ros-humble-robot-localization
```

## Costmap Configuration

### Global Costmap

```yaml
# config/nav2_params.yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      
      # Footprint for bipedal robot
      footprint: "[ [0.15, 0.1], [0.15, -0.1], [-0.15, -0.1], [-0.15, 0.1] ]"
      
      resolution: 0.05
      track_unknown_space: true
      
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
        
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

### Local Costmap

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      
      footprint: "[ [0.15, 0.1], [0.15, -0.1], [-0.15, -0.1], [-0.15, 0.1] ]"
      
      plugins: ["voxel_layer", "inflation_layer"]
      
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        observation_sources: scan
```

## Controller Configuration

### DWB Controller (Differential Drive)

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    
    # Progress checker
    progress_checker_plugin: "progress_checker"
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    
    # Controller
    controller_plugins: ["FollowPath"]
    
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      
      # Velocity limits for bipedal
      min_vel_x: 0.0
      max_vel_x: 0.3      # Conservative for walking
      max_vel_theta: 0.5
      min_speed_xy: 0.0
      max_speed_xy: 0.3
      
      # Acceleration limits
      acc_lim_x: 0.5
      acc_lim_theta: 1.0
      decel_lim_x: -0.5
      
      # Goal tolerance
      xy_goal_tolerance: 0.1
      yaw_goal_tolerance: 0.1
```

:::danger Bipedal Velocity Limits
Humanoid robots are less stable than wheeled robots. Use conservative velocity limits:
```yaml
max_vel_x: 0.3      # Not 1.0!
max_vel_theta: 0.5  # Slow turns
```
:::

## Planner Configuration

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: true
      allow_unknown: true
```

## Recovery Behaviors

```yaml
recoveries_server:
  ros__parameters:
    recovery_plugins: ["spin", "backup", "wait"]
    
    spin:
      plugin: "nav2_recoveries/Spin"
    backup:
      plugin: "nav2_recoveries/BackUp"
    wait:
      plugin: "nav2_recoveries/Wait"
```

## Sending Navigation Goals

### Python Client

```python
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import rclpy

def main():
    rclpy.init()
    
    navigator = BasicNavigator()
    
    # Wait for Nav2 to be ready
    navigator.waitUntilNav2Active()
    
    # Create goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 3.0
    goal_pose.pose.position.y = 2.0
    goal_pose.pose.orientation.w = 1.0
    
    # Navigate!
    navigator.goToPose(goal_pose)
    
    # Monitor progress
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f'Distance remaining: {feedback.distance_remaining:.2f}m')
    
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal reached!')
    else:
        print(f'Navigation failed: {result}')
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch Nav2

```python
# launch/nav2_bringup.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_dir = get_package_share_directory('nav2_bringup')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'map': '/path/to/map.yaml',
                'params_file': '/path/to/nav2_params.yaml'
            }.items()
        ),
    ])
```

---

**Next**: [Room Mapping Deliverable →](./04-room-mapping)
