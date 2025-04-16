## Slam Toolbox
# How to run
'''
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/ros2_ws/src/slam_toolbox/config/mapper_params_online_async.yaml use_sim_time:=true
'''
Change use_sim_time:=true to false if in real time

# How to run simulation
'''
ros2 launch irobot_create_gazebo_bringup create3_gazebo_aws_small.launch.py 
'''
# How to run keyboard teleoperation
'''
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
'''

# undock

```
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
```
