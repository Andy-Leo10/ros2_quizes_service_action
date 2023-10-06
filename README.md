# ros2_quizes_service_action
practice of service and action in ROS2

#always use in terminal
For compiling
```
cd ~/ros2_ws/ ;colcon build;source install/setup.bash
```
For sourcing only
```
cd ~/ros2_ws/ ;source install/setup.bash
```

##service
```
ros2 launch services_quiz services_quiz_server.launch.py
ros2 service call /rotate services_quiz_srv/srv/Spin "{direction: 'right', angular_velocity: 1.0, time: 5}"
ros2 launch services_quiz services_quiz_client.launch.py
```
##action
```
ros2 launch actions_quiz actions_quiz_server.launch.py
ros2 action send_goal -f /distance_as actions_quiz_msg/action/Distance "{seconds: 2}"
ros2 launch actions_quiz actions_quiz_client.launch.py
```
