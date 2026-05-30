# crawler_sensors
Two ROS 2 Python packages:
- teensy_serial_bridge: reads 18 sensors from Teensy (USB CDC), publishes arrays
- sensor_fuser: reduces arrays and smooths to scalar feedback topics for controllers

See each package's config for parameters. Build with:
```
colcon build --symlink-install
source install/setup.bash
```
Launch:
```
ros2 launch teensy_serial_bridge teensy_serial_bridge.launch.py
ros2 launch sensor_fuser sensor_fuser.launch.py
```
