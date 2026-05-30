# CRAWLER
A research repository for a hybrid soft-rigid crawling robot called CRAWLER.

## Build
After cloning and before building, make sure that all the Python node files have executable permission. If not, perform chmod +x to them.

After cloning the repo to your ROS2 workspace, build the codebase for the robot with only open-loop control:

```bash
colcon build --symlink-install --packages-select crawler_bringup crawler_control crawler_msgs crawler_tasks
source install/setup.bash
```
To build the codebase for the robot with feedback (closed-loop) control, build the following additional packages:
```bash
colcon build --symlink-install --packages-select contact_models crawler_controllers deformation_model sensor_fuser teensy_serial_bridge
source install/setup.bash
```

---
