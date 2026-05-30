# CRAWLER
A research repository for a hybrid soft-rigid crawling robot called CRAWLER.

## Repository Structure

```
hybrid-crawler/
├── crawler_control_open_loop/          # ROS2 packages for robot with open-loop control
│   ├── maestro_bridge/                 # Maestro bridge and Maestro YAML parameters
│   └── src/                            
│       ├── crawler_bringup/            # Launch package that starts the full crawler control stack
│       ├── crawler_control/            # Low-level joint control, Maestro serial bridge, reached estimator, and goal-to-setpoint relay
│       ├── crawler_msgs/               # Custom ROS 2 messages and actions, including JointSetpoint and Locomotion
│       └── crawler_tasks/              # Task-space locomotion action server and gait definitions such as forward, backward, left, and right
│
├── crawler_models_and_controllers/     # Model-based control packages for deformation models, contact models, and feedback controllers
│   ├── contact_models/                 # Maps desired/contact-force controller outputs to deformation or pressure increments
│   ├── crawler_controllers/            # PID, bang-bang, pump, force, and pressure-combiner controller nodes
│   └── deformation_model/              # Cable-driven arm inverse kinematics and soft-arm pressure/deformation model nodes
│
├── crawler_sensors/                    # Sensor interface and signal-fusion packages for Teensy-based pressure/contact sensing
│   ├── sensor_fuser/                   # Fuses raw sensor arrays into unified feedback topics for cable, front soft, and rear soft control loops
│   └── teensy_serial_bridge/           # Reads 18 pressure/contact sensor values from Teensy over serial and publishes raw ROS sensor arrays
│
└── README.md                           # Top-level project documentation
```

---


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
