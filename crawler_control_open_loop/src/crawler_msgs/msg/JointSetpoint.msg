# Joints order: J1..J7
# J1,J2 (prismatic): position [m], speed [m/s]
# J3..J7 (revolute): position [rad], speed [rad/s]
std_msgs/Header header
float64[7] position
float64[7] speed

# Soft actuator pairs (front/back), each pair symmetric
bool pumps_front_on
bool solenoids_front_on
bool pumps_back_on
bool solenoids_back_on
