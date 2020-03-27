
std_msgs/Header header

# Movement

float64[] joint_pos
float64[] joint_vel
float64[] joint_tor
float64[] tool0_pose
float64[] tool_pose
float64[] tcp_speed
float64[] tcp_force

# Status

bool robot_link
bool robot_error
bool project_run
bool project_pause
bool camera_light
bool safetyguard_a
bool e_stop

int32 speed_percentage
int32 robot_mode
int32 led_status

# Error

int32 error_code
string error_content

# IO

bool[] cb_digital_input
float32[] cb_analog_input
bool[] ee_digital_input
float32[] ee_analog_input

