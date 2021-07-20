
std_msgs/Header header

# Connection Status
bool is_svr_connected       # Ethernet connection
bool is_sct_connected       # Listen Node connection

# Response ROS Node Status
int32 tmsrv_cperr           # TM Ethernet Slave Communication Last CPERR Error Code
int32 tmscript_cperr        # External Script Communication Last CPERR Error Code
int32 tmsrv_dataerr         # TMSVR Communication Last Data Error Code
int32 tmscript_dataerr      # External Script Communication TMSCT (or TMSTA) Last Data Error Code

# Movement
float64[] joint_pos
float64[] joint_vel
float64[] joint_tor
float64[] tool0_pose
float64[] tool_pose
float64[] tcp_speed
float64[] tcp_force

# Torque Data(for TM-Flow 1.84)
float64[] joint_tor_average
float64[] joint_tor_min
float64[] joint_tor_max

# Status
bool robot_link              # Link Robot Controller
bool is_data_table_correct   # Data Table Setting is Correct
bool robot_error             # Error or Not
bool project_run             # Project Runing or not
bool project_pause           # Project Pause or not
bool safetyguard_a           # Safety IO(Safeguard Port A trigger) 
bool e_stop                  # Emergency STOP
bool camera_light            # Light
int32 error_code             # Last Error Code
int32 project_speed          # Project Running Speed
int32 ma_mode                # M/A Mode
int32 robot_light            # Robot light

# IO
uint8[] cb_digital_output
uint8[] cb_digital_input
float32[] cb_analog_output
float32[] cb_analog_input
uint8[] ee_digital_output
uint8[] ee_digital_input
float32[] ee_analog_output
float32[] ee_analog_input

# String
string error_content

