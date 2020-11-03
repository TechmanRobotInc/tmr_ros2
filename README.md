# __TM ROS Driver__

This driver is for ROS2 Dashing version.

If you want to use Foxy version, please go to our [Foxy version driver](https://github.com/TechmanRobotInc/tmr_ros2).

If you want to know how to use this driver, please go to our [TM ROS1 driver](https://github.com/TechmanRobotInc/tmr_ros1)


## __Demo Code__
There are some demoe codes to show how to use this driver.

> 1. demo_send_script:<br/>
In this demo code, it shows how to send a listen node script to control the robot. <br/>
You can use service named "send_script" to send script.<br/>
"id"->The transaction number expressed in any alphanumeric characters . (Reports the CPERR 04 error if a non-alphanumeric byte is encountered .  When used as a communication packet response , it is a transaction number and identifies which group of commands to respond.<br/>
"script"-> the script which you want to send.<br/>
"ok" -> Correctness of the script.
> 2. demo_ask_item:<br/>
In this demo code, you can send TMSCT cmd by using this service. More details please refer to the TM_Robot_Expression.pdf(version 1.76_6300) Chapter 7.4 TMSCT<br/>
> 3. demo_ask_sta:<br/>
In this demo code, you can send TMSTA cmd by using this service. Please refer to the TM_Robot_Expression.pdf(1.76_6300) 7.5 TMSTA to get more detail<br/>
> 4. demo_connect_tm:<br/>
In this demo code, you can set connection. <br/>
If you set to reconnect as true, every time when driver disconnects from listen node, it will try to re-connect it.<br/>
There are two topics you can use, one is "connect_tmsvr" which is setting ethercad server connection, and the other is "connect_tmsct" which is setting TM-Flow connection.<br/>
> 5. demo_set_event:<br/>
In this demo code, there are six types of events you can use.<br/> 
func: You can use TAG, WAIT_TAG, STOP, PAUSE, RESUME and EXIT<br/>
arg0: if fun is TAG or WAIT_TAG, arg0 is timeout in ms<br/>
arg1: if fun is TAG or WAIT_TAG, arg1 is id<br/>
> 6. demo_set_io:<br/>
In this demo code, you should set module, type, pin and state. The detail you can read the TM_Robot_Expression.pdf(1.76.6300) Chapter 6.5 IO<br/>
module : MODULE_CONTROLBOX or MODULE_ENDEFFECTOR<br/>
type: TYPE_DIGITAL_IN, TYPE_DIGITAL_OUT, TYPE_INSTANT_DO, TYPE_ANALOG_IN, TYPE_ANALOG_OUT, TYPE_INSTANT_AO<br/>
pin: pin number<br/>
state: STATE_OFF or STATE_ON or other value(if digitial IO)<br/>
> 7. demo_set_positions:<br/>
In this demo, you should be careful all units are not degree, they are rad.<br/>
motion_type : PTP_J , PTP_T , LINE_J , LINE_T , CIRC_J ,CIRC_T , PLINE_J ,PLINE_T.  More details please refer to the TM_Robot_Expression.pdf Chapter 9.6-9.9<br/>
positions : target position or target joint(rad)<br/>
velocity : joint velocity-> max value is Pi -> 3.14 rad/s , line velocity ->m/s <br/>
acc_time : to max speed time in millisecond<br/>
blend_percentage : this cannot become 0
fine_goal : if false, when it return in position, but not real in position, you should wait few ms<br/>
> 8. demo_write_item: <br/>
In this demo code, you can send TMSVR cmd by using this service. More details please refer to the TM_Robot_Expression.pdf Chapter(1.76.6300) 9.3 svr_write<br/>