#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools.realtime_buffer.h>

#include <math.h>
#include <iostream>

#include <bo_msgs/bo_DataGram.h>

class BO_ROBOTHW:public hardware_interface::RobotHW{
public:
	BO_ROBOTHW(ros::Nodehandle nh);
	~BO_ROBOTHW();
	bool start();
	void stop();
	void write();
	void read();
	double get_freq()const; // 10hz 为最佳
	ros::CallbackQueue* getCallbackQueue();

private:
	hardware_interface::JointStateInterface jnt_state_interface_;
	hardware_interface::VelocityJointInterface jnt_vel_interface_;

	int wheel1_code,wheel2_code;
	double odom_x,odom_y,odom_yaw;
	double base_cmd_x,base_cmd_y,base_cmd_yaw;

	double wheel1_vel,wheel2_vel;
	double wheel1_pos,wheel2_pos;
	double wheel1_eff,wheel2_eff;
	double wheel1_cmd,wheel3_cmd;

	ros::Nodehandle nh;

	ros::Publisher cmd_serialPort;

	ros::Subscriber base_cmd_status_sub;
	ros::Subscriber base_cmd_vel_sub;

	void bo_base_cmd_vel_Callback(const bo_msgs::DataGram::ConstPtr&);
	void bo_base_cmd_status_Callback(const std_msgs::string::ConstPtr&);

	void set_vel(double wheel1,double wheel2);
	SerialNode serialnode(_nh, _nh("~"));

}
