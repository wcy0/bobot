#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <math.h>

#include <bo_robothw/bo_robothw.h>

using namespace std;
using namespace bo_robothw;

BO_ROBOTHW::BO_ROBOTHW(const ros::NodeHandle &node):nh(node)
{
  cmd_serialPort = nh.advertise<bo_msgs::bo_DataGram>("SendSerialData", 1000);

  base_cmd_vel_sub = nh.subscribe("base_vel",1000,&BO_ROBOTHW::bo_base_cmd_vel_Callback,this);

  wheel1_vel=wheel2_vel=0.0;
  wheel1_pos=wheel2_pos=0.0;
  wheel1_eff=wheel2_eff=0.0;
  wheel1_cmd=wheel1_cmd=0.0;

  hardware_interface::JointStateHandle state_handle_1("right_wheel_hinge",&wheel1_pos,&wheel1_vel,&wheel1_eff);
  jnt_state_interface_.registerHandle(state_handle_1);

  hardware_interface::JointStateHandle state_handle_2("left_wheel_hinge",&wheel2_pos,&wheel2_vel,&wheel2_eff);
  jnt_state_interface_.registerHandle(state_handle_2);

  registerInterface(&jnt_state_interface_);

  hardware_interface::JointHandle vel_handle_1(jnt_state_interface_.getHandle("right_wheel_hinge"),&wheel1_cmd);
  jnt_vel_interface_.registerHandle(vel_handle_1);

  hardware_interface::JointHandle vel_handle_2(jnt_state_interface_.getHandle("left_wheel_hinge"),&wheel2_cmd);
  jnt_vel_interface_.registerHandle(vel_handle_2);

  registerInterface(&jnt_vel_interface_);
}

BO_ROBOTHW::~BO_ROBOTHW(){
  ros::Duration(1);
}

void BO_ROBOTHW::read(){
  
  ROS_INFO_STREAM("Commands for joints:"<<wheel1_cmd<<","<<wheel2_cmd<<endl);
  speed.left_vel=wheel2_cmd;
  speed.right_vel=wheel1_cmd;
  cout<< "cmd vel"<<wheel2_cmd<<" "<<wheel1_cmd<<endl;
  cmd_serialPort.publish(BO_ROBOTHW::speed);
}

void BO_ROBOTHW::write(){
  wheel1_pos+=wheel1_vel*getPeriod().toSec();
  wheel2_pos+=wheel2_vel*getPeriod().toSec();	
  cout<< "get pos"<<wheel1_pos<<" "<<wheel2_pos<<endl;
}

void BO_ROBOTHW::bo_base_cmd_vel_Callback(const bo_msgs::bo_DataGram msg)
{
	wheel2_vel=msg.left_vel;
	wheel1_vel=msg.right_vel;
	cout<< "get vel"<<wheel1_vel<<" "<<wheel2_vel<<endl;
}

void BO_ROBOTHW::stop()
{
  speed.left_vel=0.0;
  speed.right_vel=0.0;
  cmd_serialPort.publish(BO_ROBOTHW::speed);
}

