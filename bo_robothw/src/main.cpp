#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <bo_robothw/bo_robothw.h>

using namespace bo_robothw;

int main (int argc ,char **argv)
{
  double x,y,theta;

  ros::init(argc,argv,"robothw");
  ros::NodeHandle nh;

  BO_ROBOTHW my_robot(nh);
  ROS_INFO_STREAM("period: " << my_robot.getPeriod().toSec());
  controller_manager::ControllerManager cm(&my_robot,nh);

  ros::Rate rate(30);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (ros::ok()) {

      my_robot.read();
      my_robot.write();
    
      cm.update(my_robot.getTime(), my_robot.getPeriod());
      rate.sleep();
    }

    my_robot.stop();
    spinner.stop();

    return 0;

}
