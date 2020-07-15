#include <ros/ros.h>
#include "RemoteControl.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "remote_control");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  // create DbwNode class
  remote_control::RemoteControl n(node, priv_nh);

  // handle callbacks until shut down
  
  // ros::spin();

  return 0;
}