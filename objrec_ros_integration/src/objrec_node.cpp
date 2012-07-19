
#include <ros/ros.h>
#include <objrec_ros_integration/objrec_interface.h>

using namespace objrec_ros_integration;

int main(int argc, char** argv) {

  ros::init(argc,argv,"objrec_node");

  ObjRecInterface or_interface();

  ros::spin();
  return 0;
}
