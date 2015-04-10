
#include <ros/ros.h>
#include <objrec_ros_integration/objrec_interface.h>

using namespace objrec_ros_integration;

int main(int argc, char** argv) {

  ros::init(argc,argv,"objrec_node");

  boost::scoped_ptr<ObjRecInterface> or_interface(new ObjRecInterface());
  or_interface->start();

  ros::spin();
  return 0;
}
