
#include <objrec_ros_integration/objrec_nodelet.h>
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(objrec_ros_integration, ObjRecNodelet, objrec_ros_integration::ObjRecNodelet, nodelet::Nodelet);

namespace objrec_ros_integration
{
  void ObjRecNodelet::onInit() 
  {
    NODELET_DEBUG("Initializing nodelet...");

    or_interface.reset(new ObjRecInterface(this->getPrivateNodeHandle()));
    or_interface->start();
  }
}

