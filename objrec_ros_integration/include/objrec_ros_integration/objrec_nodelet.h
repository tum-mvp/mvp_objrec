#ifndef __OBJREC_ROS_INTEGRATION_OBJREC_NODELET
#define __OBJREC_ROS_INTEGRATION_OBJREC_NODELET

#include <nodelet/nodelet.h>
#include <objrec_ros_integration/objrec_interface.h>

namespace objrec_ros_integration
{
  class ObjRecNodelet : public nodelet::Nodelet
  {
  public:
    virtual void onInit();
  private:
    boost::shared_ptr<ObjRecInterface> or_interface;
  };
}

#endif // ifndef __OBJREC_ROS_INTEGRATION_OBJREC_NODELET
