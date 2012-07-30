#ifndef __OBJREC_ROS_INTEGRATION_OBJREC_INTERFACE_H
#define __OBJREC_ROS_INTEGRATION_OBJREC_INTERFACE_H

#include <ObjRecRANSAC/ObjRecRANSAC.h>
#include <ObjRecRANSAC/Shapes/PointSetShape.h>
#include <BasicTools/DataStructures/PointSet.h>
#include <BasicToolsL1/Vector.h>
#include <BasicToolsL1/Matrix.h>
#include <BasicTools/ComputationalGeometry/Algorithms/RANSACPlaneDetector.h>
#include <VtkBasics/VtkWindow.h>
#include <vtkPolyDataWriter.h>
#include <vtkPolyData.h>
#include <vtkCommand.h>
#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataReader.h>
#include <list>
#include <boost/scoped_ptr.hpp>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud2.h>
#include <objrec_msgs/RecognizedObjects.h>
#include <objrec_msgs/ObjRecConfig.h>

namespace objrec_ros_integration {
  class ObjRecInterface {
  public:
    ObjRecInterface(ros::NodeHandle nh = ros::NodeHandle("~"));
    ~ObjRecInterface();

  private:
    void load_models_from_rosparam();
    void add_model(
        const std::string &model_name,
        const std::string &model_uri);

    void reconfigure_cb(objrec_msgs::ObjRecConfig &config, uint32_t level);

    void cloud_cb(const sensor_msgs::PointCloud2 &msg);
    void publish_markers(const objrec_msgs::RecognizedObjects &msg);

    // ROS Structures
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher objects_pub_;
    ros::Publisher markers_pub_;

    // ROS Dynamic Reconfigure
    dynamic_reconfigure::Server<objrec_msgs::ObjRecConfig> reconfigure_server_;

    // ROS Interface parameters
    bool publish_markers_enabled_;

    // ObjRec structure
    boost::scoped_ptr<ObjRecRANSAC> objrec_;

    std::list<boost::shared_ptr<UserData> > user_data_list_;
    std::list<vtkSmartPointer<vtkPolyDataReader> > readers_;
    std::map<std::string,std::string> model_uris_;
    std::map<std::string,std::string> stl_uris_;

    // A vtkPoints structure for accumulating points from the scene cloud
    vtkSmartPointer<vtkPoints> scene_points_;

    // ObjRec parameters (all in millimeter)
    double pair_width_;
    double voxel_size_;
    double object_visibility_;
    double relative_object_size_;
    double relative_number_of_illegal_points_;
    double z_distance_threshold_as_voxel_size_fraction_;
    double normal_estimation_radius_;
    double intersection_fraction_;
    
    // Enable iterative closest point post-processing
    bool icp_post_processing_;
    // This should equal the number of CPU cores
    int num_threads_;

    // All points in the input scene which have z-values bigger than
    // 'maxSceneZValue' will be ignored.  This makes sense only if
    // 'cutDistantScenePoints' = true;
    double max_scene_z_value_;// in millimeter

    // If set to 'false' all scene points will be used for the recognition.
    // However, it makes sense to set it to 'true' since a typical stereo
    // reconstruction gets quite noisy at far distances.
    bool cut_distant_scene_points_;

    // If all objects are on a table and if that table occupies a significant
    // portion of the  scene (at least 20% of the points) than it would make
    // sense to detect the plane and throw its points away.
    bool use_only_points_above_plane_;

    // The desired success probability for object detection. The higher the
    // value the more samples are needed => the more computation time will
    // expire.  TRADEOFF: clear.
    double success_probability_;

    // Plane detection parameters
    double plane_thickness_; // Since real data is noisy the plane is not infinitely thin
    double rel_num_of_plane_points_; // At least 20% of the scene points belong to the plane
  };
}

#endif // ifndef __OBJREC_ROS_INTEGRATION_OBJREC_INTERFACE_H
