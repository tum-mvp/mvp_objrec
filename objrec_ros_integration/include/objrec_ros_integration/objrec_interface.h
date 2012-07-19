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

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/scoped_ptr.hpp>

namespace objrec_ros_integration {
  class ObjRecInterface {
  public:
    ObjRecInterface(ros::NodeHandle nh = ros::NodeHandle("~"));
    ~ObjRecInterface();

  private:
    void load_models_from_rosparam();
    void add_model(const std::string &model_name, const std::string &model_path);

    void cloud_cb(const sensor_msgs::PointCloud2 &msg);
    void visualization_pub_thread();

    // ROS Structures
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;

    // ObjRec structure
    boost::scoped_ptr<ObjRecRANSAC> objrec_;

    std::list<boost::shared_ptr<UserData> > user_data_list_;
    std::list<vtkSmartPointer<vtkPolyDataReader> > readers_;

    // A vtkPoints structure for accumulating points from the scene cloud
    vtkSmartPointer<vtkPoints> scene_points_;
    std::list<PointSetShape*> detected_models_;

    // ObjRec parameters

    // 'pairwidth' should be roughly half the extent of the visible object
    // part. This means, for each object point p there should be (at least) one
    // point q (from the same object) such that ||p - q|| <= 'pairwidth'.
    // TRADEOFF: smaller values allow for detection in occluded scenes but lead
    // to more imprecise alignment.  Bigger values lead to better alignment but
    // require large visible object parts.
    double pair_width_; // in millimeter

    // 'voxelsize' is the size of the leafs of the octree, i.e., the "size" of
    // the discretization.  TRADEOFF: High values lead to less computation time
    // but ignore object details, e.g., the method could not distinguish
    // between a cylinder and an Amicelli box.  Small values allow to better
    // distinguish between objects, but will introduce more holes in the
    // resulting "voxel-surface" (especially for a sparsely sampled scene) and
    // thus will make normal computation unreliable.  Processing time, of
    // course, will increase with smaller voxel size.
    double voxel_size_;
    
    // 'objectVisibility' is the expected visible object part expressed as
    // fraction of the hole object.  For example 'objectVisibility = 0.1' means
    // that 10% of the object surface is visible in the scene.  Note that the
    // visibility can not be more than 0.5 since a typical camera can not see
    // more than the half of the object.  TRADEOFF: smaller values allow for a
    // detection in occluded scenes but also lead to more false positives since
    // object hypotheses with small alignment with the scene will be accepted
    double object_visibility_;

    // 'relative object size' is the expected fraction of the scene points
    // which belong to an object.  For example a value of 0.05 means that each
    // object represented in the scene will contain at least 5% of all scene
    // points.  TRADEOFF: lower values lead to more computation time and to
    // higher success probability.
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

  };
}

#endif // ifndef __OBJREC_ROS_INTEGRATION_OBJREC_INTERFACE_H
