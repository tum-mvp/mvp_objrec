
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
#include <list>

#include <resource_retriever/retriever.h>

#include <ros/ros.h>
#include <ros/exceptions.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <objrec_ros_integration/objrec_interface.h>

#include <sensor_msgs/PointCloud2.h>
#include <limits>

#include <objrec_msgs/PointSetShape.h>
#include <objrec_msgs/RecognizedObjects.h>

// Helper function for raising an exception if a required parameter is not found
template <class T>
static void require_param(const ros::NodeHandle &nh, const std::string &param_name, T &var)
{
  if(!nh.getParam(param_name, var)) {
    ROS_FATAL_STREAM("Required parameter not found! Namespace: "<<nh.getNamespace()<<" Parameter: "<<param_name);
    throw ros::InvalidParameterException("Parameter not found!");
  }
}

using namespace objrec_ros_integration;

ObjRecInterface::ObjRecInterface(ros::NodeHandle nh) :
  nh_(nh),
  scene_points_(vtkPoints::New(VTK_DOUBLE))
{
  // Get construction parameters from ROS & construct object recognizer
  require_param(nh,"pair_width",pair_width_);
  require_param(nh,"voxel_size",voxel_size_);
  
  objrec_.reset(new ObjRecRANSAC(pair_width_, voxel_size_, 0.5));

  // Get post-construction parameters from ROS
  require_param(nh,"object_visibility",object_visibility_);
  require_param(nh,"relative_object_size",relative_object_size_);
  require_param(nh,"relative_number_of_illegal_points",relative_number_of_illegal_points_);
  require_param(nh,"z_distance_threshold_as_voxel_size_fraction",z_distance_threshold_as_voxel_size_fraction_);
  require_param(nh,"normal_estimation_radius",normal_estimation_radius_);
  require_param(nh,"intersection_fraction",intersection_fraction_);
  require_param(nh,"icp_post_processing",icp_post_processing_);
  require_param(nh,"num_threads",num_threads_);

	objrec_->setVisibility(object_visibility_);
	objrec_->setRelativeObjectSize(relative_object_size_);
	objrec_->setRelativeNumberOfIllegalPoints(relative_number_of_illegal_points_);
	objrec_->setZDistanceThreshAsVoxelSizeFraction(z_distance_threshold_as_voxel_size_fraction_); // 1.5*params.voxelSize
	objrec_->setNormalEstimationRadius(normal_estimation_radius_);
	objrec_->setIntersectionFraction(intersection_fraction_);
	//objrec_->setICPPostProcessing(icp_post_processing_);//FIXME: this is
  //unimplemented
	objrec_->setNumberOfThreads(num_threads_);

  // Get model info from rosparam
  this->load_models_from_rosparam(); 

  // Get additional parameters from ROS
  require_param(nh,"success_probability",success_probability_);
  require_param(nh,"max_scene_z_value",max_scene_z_value_);
  require_param(nh,"use_only_points_above_plane",use_only_points_above_plane_);
  require_param(nh,"cut_distant_scene_points",cut_distant_scene_points_);

  // Construct pointclound subscriber
  cloud_sub_ = nh.subscribe("points", 1, &ObjRecInterface::cloud_cb, this);
  objects_pub_ = nh.advertise<objrec_msgs::RecognizedObjects>("recognized_objects",5);

  ROS_INFO_STREAM("Constructed ObjRec interface.");
}

ObjRecInterface::~ObjRecInterface() { }

void ObjRecInterface::load_models_from_rosparam()
{
  ROS_INFO_STREAM("Loading models from rosparam...");

  // Get the list of model param names
  XmlRpc::XmlRpcValue objrec_models_xml;
  nh_.param("models", objrec_models_xml, objrec_models_xml);

  // Iterate through the models 
  for(int i =0; i < objrec_models_xml.size(); i++) {
    std::string model_label = static_cast<std::string>(objrec_models_xml[i]);

    // Get the mesh uri & store it
    require_param(nh_,"model_paths/"+model_label,model_uris_[model_label]);

    // Add the model
    this->add_model(model_label, model_uris_[model_label]);
  }
}

void ObjRecInterface::add_model(
    const std::string &model_label,
    const std::string &model_uri)
{
  ROS_INFO_STREAM("Adding model \""<<model_label<<"\" from "<<model_uri);
  // Fetch the model data with a ros resource retriever
  resource_retriever::Retriever retriever;
  resource_retriever::MemoryResource resource;

  try {
    resource = retriever.get(model_uri); 
  } catch (resource_retriever::Exception& e) {
    ROS_ERROR_STREAM("Failed to retrieve \""<<model_label<<"\" model file from \""<<model_uri<<"\" error: "<<e.what());
    return;
  }

  // Load the model into objrec
  vtkSmartPointer<vtkPolyDataReader> reader =
    vtkSmartPointer<vtkPolyDataReader>::New();
  // This copies the data from the resource structure into the polydata reader
  reader->SetBinaryInputString(
      (const char*)resource.data.get(),
      resource.size);
  reader->ReadFromInputStringOn();
  reader->Update();
  readers_.push_back(reader);
  
  // Create new model user data
  boost::shared_ptr<UserData> user_data(new UserData());
  user_data->setLabel(model_label.c_str());
  user_data_list_.push_back(user_data);

  // Add the model to the model library
  objrec_->addModel(reader->GetOutput(), user_data.get());
}


void ObjRecInterface::cloud_cb(const sensor_msgs::PointCloud2 &msg) {
  ROS_INFO("Received point cloud.");

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(msg, cloud);

  scene_points_->SetNumberOfPoints( cloud.points.size() );
  scene_points_->Reset();

  // Fill VTK points structure and convert units
  for (int j = 0; j < (int) cloud.points.size(); ++j) {
    if (cloud.points[j].x > -0.35) {
      scene_points_->InsertNextPoint(
          cloud.points[j].x * 1000.0,
          cloud.points[j].y * 1000.0,
          cloud.points[j].z * 1000.0);
    } 
  }

  // Detect models
  std::list<PointSetShape*> detected_models;

  ROS_INFO_STREAM("ObjRec: Attempting recognition...");
  objrec_->doRecognition(scene_points_, success_probability_, detected_models);

  ROS_INFO("ObjRec: Seconds elapsed = %.2lf \n", objrec_->getLastOverallRecognitionTimeSec());
  ROS_INFO("ObjRec: Seconds per hypothesis = %.6lf  \n", objrec_->getLastOverallRecognitionTimeSec()
      / (double) objrec_->getLastNumberOfCheckedHypotheses());

  // Construct recognized objects message
  objrec_msgs::RecognizedObjects objects_msg;

  for(std::list<PointSetShape*>::iterator it = detected_models.begin();
      it != detected_models.end();
      ++it)
  {
    objrec_msgs::PointSetShape pss_msg;
    pss_msg.label = (*it)->getUserData()->getLabel();

    objects_msg.objects.push_back(pss_msg);
    delete *it;
  }

  // Publish the recognized objects
  objects_pub_.publish(objects_msg);
}
