
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

#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <dynamic_reconfigure/server.h>

#include <objrec_msgs/PointSetShape.h>
#include <objrec_msgs/RecognizedObjects.h>
#include <objrec_msgs/ObjRecConfig.h>

#include <objrec_ros_integration/objrec_interface.h>

// Helper function for raising an exception if a required parameter is not found
template <class T>
static void require_param(const ros::NodeHandle &nh, const std::string &param_name, T &var)
{
  if(!nh.getParam(param_name, var)) {
    ROS_FATAL_STREAM("Required parameter not found! Namespace: "<<nh.getNamespace()<<" Parameter: "<<param_name);
    throw ros::InvalidParameterException("Parameter not found!");
  }
}

static void array_to_pose(const double* array, geometry_msgs::Pose &pose_msg)
{
  tf::Matrix3x3 rot_m =  tf::Matrix3x3(
      array[0],array[1],array[2],
      array[3],array[4],array[5],
      array[6],array[7],array[8]);
  tf::Quaternion rot_q;
  rot_m.getRotation(rot_q);
  tf::quaternionTFToMsg(rot_q, pose_msg.orientation);

  pose_msg.position.x = array[9] / 1000.0;
  pose_msg.position.y = array[10] / 1000.0;
  pose_msg.position.z = array[11] / 1000.0;
}

using namespace objrec_ros_integration;

ObjRecInterface::ObjRecInterface(ros::NodeHandle nh) :
  nh_(nh),
  reconfigure_server_(nh),
  publish_markers_enabled_(false),
  n_clouds_per_recognition_(1),
  downsample_voxel_size_(3.5),
  scene_points_(vtkPoints::New(VTK_DOUBLE)),
  time_to_stop_(false)
{
  // Interface configuration
  nh.getParam("publish_markers", publish_markers_enabled_);
  nh.getParam("n_clouds_per_recognition", n_clouds_per_recognition_);
  nh.getParam("downsample_voxel_size", downsample_voxel_size_);

  nh.getParam("x_clip_min", x_clip_min_);
  nh.getParam("x_clip_max", x_clip_max_);
  nh.getParam("y_clip_min", y_clip_min_);
  nh.getParam("y_clip_max", y_clip_max_);
  nh.getParam("z_clip_min", z_clip_min_);
  nh.getParam("z_clip_max", z_clip_max_);

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
  require_param(nh,"num_threads",num_threads_);

	objrec_->setVisibility(object_visibility_);
	objrec_->setRelativeObjectSize(relative_object_size_);
	objrec_->setRelativeNumberOfIllegalPoints(relative_number_of_illegal_points_);
	objrec_->setZDistanceThreshAsVoxelSizeFraction(z_distance_threshold_as_voxel_size_fraction_); // 1.5*params.voxelSize
	objrec_->setNormalEstimationRadius(normal_estimation_radius_);
	objrec_->setIntersectionFraction(intersection_fraction_);
	objrec_->setNumberOfThreads(num_threads_);

  // Get model info from rosparam
  this->load_models_from_rosparam(); 

  // Get additional parameters from ROS
  require_param(nh,"success_probability",success_probability_);
  require_param(nh,"use_only_points_above_plane",use_only_points_above_plane_);

  // Plane detection parameters
  require_param(nh,"plane_thickness",plane_thickness_);
  require_param(nh,"rel_num_of_plane_points",rel_num_of_plane_points_);

  // Construct subscribers and publishers
  cloud_sub_ = nh.subscribe("points", 1, &ObjRecInterface::cloud_cb, this);
  pcl_cloud_sub_ = nh.subscribe("pcl_points", 1, &ObjRecInterface::pcl_cloud_cb, this);
  objects_pub_ = nh.advertise<objrec_msgs::RecognizedObjects>("recognized_objects",20);
  markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>("recognized_objects_markers",20);
  foreground_points_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("foreground_points",10);

  // Set up dynamic reconfigure
  reconfigure_server_.setCallback(boost::bind(&ObjRecInterface::reconfigure_cb, this, _1, _2));

  // Start recognition thread
  recognition_thread_.reset(new boost::thread(boost::bind(&ObjRecInterface::recognize_objects, this)));

  ROS_INFO_STREAM("Constructed ObjRec interface.");
}

ObjRecInterface::~ObjRecInterface() { 
  time_to_stop_ = true;
  recognition_thread_->join();
}

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
    require_param(nh_,"model_uris/"+model_label,model_uris_[model_label]);
    // TODO: make this optional
    require_param(nh_,"stl_uris/"+model_label,stl_uris_[model_label]);

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

void ObjRecInterface::reconfigure_cb(objrec_msgs::ObjRecConfig &config, uint32_t level)
{
  ROS_DEBUG("Reconfigure Request!");
  object_visibility_ = config.object_visibility;
  relative_object_size_ = config.relative_object_size;
  relative_number_of_illegal_points_ = config.relative_number_of_illegal_points;
  z_distance_threshold_as_voxel_size_fraction_ = config.z_distance_threshold_as_voxel_size_fraction; // 1.5*params.voxelSize
  normal_estimation_radius_ = config.normal_estimation_radius;
  intersection_fraction_ = config.intersection_fraction;
  num_threads_ = config.num_threads;

	objrec_->setVisibility(object_visibility_);
	objrec_->setRelativeObjectSize(relative_object_size_);
	objrec_->setRelativeNumberOfIllegalPoints(relative_number_of_illegal_points_);
	objrec_->setZDistanceThreshAsVoxelSizeFraction(z_distance_threshold_as_voxel_size_fraction_); // 1.5*params.voxelSize
	objrec_->setNormalEstimationRadius(normal_estimation_radius_);
	objrec_->setIntersectionFraction(intersection_fraction_);
	objrec_->setNumberOfThreads(num_threads_);

  // Other parameters
  use_only_points_above_plane_ = config.use_only_points_above_plane;
  n_clouds_per_recognition_ = config.n_clouds_per_recognition;
  publish_markers_enabled_ = config.publish_markers;
  downsample_voxel_size_ = config.downsample_voxel_size;

  x_clip_min_ = config.x_clip_min;
  x_clip_max_ = config.x_clip_max;
  y_clip_min_ = config.y_clip_min;
  y_clip_max_ = config.y_clip_max;
  z_clip_min_ = config.z_clip_min;
  z_clip_max_ = config.z_clip_max;
}

void ObjRecInterface::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &points_msg)
{
  // Convert to PCL cloud
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*points_msg, *cloud);

  this->pcl_cloud_cb(cloud);
}

void ObjRecInterface::pcl_cloud_cb(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud)
{
  // Lock the buffer mutex while we're capturing a new point cloud
  boost::mutex::scoped_lock buffer_lock(buffer_mutex_);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clipped(new pcl::PointCloud<pcl::PointXYZRGB>());
  cloud_clipped->header = cloud->header;
  for (int j = 0; j < (int) cloud->points.size(); ++j) {
    if (cloud->points[j].x > x_clip_min_ && cloud->points[j].x < x_clip_max_ &&
        cloud->points[j].y > y_clip_min_ && cloud->points[j].y < y_clip_max_ &&
        cloud->points[j].z > z_clip_min_ && cloud->points[j].z < z_clip_max_) 
    {
      // Add point
      cloud_clipped->push_back(cloud->points[j]);
    } 
  }

  // Store the cloud
  clouds_.push(cloud_clipped);

  // Increment the cloud index
  if(clouds_.size() > (unsigned)n_clouds_per_recognition_) {
    clouds_.pop();
  }

  foreground_points_pub_.publish(cloud_clipped);
}

void ObjRecInterface::recognize_objects() 
{
  while(ros::ok() && !time_to_stop_) {
    // Don't hog the cpu
    ros::Duration(0.03).sleep();

    // Scope for syncrhonization
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_full(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    {
      // Lock the buffer mutex
      boost::mutex::scoped_lock buffer_lock(buffer_mutex_);

      // Continue if the cloud is empty
      if(clouds_.empty()) {
        ROS_WARN("Point cloud empty!");
        continue;
      }

      ROS_INFO_STREAM("Computing objects from "
          <<scene_points_->GetNumberOfPoints()<<" points "
          <<"between "<<(ros::Time::now() - clouds_.back()->header.stamp)
          <<" to "<<(ros::Time::now() - clouds_.front()->header.stamp)<<" seconds after they were acquired.");

      // Copy references to the stored clouds
      cloud_full->header = clouds_.front()->header;

      while(!clouds_.empty()) {
        *cloud_full += *(clouds_.front());
        clouds_.pop();
      }
    }

    ROS_DEBUG_STREAM("Full cloud has "<<cloud_full->points.size());

    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setInputCloud(cloud_full);
    voxel_grid.setLeafSize(downsample_voxel_size_/1000.0,downsample_voxel_size_/1000.0,downsample_voxel_size_/1000.0);
    voxel_grid.filter(*cloud);

    ROS_DEBUG_STREAM("Decimated cloud has "<<cloud->points.size());
    
    // Fill VTK points structure and convert units
    // ObjRec operates in mm
    // Reset the insertion point in the new buffer
    scene_points_->SetNumberOfPoints(cloud->points.size());
    scene_points_->Reset();

    // Require the points are inside of the clopping box
    for (int j = 0; j < (int) cloud->points.size(); ++j) {
      // Add point
      scene_points_->InsertNextPoint(
          cloud->points[j].x * 1000.0,
          cloud->points[j].y * 1000.0,
          cloud->points[j].z * 1000.0);
    }

    if(scene_points_->GetNumberOfPoints() == 0) {
      ROS_WARN("No foreground points!");
      continue;
    }

    // Remove ground plane
    vtkSmartPointer<vtkPoints> background_points(vtkPoints::New(VTK_DOUBLE));
    vtkSmartPointer<vtkPoints> foreground_points(vtkPoints::New(VTK_DOUBLE));
    RANSACPlaneDetector planeDetector;
    //double plane_normal[3], plane_points[3][3];

    if(use_only_points_above_plane_) {
      ROS_INFO("ObjRec: Removing points not above plane...");

      // Perform the plane detection
      planeDetector.detectPlane(scene_points_, rel_num_of_plane_points_, plane_thickness_);
      // Check the orientation of the detected plane normal
      if ( planeDetector.getPlaneNormal()[2] > 0.0 ) {
        planeDetector.flipPlaneNormal();
      }

      // Get plane normal for visualization
      //plane_normal = planeDetector.getPlaneNormal();
      //planeDetector.getPlanePoints(plane_points[0],plane_points[1],plane_points[2]);

      // Get the points above the plane (the scene) and the ones below it (background)
      planeDetector.getPointsAbovePlane(foreground_points, background_points);
    } else {
      foreground_points = scene_points_;
    }

    // Detect models
    std::list<PointSetShape*> detected_models;

    ROS_INFO_STREAM("ObjRec: Attempting recognition...");
    objrec_->doRecognition(foreground_points, success_probability_, detected_models);

    ROS_INFO("ObjRec: Seconds elapsed = %.2lf \n", objrec_->getLastOverallRecognitionTimeSec());
    ROS_INFO("ObjRec: Seconds per hypothesis = %.6lf  \n", objrec_->getLastOverallRecognitionTimeSec()
        / (double) objrec_->getLastNumberOfCheckedHypotheses());

    // Construct recognized objects message
    objrec_msgs::RecognizedObjects objects_msg;
    objects_msg.header = cloud->header;

    for(std::list<PointSetShape*>::iterator it = detected_models.begin();
        it != detected_models.end();
        ++it)
    {
      PointSetShape *detected_model = *it;

      // Construct and populate a message
      objrec_msgs::PointSetShape pss_msg;
      pss_msg.label = detected_model->getUserData()->getLabel();
      pss_msg.confidence = detected_model->getConfidence();
      array_to_pose(detected_model->getRigidTransform(), pss_msg.pose);

      objects_msg.objects.push_back(pss_msg);
      delete *it;
    }

    // Publish the visualization markers
    this->publish_markers(objects_msg);

    // Publish the recognized objects
    objects_pub_.publish(objects_msg);

    // Publish the points used in the scan, for debugging
    /**
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr foreground_points_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
    foreground_points_pcl->header = cloud->header;
    for(unsigned int i=0; i<foreground_points->GetNumberOfPoints(); i++) {
      double point[3];
      foreground_points->GetPoint(i,point);
      foreground_points_pcl->push_back(pcl::PointXYZ(point[0]/1000.0,point[1]/1000.0,point[2]/1000.0));
    }
    foreground_points_pub_.publish(foreground_points_pcl);
    **/
  }
}

void ObjRecInterface::publish_markers(const objrec_msgs::RecognizedObjects &objects_msg)
{
  visualization_msgs::MarkerArray marker_array;
  int id = 0;

  for(std::vector<objrec_msgs::PointSetShape>::const_iterator it = objects_msg.objects.begin();
      it != objects_msg.objects.end();
      ++it)
  {
    visualization_msgs::Marker marker;

    marker.header = objects_msg.header;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(10*it->confidence);
    marker.ns = "objrec";
    marker.id = 0;

    marker.scale.x = 0.001;
    marker.scale.y = 0.001;
    marker.scale.z = 0.001;

    marker.color.a = 0.75;
    marker.color.r = 1.0;
    marker.color.g = 0.1;
    marker.color.b = 0.3;

    marker.id = id++;
    marker.pose = it->pose;
    marker.mesh_resource = stl_uris_[it->label];

    marker_array.markers.push_back(marker);
  }

  // Publish the markers
  markers_pub_.publish(marker_array);
}

