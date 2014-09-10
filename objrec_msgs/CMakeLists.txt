# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(objrec_msgs)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation
    dynamic_reconfigure)

# Build vtk-stl converter
generate_dynamic_reconfigure_options(
  cfg/ObjRec.cfg)

add_message_files(
  FILES 
  PointSetShape.msg
  RecognizedObjects.msg)

generate_messages(DEPENDENCIES std_msgs  geometry_msgs)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS roscpp tf geometry_msgs visualization_msgs sensor_msgs pcl_ros resource_retriever dynamic_reconfigure nodelet objrec_msgs objrec_core
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
