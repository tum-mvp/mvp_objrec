# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(objrec_ros_integration)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp tf geometry_msgs
  visualization_msgs sensor_msgs pcl_ros pcl_conversions resource_retriever
  dynamic_reconfigure nodelet objrec_msgs)

# Build vtk-stl converter
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_executable(vtk_to_stl src/vtk_to_stl.cpp)
target_link_libraries(vtk_to_stl vtkHybrid)

# Add objrec nodelet
include_directories(include ${catkin_INCLUDE_DIRS})

FIND_PACKAGE(CUDA)

# Add objrec node
if(CUDA_FOUND)
  add_definitions(-DUSE_CUDA)
  cuda_add_library(objrecransac_interface src/objrec_interface.cpp)
  cuda_add_library(objrecransac_nodelet src/objrec_nodelet.cpp)
  cuda_add_executable(objrec_node src/objrec_node.cpp)
else()
  add_library(objrecransac_interface src/objrec_interface.cpp)
  add_library(objrecransac_nodelet src/objrec_nodelet.cpp)
  add_executable(objrec_node src/objrec_node.cpp)
endif()

find_library(ObjRecRANSAC_LIBRARY ObjRecRANSAC)

target_link_libraries(objrecransac_interface ${ObjRecRANSAC_LIBRARY} ${VTK_LIBS} vtkHybrid ${catkin_LIBRARIES})
target_link_libraries(objrecransac_nodelet objrecransac_interface)
target_link_libraries(objrec_node objrecransac_interface) 

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS roscpp tf geometry_msgs visualization_msgs sensor_msgs pcl_ros resource_retriever dynamic_reconfigure nodelet objrec_msgs objrec_core
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include
    LIBRARIES # TODO
)
