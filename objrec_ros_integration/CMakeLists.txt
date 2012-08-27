cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

add_executable(vtk_to_stl src/vtk_to_stl.cpp)
target_link_libraries(vtk_to_stl vtkHybrid)

rosbuild_add_library(${PROJECT_NAME} src/objrec_interface.cpp src/objrec_nodelet.cpp)

rosbuild_find_ros_package(cuda)
include(${cuda_PACKAGE_PATH}/roscuda.cmake)

if(CUDA_FOUND)
  add_definitions(-DUSE_CUDA)
  add_definitions(-DCUDA_DEVICE_ID=1 )

  rosbuild_find_ros_package(objrec_core)
  roscuda_add_executable(objrec_node
    src/objrec_node.cpp
    ${objrec_core_PACKAGE_PATH}/ObjRecRANSAC/src/ObjRecRANSAC/AcceptHypothesisAlgo.cu
    )
else()
  target_link_libraries(${PROJECT_NAME}  ${VTK_LIBS} vtkHybrid)

  rosbuild_add_executable(objrec_node src/objrec_node.cpp)
endif()

target_link_libraries(objrec_node ${PROJECT_NAME} ) 


