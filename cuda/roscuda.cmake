
rosbuild_find_ros_package(cuda)  
SET(CMAKE_MODULE_PATH "${cuda_PACKAGE_PATH}/FindCuda" ${CMAKE_MODULE_PATH})
FIND_PACKAGE(CUDA)

macro(roscuda_add_executable exe)
  CUDA_ADD_EXECUTABLE(${ARGV})

  # Add explicit dependency of each file on our manifest.xml and those of
  # our dependencies.
  get_target_property(_srclist ${exe} SOURCES)
  foreach(_src ${_srclist})
    set(_file_name _file_name-NOTFOUND)
    find_file(_file_name ${_src} ${CMAKE_CURRENT_SOURCE_DIR} /)
    if(NOT _file_name)
      message("[rosbuild] Couldn't find source file ${_src}; assuming that it is in ${CMAKE_CURRENT_SOURCE_DIR} and will be generated later")
      set(_file_name ${CMAKE_CURRENT_SOURCE_DIR}/${_src})
    endif(NOT _file_name)
    add_file_dependencies(${_file_name} ${ROS_MANIFEST_LIST})
  endforeach(_src)

  rosbuild_add_compile_flags(${exe} ${${PROJECT_NAME}_CFLAGS_OTHER})
  rosbuild_add_link_flags(${exe} ${${PROJECT_NAME}_LDFLAGS_OTHER})

  if(ROS_BUILD_STATIC_EXES AND ${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    # This will probably only work on Linux.  The LINK_SEARCH_END_STATIC
    # property should be sufficient, but it doesn't appear to work
    # properly.
    rosbuild_add_link_flags(${exe} -static-libgcc -Wl,-Bstatic)
  endif(ROS_BUILD_STATIC_EXES AND ${CMAKE_SYSTEM_NAME} STREQUAL "Linux")

  target_link_libraries(${exe} ${${PROJECT_NAME}_LIBRARIES})

  # Add ROS-wide compile and link flags (usually things like -Wall).  These
  # are set in rosconfig.cmake.
  rosbuild_add_compile_flags(${exe} ${ROS_COMPILE_FLAGS})
  rosbuild_add_link_flags(${exe} ${ROS_LINK_FLAGS})

  # Make sure to do any prebuild work (e.g., msg/srv generation) before
  # building this target.
  add_dependencies(${exe} rosbuild_precompile)

  # If we're linking boost statically, we have to force allow multiple definitions because
  # rospack does not remove duplicates
  if ("$ENV{ROS_BOOST_LINK}" STREQUAL "static")
    rosbuild_add_link_flags(${exe} "-Wl,--allow-multiple-definition")
  endif("$ENV{ROS_BOOST_LINK}" STREQUAL "static")
endmacro(roscuda_add_executable)
