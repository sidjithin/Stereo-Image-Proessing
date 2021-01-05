cmake_minimum_required(VERSION 2.8)
project(stereo_image_proc)

find_package(catkin REQUIRED cv_bridge dynamic_reconfigure image_geometry image_proc image_transport message_filters nodelet sensor_msgs stereo_msgs message_generation) 
find_package(Boost REQUIRED COMPONENTS thread)

add_message_files(
   FILES
   BoundingBox.msg
   BoundingBoxes.msg
 )
 
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   
 )

if(cv_bridge_VERSION VERSION_GREATER "1.12.0")
  add_compile_options(-std=c++11)
endif()

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/Disparity.cfg)

catkin_package(
  CATKIN_DEPENDS image_geometry image_proc sensor_msgs stereo_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime
)

include_directories(include)

find_package(OpenCV 4 REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# See note in image_proc/CMakeLists.txt
add_definitions(-DOPENCV_TRAITS_ENABLE_DEPRECATED)

# Nodelet library
add_library(${PROJECT_NAME} src/libstereo_image_proc/processor.cpp src/libstereo_image_proc/disparity.cpp src/libstereo_image_proc/point_cloud2.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
                                      ${OpenCV_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Standalone node
add_executable(stereoimageproc_exe1 src/nodes/stereo_image_proc.cpp)
add_dependencies(stereoimageproc_exe1 stereo_image_proc_generate_messages_cpp)

target_link_libraries(stereoimageproc_exe1 stereo_image_proc)
SET_TARGET_PROPERTIES(stereoimageproc_exe1 PROPERTIES OUTPUT_NAME stereo_image_proc2)
install(TARGETS stereoimageproc_exe1
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install the launch file
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

# install the include directory
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
