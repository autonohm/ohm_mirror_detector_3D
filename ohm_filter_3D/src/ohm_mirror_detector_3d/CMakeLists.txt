cmake_minimum_required(VERSION 2.8.3)
project(ohm_mirror_detector_3d)

## Find VTK
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

## Find PCL
FIND_PACKAGE(PCL 1.5 REQUIRED)
## SET FLAGS
SET(CMAKE_CXX_FLAGS "-fopenmp")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wextra -ggdb -O0 -Wunreachable-code")

## Find OPENCV2
FIND_PACKAGE(OpenCV REQUIRED)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
	geometry_msgs
	tf
	nav_msgs
	ohm_hokuyo3d
  visualization_msgs
	message_generation
)

## System dependencies are found with CMake's conventions
FIND_PACKAGE(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
#ADD_MESSAGE_FILES(
#  FILES
#	Coord.msg
#	Dist.msg
#  CoordDist.msg
#	MultiCoordDist.msg
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#GENERATE_MESSAGES(
#  DEPENDENCIES
#  sensor_msgs
#  std_msgs
#	geometry_msgs
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ohm_mirror_detector_3d
  CATKIN_DEPENDS 
		roscpp 
		rospy 
		sensor_msgs 
		std_msgs 
		visualization_msgs 
		tf 
		nav_msgs 
		geometry_msgs 
		ohm_hokuyo3d
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
INCLUDE_DIRECTORIES(
  ${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OPENCV2_INCLUDE_DIRS}
  $ENV{OBVIOUSLY_ROOT}
            ../obcore/math
            ../obcore/filter
            ../obcore
					  /usr/include/eigen3
  ${PROJECT_SOURCE_DIR}/include
)

LINK_DIRECTORIES(
  $ENV{OBVIOUSLY_ROOT}/build/release/obcore  
  $ENV{OBVIOUSLY_ROOT}/build/release/obvision
  $ENV{OBVIOUSLY_ROOT}/build/release/obgraphic
  ${PCL_LIBRARY_DIRS}
  ${PROJECT_SOURCE_DIR}/lib
)

ADD_DEFINITIONS(${PCL_DEFINITIONS})

SET(CORELIBS obcore gsl gslcblas lua5.1)
SET(GRAPHICLIBS obgraphic vtkHybrid glut GL jpeg)
SET(VISIONLIBS obvision ann flann)
SET(OBVIOUSLIBS  obvision obcore	obgraphic)
SET(PCLLIBS)

## Declare a cpp executable
 ADD_EXECUTABLE(ohm_mirror_prefilter_3d src/ohm_mirror_prefilter3D.cpp src/Scan3D.cpp src/Object3D.cpp src/planeFunctions.cpp src/rosfunctions3D.cpp src/reflectionIdentification.cpp src/testFunctions.cpp)
 ADD_DEPENDENCIES(ohm_mirror_prefilter_3d ohm_mirror_prefilter3d_generate_messages_cpp rosfunctions3d)

 target_link_libraries(ohm_mirror_prefilter_3d
   ${catkin_LIBRARIES}
	 ${OpenCV_LIBS}
	 ${GRAPHICLIBS} 
	 ${CORELIBS} 
	 ${VISIONLIBS}
	 ${PCL_LIBRARIES}
 )

 ADD_EXECUTABLE(ohm_2d_extractor src/ohm_2d_extractor.cpp)
 ADD_DEPENDENCIES(ohm_2d_extractor ohm_2d_extractor_generate_messages_cpp)

 target_link_libraries(ohm_2d_extractor
   ${catkin_LIBRARIES}
	 #${OpenCV_LIBS}
	 #${GRAPHICLIBS} 
	 #${CORELIBS} 
	 #${VISIONLIBS}
	 #${PCL_LIBRARIES}
 )

 ADD_EXECUTABLE(ohm_mirror_postfilter_3d src/ohm_mirror_postfilter3D.cpp src/Scan3D.cpp src/Object3D.cpp src/planeFunctions.cpp src/rosfunctions3D.cpp src/reflectionIdentification.cpp src/testFunctions.cpp)
 ADD_DEPENDENCIES(ohm_mirror_postfilter_3d ohm_mirror_postfilter3d_generate_messages_cpp rosfunctions3d)

 target_link_libraries(ohm_mirror_postfilter_3d
   ${catkin_LIBRARIES}
	 ${OpenCV_LIBS}
	 ${GRAPHICLIBS} 
	 ${CORELIBS} 
	 ${VISIONLIBS}
	 ${PCL_LIBRARIES}
 )

#  ADD_EXECUTABLE(ohm_mirror_postfilter_3d src/ohm_mirror_postfilter3D.cpp src/Scan3D.cpp src/Object3D.cpp src/planeFunctions.cpp src/rosfunctions3D.cpp src/reflectionIdentification.cpp src/testFunctions.cpp)
# ADD_DEPENDENCIES(ohm_mirror_postfilter_3d ohm_mirror_postfilter3d_generate_messages_cpp rosfunctions3d)
## Specify libraries to link a library or executable target against

# target_link_libraries(ohm_mirror_postfilter_3d
#   ${catkin_LIBRARIES}
#	 ${OpenCV_LIBS}
#	 ${GRAPHICLIBS} 
#	 ${CORELIBS} 
#	 ${VISIONLIBS}
#	 ${PCL_LIBRARIES}
# )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ohm_echo_filter ohm_echo_filter_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ohm_echo_filter.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
