CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(kinect_v2)

FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(GLUT REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(catkin REQUIRED COMPONENTS
  roscpp
  kinect_msgs
)

# Important: change the following 2 paths according to the installation
SET(LIBFREENECT2_PATH $ENV{LIBFREENECT2})
SET(NITE2_PATH $ENV{NITE2})

# Set include and library dirs for ease of use
SET(OPENNI2_PATH ${LIBFREENECT2_PATH}/drivers/OpenNI2-Freenect2Driver)
SET(OPENNI2_INCLUDE_DIRS
  ${OPENNI2_PATH}/extern/OpenNI-Linux-x64-2.2.0.33/Include)
SET(OPENNI2_LIBRARIES
  ${OPENNI2_PATH}/lib/OpenNI2-Freenect2Driver/libFreenect2Driver.so)
SET(NITE2_INCLUDE_DIRS ${NITE2_PATH}/Include)
SET(NITE2_LIBRARIES
  ${NITE2_PATH}/Samples/Bin/libOpenNI2.so
  ${NITE2_PATH}/Samples/Bin/libNiTE2.so
  )

ADD_CUSTOM_TARGET(link_target ALL
  COMMAND ${CMAKE_COMMAND} -E create_symlink
  ${NITE2_PATH}/Samples/Bin/NiTE2 NiTE2)

CATKIN_PACKAGE(
  DEPENDS roscpp kinect_msgs
)

INCLUDE_DIRECTORIES(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${GLUT_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIRS}
  ${OPENNI2_INCLUDE_DIRS}
  ${NITE2_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

ADD_LIBRARY(${PROJECT_NAME}
  src/utils/marker.cpp
  src/utils/ball-marker.cpp
  src/utils/frame-marker.cpp
  src/utils/line-marker.cpp
)

# Viewer using only OpenGL
ADD_EXECUTABLE(mainViewGl
  src/main-view-gl.cpp
  src/utils/skviewer-gl.cpp
)
TARGET_LINK_LIBRARIES(mainViewGl
  ${GLUT_LIBRARY}
  ${OPENGL_LIBRARIES}
  ${OPENNI2_LIBRARIES}
  ${NITE2_LIBRARIES}
)

# Viewer using OpenGL and Rviz
ADD_EXECUTABLE(mainViewGlRviz
  src/main-view-gl-rviz.cpp
  src/utils/skviewer-gl-rviz.cpp
)
TARGET_LINK_LIBRARIES(mainViewGlRviz
  ${GLUT_LIBRARY}
  ${OPENGL_LIBRARIES}
  ${OPENNI2_LIBRARIES}
  ${NITE2_LIBRARIES}
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Viewer using only RViz
ADD_EXECUTABLE(mainViewRviz
  src/main-view-rviz.cpp
  src/utils/skviewer-rviz.cpp
)
TARGET_LINK_LIBRARIES(mainViewRviz
  ${OPENNI2_LIBRARIES}
  ${NITE2_LIBRARIES}
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Publisher of BodyArray messages and show in RViz
ADD_EXECUTABLE(mainBodyArray
  src/main-body-array.cpp
  src/utils/skbody-array.cpp
)
TARGET_LINK_LIBRARIES(mainBodyArray
  ${OPENNI2_LIBRARIES}
  ${NITE2_LIBRARIES}
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Publisher of BodyArray messages and show in RViz
ADD_EXECUTABLE(mainBodyArrayOrient
  src/main-body-array-2.cpp
  src/utils/skbody-array-2.cpp
)
TARGET_LINK_LIBRARIES(mainBodyArrayOrient
  ${OPENNI2_LIBRARIES}
  ${NITE2_LIBRARIES}
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
