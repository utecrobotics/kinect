/*********************************************************
*                                                        *
*                                                        *
*********************************************************/

#ifndef _SKVIEWER_RVIZ_H_
#define _SKVIEWER_RVIZ_H_

#include "NiTE.h"

#include <kinect_v2/markers.hpp>
#include <kinect_msgs/Skeleton.h>
#include "ros/ros.h"


#define MAX_DEPTH 10000


class SkViewerRviz
{
public:
  SkViewerRviz(const char* strName, ros::NodeHandle& nh);
  virtual ~SkViewerRviz();

  // Initialize the tracker
  virtual openni::Status Init(int argc, char **argv);
  // virtual openni::Status Run();	//Does not return
  virtual void Display();

protected:
  // Not in Viewer.cpp
  virtual void DisplayPostDraw(){};	// Overload to draw over the screen image
  // virtual void OnKey(unsigned char key, int x, int y);
  // virtual openni::Status InitOpenGL(int argc, char **argv);
  // void InitOpenGLHooks();
  void Finalize();

private:
  SkViewerRviz(const SkViewerRviz&);
  SkViewerRviz& operator=(SkViewerRviz&);

  static SkViewerRviz* self_;
  // static void glutIdle();
  // static void glutDisplay();
  // static void glutKeyboard(unsigned char key, int x, int y);

  // Drawings
  void DrawSkeleton(const nite::UserData& userData);
  void DrawLimb(const nite::SkeletonJoint& joint1,
                const nite::SkeletonJoint& joint2,
                int color);

  float                pDepthHist_[MAX_DEPTH];
  char                 strName_[ONI_MAX_STR];
  // Size of texture map
  openni::RGB888Pixel* pTexMap_;
  unsigned int         nTexMapX_;
  unsigned int         nTexMapY_;

  openni::Device       device_;
  // Main Object of User Tracker algorithm: provides access to skeleton
  nite::UserTracker* userTrackerPtr_;

  nite::UserId poseUser_;
  uint64_t poseTime_;

  // ROS node handler
  ros::NodeHandle nh_;

  // Markers
  std::vector<BallMarker> markers_;
  void show_marker(const nite::SkeletonJoint& joint,
                   const unsigned int& marker_id,
                   const std::string& joint_name);
  // Joint Position
  Eigen::VectorXd p_;
  // For skeleton lines
  LineListMarker lines_;

  // Publish to a given topic
  //void publish_data();
  ros::Publisher pub_;
  kinect_msgs::Skeleton skeleton_;
  unsigned int offset_;
};


#endif
