/*********************************************************
*                                                        *
*   Based on:                                            *
*   PrimeSense NiTE 2.0 - User Viewer Sample             *
*                                                        *
*********************************************************/

#ifndef _SKVIEWER_GL_RVIZ_H_
#define _SKVIEWER_GL_RVIZ_H_

#include "NiTE.h"

#include <kinect_v2/markers.hpp>
#include "ros/ros.h"


#define MAX_DEPTH 10000


class SkViewerGlRviz
{
public:
  SkViewerGlRviz(const char* strName, ros::NodeHandle& nh);
  virtual ~SkViewerGlRviz();

  // Initialize the tracker
  virtual openni::Status Init(int argc, char **argv);
  virtual openni::Status Run();	//Does not return

protected:
  virtual void Display();
  // Not in Viewer.cpp
  virtual void DisplayPostDraw(){};	// Overload to draw over the screen image
  virtual void OnKey(unsigned char key, int x, int y);
  virtual openni::Status InitOpenGL(int argc, char **argv);
  void InitOpenGLHooks();
  void Finalize();

private:
  SkViewerGlRviz(const SkViewerGlRviz&);
  SkViewerGlRviz& operator=(SkViewerGlRviz&);

  static SkViewerGlRviz* self_;
  static void glutIdle();
  static void glutDisplay();
  static void glutKeyboard(unsigned char key, int x, int y);

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
  std::vector<BallMarker*> markers_;
  void show_marker(const nite::SkeletonJoint& joint,
                   const unsigned int& marker_id);
  // Joint Position
  Eigen::VectorXd p_;

  // For skeleton lines
  LineListMarker* lines_;
};


#endif
