/*********************************************************
*                                                        *
*                                                        *
*********************************************************/

#ifndef _SKBODY_ARRAY2_H_
#define _SKBODY_ARRAY2_H_

#include "NiTE.h"

#include <kinect_v2/markers.hpp>
#include <kinect_msgs/SkeletonFixedOrder.h>
#include "ros/ros.h"


#define MAX_DEPTH 10000


class SkBodyArray2
{
public:
  SkBodyArray2(const char* strName, ros::NodeHandle& nh);
  virtual ~SkBodyArray2();

  // Initialize the tracker
  virtual openni::Status Init(int argc, char **argv);
  virtual void Display();

protected:
  virtual void DisplayPostDraw(){};	// Overload to draw over the screen image
  void Finalize();

private:
  SkBodyArray2(const SkBodyArray2&);
  SkBodyArray2& operator=(SkBodyArray2&);

  static SkBodyArray2* self_;

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
  std::vector<FrameMarker*> markers_;
  void show_marker(const nite::SkeletonJoint& joint,
                   const unsigned int& marker_id,
                   const std::string& joint_name);
  // Joint Position
  Eigen::VectorXd p_;
  Eigen::VectorXd pl_;
  // For skeleton lines
  LineMarker* lines_;

  // Publish to a given topic
  ros::Publisher pub_;
  kinect_msgs::SkeletonFixedOrder body_array_;
  unsigned int offset_;
};


#endif
