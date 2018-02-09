/*********************************************************************
*                                                                    *
*  Class that publishes messages of type SkeletonFixedOrder to the   *
*  kinect_skeleton topic. This is used for the teleoperation of NAO  *
*  (see the nao_kinect_teleop package)                               *
*                                                                    *
*********************************************************************/

#ifndef _SKFIXED_H_
#define _SKFIXED_H_

#include "NiTE.h"

#include <kinect_v2/markers.hpp>
#include <kinect_msgs/SkeletonFixedOrder.h>
#include "ros/ros.h"


#define MAX_DEPTH 10000


class SkFixed
{
public:

  SkFixed(const char* strName, ros::NodeHandle& nh);
  virtual ~SkFixed();

  // Initialize the tracker
  virtual openni::Status Init(int argc, char **argv);
  virtual void Display();

protected:

  void Finalize();

private:

  SkFixed(const SkFixed&);
  SkFixed& operator=(SkFixed&);

  static SkFixed* self_;

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

  // Ball markers
  std::vector<BallMarker*> ball_markers_;
  void show_marker(const nite::SkeletonJoint& joint,
                   const unsigned int& marker_id,
                   const std::string& joint_name);
  // Line markers (for skeleton lines)
  LineMarker* line_markers_;
  // Frame markers
  std::vector<FrameMarker*> frame_markers_;

  // Temporal storage for joint poses
  Eigen::VectorXd pose_;
  // Temporal storage for joint positions
  Eigen::VectorXd position_;

  // Publish to a given topic
  ros::Publisher pub_;
  kinect_msgs::SkeletonFixedOrder skeleton_msg_;
  unsigned int offset_;
};


#endif
