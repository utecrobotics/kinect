/*********************************************************
*                                                        *
*  Publish topics to BodyArray for NAO teleoperation     *
*  (see the nao_kinect_teleop package)                   *
*                                                        *
*********************************************************/

#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include <iostream>


#include <Eigen/Dense>

#include "kinect_v2/skbody-array.hpp"
#include <kinect_v2/NiteSampleUtilities.h>

#define GL_WIN_SIZE_X 1280
#define GL_WIN_SIZE_Y 1024
#define TEXTURE_SIZE  512


SkBodyArray* SkBodyArray::self_ = NULL;

bool g_drawSkeleton     = true;
bool g_drawCenterOfMass = false;
bool g_drawStatusLabel  = true;
bool g_drawBoundingBox  = false;
bool g_drawBackground   = true;
bool g_drawDepth        = true;
bool g_drawFrameId      = false;
int  g_nXRes = 0;
int  g_nYRes = 0;

// Time to hold in pose to exit program (in milliseconds)
const int g_poseTimeoutToExit = 2000;

// Other constants
#define MAX_USERS 10

bool SHOW_MARKERS = false;

nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};
bool g_visibleUsers[MAX_USERS] = {false};
char g_userStatusLabels[MAX_USERS][100] = {{0}};
char g_generalMessage[100] = {0};

#define USER_MESSAGE(msg) {\
    sprintf(g_userStatusLabels[user.getId()], "%s", msg);               \
    printf("[%08" PRIu64 "] User #%d:\t%s\n", ts, user.getId(), msg);}

int colorCount = 3;
float Colors[][3] = {{1, 0, 0},
                     {0, 1, 0},
                     {0, 0, 1},
                     {1, 1, 1}};


SkBodyArray::SkBodyArray(const char* strName,
                               ros::NodeHandle& nh)
  :
  poseUser_(0),
  nh_(nh)
{
  // Initialize self_ to the current object
  self_ = this;
  // Copy strName to  strName_
  strncpy(strName_, strName, ONI_MAX_STR);
  // New User Tracker
  userTrackerPtr_ = new nite::UserTracker;
  // Resize for positions
  p_.resize(3);
  if (SHOW_MARKERS)
  {
    // Resize the markers size (for all the limbs)
    markers_.resize(6);
    // Initialize the markers
    for (unsigned int i=0; i<markers_.size(); ++i)
    {
      markers_[i] = new BallMarker(nh_, GREEN);
    }
    lines_ = new LineMarker(nh_, GREEN);
  }
    // Create the publisher
  pub_ = nh.advertise<kinect_msgs::BodyArray>("kinect_points", 10);

}


SkBodyArray::~SkBodyArray()
{
  Finalize();
  delete[] pTexMap_;
  self_ = NULL;
}


void SkBodyArray::Finalize()
{
  // Delete the main object
  delete userTrackerPtr_;
  nite::NiTE::shutdown();
  openni::OpenNI::shutdown();
}


openni::Status SkBodyArray::Init(int argc, char **argv)
{
  pTexMap_ = NULL;

  openni::Status rc = openni::OpenNI::initialize();
  if (rc != openni::STATUS_OK)
  {
    printf("Failed to initialize OpenNI\n%s\n",
           openni::OpenNI::getExtendedError());
    return rc;
  }

  const char* deviceUri = openni::ANY_DEVICE;
  for (int i = 1; i < argc-1; ++i)
  {
    if (strcmp(argv[i], "-device") == 0)
    {
      deviceUri = argv[i+1];
      break;
    }
  }

  rc = device_.open(deviceUri);
  if (rc != openni::STATUS_OK)
  {
    printf("Failed to open device\n%s\n", openni::OpenNI::getExtendedError());
    return rc;
  }

  nite::NiTE::initialize();

  // Create and initialize empty user tracker
  if (userTrackerPtr_->create(&device_) != nite::STATUS_OK)
  {
    return openni::STATUS_ERROR;
  }

  // return InitOpenGL(argc, argv);
}


void updateUserState(const nite::UserData& user, uint64_t ts)
{
  if (user.isNew())
  {
    USER_MESSAGE("New");
  }
  else if (user.isVisible() && !g_visibleUsers[user.getId()])
    printf("[%08" PRIu64 "] User #%d:\tVisible\n", ts, user.getId());
  else if (!user.isVisible() && g_visibleUsers[user.getId()])
    printf("[%08" PRIu64 "] User #%d:\tOut of Scene\n", ts, user.getId());
  else if (user.isLost())
  {
    USER_MESSAGE("Lost");
  }
  g_visibleUsers[user.getId()] = user.isVisible();

  if(g_skeletonStates[user.getId()] != user.getSkeleton().getState())
  {
    switch(g_skeletonStates[user.getId()] = user.getSkeleton().getState())
    {
    case nite::SKELETON_NONE:
      USER_MESSAGE("Stopped tracking.")
        break;
    case nite::SKELETON_CALIBRATING:
      USER_MESSAGE("Calibrating...")
        break;
    case nite::SKELETON_TRACKED:
      USER_MESSAGE("Tracking!")
        break;
    case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
    case nite::SKELETON_CALIBRATION_ERROR_HANDS:
    case nite::SKELETON_CALIBRATION_ERROR_LEGS:
    case nite::SKELETON_CALIBRATION_ERROR_HEAD:
    case nite::SKELETON_CALIBRATION_ERROR_TORSO:
      USER_MESSAGE("Calibration Failed... :-|")
        break;
    }
  }
}


void DrawStatusLabel(nite::UserTracker* pUserTracker, const nite::UserData& user)
{
  int color = user.getId() % colorCount;
  float x,y;
  pUserTracker->convertJointCoordinatesToDepth(user.getCenterOfMass().x,
                                               user.getCenterOfMass().y,
                                               user.getCenterOfMass().z, &x, &y);
  x *= GL_WIN_SIZE_X/(float)g_nXRes;
  y *= GL_WIN_SIZE_Y/(float)g_nYRes;
  char *msg = g_userStatusLabels[user.getId()];
}


void DrawFrameId(int frameId)
{
  char buffer[80] = "";
  sprintf(buffer, "%d", frameId);
}


void DrawCenterOfMass(nite::UserTracker* pUserTracker, const nite::UserData& user)
{
  float coordinates[3] = {0};

  pUserTracker->convertJointCoordinatesToDepth(user.getCenterOfMass().x,
                                               user.getCenterOfMass().y,
                                               user.getCenterOfMass().z,
                                               &coordinates[0],
                                               &coordinates[1]);

  coordinates[0] *= GL_WIN_SIZE_X/(float)g_nXRes;
  coordinates[1] *= GL_WIN_SIZE_Y/(float)g_nYRes;
}


void DrawBoundingBox(const nite::UserData& user)
{
  float coordinates[] =
    {
      user.getBoundingBox().max.x, user.getBoundingBox().max.y, 0,
      user.getBoundingBox().max.x, user.getBoundingBox().min.y, 0,
      user.getBoundingBox().min.x, user.getBoundingBox().min.y, 0,
      user.getBoundingBox().min.x, user.getBoundingBox().max.y, 0,
    };
  coordinates[0]  *= GL_WIN_SIZE_X/(float)g_nXRes;
  coordinates[1]  *= GL_WIN_SIZE_Y/(float)g_nYRes;
  coordinates[3]  *= GL_WIN_SIZE_X/(float)g_nXRes;
  coordinates[4]  *= GL_WIN_SIZE_Y/(float)g_nYRes;
  coordinates[6]  *= GL_WIN_SIZE_X/(float)g_nXRes;
  coordinates[7]  *= GL_WIN_SIZE_Y/(float)g_nYRes;
  coordinates[9]  *= GL_WIN_SIZE_X/(float)g_nXRes;
  coordinates[10] *= GL_WIN_SIZE_Y/(float)g_nYRes;

}


void PrintCoordinates(const nite::SkeletonJoint& joint,
                      const std::string& joint_name)
{
  std::cout << joint.getPosition().x << " "
            << joint.getPosition().y << " "
            << joint.getPosition().z << std::endl;
}


void SkBodyArray::show_marker(const nite::SkeletonJoint& joint,
                               const unsigned int& marker_id,
                               const std::string& joint_name)
{
  if (joint.getPositionConfidence() == 1)
  {
    if (SHOW_MARKERS)
    {
      // Convert from mm to m
      p_ <<
        -joint.getPosition().z/1000.0,
        joint.getPosition().x/1000.0,
        joint.getPosition().y/1000.0;
      // Publish the values to the markers
      markers_[marker_id]->setPose(p_);
      markers_[marker_id]->publish();
    }
    // Store data in ROS message
    body_array_.body.at(marker_id).x = -joint.getPosition().z/1000.0;
    body_array_.body.at(marker_id).y = joint.getPosition().x/1000.0;
    body_array_.body.at(marker_id).z = joint.getPosition().y/1000.0;
  }
  else if (joint.getPositionConfidence() < 0.5f)
  {
    // Store data in ROS message
    // body_array_.body.at(marker_id).x = -joint.getPosition().z/1000.0;
    // body_array_.body.at(marker_id).y = joint.getPosition().x/1000.0;
    // body_array_.body.at(marker_id).z = joint.getPosition().y/1000.0;
    return;
  }
  else
  {
    // TODO: maybe change the color of the markers here
    // Store data in ROS message
    body_array_.body.at(marker_id).x = -joint.getPosition().z/1000.0;
    body_array_.body.at(marker_id).y = joint.getPosition().x/1000.0;
    body_array_.body.at(marker_id).z = joint.getPosition().y/1000.0;

    if (SHOW_MARKERS)
    {
      p_ <<
        -joint.getPosition().z/1000.0,
        joint.getPosition().x/1000.0,
        joint.getPosition().y/1000.0;
      markers_[marker_id]->setPose(p_);
      markers_[marker_id]->publish();
    }
    return;
  }
}


void SkBodyArray::DrawLimb(const nite::SkeletonJoint& joint1,
                       const nite::SkeletonJoint& joint2,
                       int color)
{
  float coordinates[6] = {0};
  userTrackerPtr_->convertJointCoordinatesToDepth(joint1.getPosition().x,
                                                  joint1.getPosition().y,
                                                  joint1.getPosition().z,
                                                  &coordinates[0],
                                                  &coordinates[1]);
  userTrackerPtr_->convertJointCoordinatesToDepth(joint2.getPosition().x,
                                                  joint2.getPosition().y,
                                                  joint2.getPosition().z,
                                                  &coordinates[3],
                                                  &coordinates[4]);

  coordinates[0] *= GL_WIN_SIZE_X/(float)g_nXRes;
  coordinates[1] *= GL_WIN_SIZE_Y/(float)g_nYRes;
  coordinates[3] *= GL_WIN_SIZE_X/(float)g_nXRes;
  coordinates[4] *= GL_WIN_SIZE_Y/(float)g_nYRes;

  if (joint1.getPositionConfidence() == 1 && joint2.getPositionConfidence() == 1)
  {
    // For lines in rviz
    if (SHOW_MARKERS)
    {
      lines_->setColor(GREEN);
      p_ <<
        -joint1.getPosition().z/1000.0,
        joint1.getPosition().x/1000.0,
        joint1.getPosition().y/1000.0;
      lines_->setPose(p_);
      p_ <<
        -joint2.getPosition().z/1000.0,
        joint2.getPosition().x/1000.0,
        joint2.getPosition().y/1000.0;
      lines_->setPose(p_);
    }
  }
  else if (joint1.getPositionConfidence() < 0.5f ||
           joint2.getPositionConfidence() < 0.5f)
  {
    return;
  }
  else
  {
    if (SHOW_MARKERS)
    {
      // For lines in rviz
      lines_->setColor(LIGHTGRAY);
      p_ <<
        -joint1.getPosition().z/1000.0,
        joint1.getPosition().x/1000.0,
        joint1.getPosition().y/1000.0;
      lines_->setPose(p_);
      p_ <<
        -joint2.getPosition().z/1000.0,
        joint2.getPosition().x/1000.0,
        joint2.getPosition().y/1000.0;
      lines_->setPose(p_);
    }
  }

  if (joint1.getPositionConfidence() == 1)
  {
  }
  else
  {
  }

  if (joint2.getPositionConfidence() == 1)
  {
  }
  else
  {
  }
}


void SkBodyArray::DrawSkeleton(const nite::UserData& userData)
{
  if (SHOW_MARKERS)
    lines_->reset();

  DrawLimb(userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER),
           userData.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW),
           userData.getId() % colorCount);
  DrawLimb(userData.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW),
           userData.getSkeleton().getJoint(nite::JOINT_LEFT_HAND),
           userData.getId() % colorCount);

  // PrintCoordinates(userData.getSkeleton().getJoint(nite::JOINT_LEFT_HAND),
  //                  "left_hand");

  DrawLimb(userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER),
           userData.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW),
           userData.getId() % colorCount);
  DrawLimb(userData.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW),
           userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND),
           userData.getId() % colorCount);

  DrawLimb(userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER),
           userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER),
           userData.getId() % colorCount);

  DrawLimb(userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER),
           userData.getSkeleton().getJoint(nite::JOINT_TORSO),
           userData.getId() % colorCount);
  DrawLimb(userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER),
           userData.getSkeleton().getJoint(nite::JOINT_TORSO),
           userData.getId() % colorCount);

  if (SHOW_MARKERS)
    lines_->publish();

  // Initialize size of pub_
  body_array_.body.resize(9);

  // Send Values to RVIZ Markers
  show_marker(userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER),
              0, "joint_left_shoulder");
  show_marker(userData.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW),
              1, "joint_left_elbow");
  show_marker(userData.getSkeleton().getJoint(nite::JOINT_LEFT_HAND),
              2, "joint_left_hand");
  show_marker(userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER),
              3, "joint_right_shoulder");
  show_marker(userData.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW),
              4, "joint_right_elbow");
  show_marker(userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND),
              5, "joint_right_hand");
  show_marker(userData.getSkeleton().getJoint(nite::JOINT_HEAD),
              6, "joint_right_hand");
  show_marker(userData.getSkeleton().getJoint(nite::JOINT_NECK),
              7, "joint_right_hand");
  show_marker(userData.getSkeleton().getJoint(nite::JOINT_TORSO),
              8, "joint_right_hand");

  // body_array_.header.stamp = ros::Time::now();
  pub_.publish(body_array_);
}


void SkBodyArray::Display()
{
  openni::VideoFrameRef depthFrame;
  // Pointer to the next frame of data
  nite::UserTrackerFrameRef userTrackerFrame;
  // Generate data for the next frame
  nite::Status rc = userTrackerPtr_->readFrame(&userTrackerFrame);
  if (rc != nite::STATUS_OK)
  {
    printf("GetNextData failed\n");
    return;
  }

  // Get raw depth frame
  depthFrame = userTrackerFrame.getDepthFrame();

  if (pTexMap_ == NULL)
  {
    // Texture map init
    nTexMapX_ = (((depthFrame.getVideoMode().getResolutionX())-1)/
                 (TEXTURE_SIZE)+1)*TEXTURE_SIZE;
    nTexMapY_ = (((depthFrame.getVideoMode().getResolutionY())-1)/
                 (TEXTURE_SIZE)+1)*TEXTURE_SIZE;
    pTexMap_ = new openni::RGB888Pixel[nTexMapX_ * nTexMapY_];
  }

  // Get segmentation of the scene
  const nite::UserMap& userLabels = userTrackerFrame.getUserMap();

  // TODO: is this important?
  if (depthFrame.isValid() && g_drawDepth)
  {
    calculateHistogram(pDepthHist_, MAX_DEPTH, depthFrame);
  }

  memset(pTexMap_, 0, nTexMapX_*nTexMapY_*sizeof(openni::RGB888Pixel));

  float factor[3] = {1, 1, 1};
  // Check if we need to draw depth frame to texture
  if (depthFrame.isValid() && g_drawDepth)
  {
    // Pointer to the segmentation data
    const nite::UserId* pLabels = userLabels.getPixels();

    const openni::DepthPixel* pDepthRow =
      (const openni::DepthPixel*)depthFrame.getData();

    openni::RGB888Pixel* pTexRow = pTexMap_ + depthFrame.getCropOriginY()*nTexMapX_;
    int rowSize = depthFrame.getStrideInBytes()/sizeof(openni::DepthPixel);

    for (int y = 0; y < depthFrame.getHeight(); ++y)
    {
      const openni::DepthPixel* pDepth = pDepthRow;
      openni::RGB888Pixel* pTex = pTexRow + depthFrame.getCropOriginX();

      for (int x = 0; x < depthFrame.getWidth(); ++x, ++pDepth, ++pTex, ++pLabels)
      {
        if (*pDepth != 0)
        {
          if (*pLabels == 0)
          {
            if (!g_drawBackground)
            {
              factor[0] = factor[1] = factor[2] = 0;
            }
            else
            {
              factor[0] = Colors[colorCount][0];
              factor[1] = Colors[colorCount][1];
              factor[2] = Colors[colorCount][2];
            }
          }
          else
          {
            factor[0] = Colors[*pLabels % colorCount][0];
            factor[1] = Colors[*pLabels % colorCount][1];
            factor[2] = Colors[*pLabels % colorCount][2];
          }
          // // Add debug lines - every 10cm
          // else if ((*pDepth / 10) % 10 == 0)
          // {
          //   factor[0] = factor[2] = 0;
          // }

          int nHistValue = pDepthHist_[*pDepth];
          pTex->r = nHistValue*factor[0];
          pTex->g = nHistValue*factor[1];
          pTex->b = nHistValue*factor[2];
          factor[0] = factor[1] = factor[2] = 1;
        }
      }
      pDepthRow += rowSize;
      pTexRow += nTexMapX_;
    }
  }

  g_nXRes = depthFrame.getVideoMode().getResolutionX();
  g_nYRes = depthFrame.getVideoMode().getResolutionY();

  const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
  for (int i = 0; i < users.getSize(); ++i)
  {
    const nite::UserData& user = users[i];

    updateUserState(user, userTrackerFrame.getTimestamp());
    if (user.isNew())
    {
      // Start skeleton tracking
      userTrackerPtr_->startSkeletonTracking(user.getId());
      userTrackerPtr_->startPoseDetection(user.getId(), nite::POSE_CROSSED_HANDS);
    }
    else if (!user.isLost())
    {
      if (g_drawStatusLabel)
        DrawStatusLabel(userTrackerPtr_, user); // TODO: is this important?
      if (g_drawCenterOfMass)
        DrawCenterOfMass(userTrackerPtr_, user); // TODO: is this important?
      if (g_drawBoundingBox)
        DrawBoundingBox(user);  // TODO: Is this important?
      if (users[i].getSkeleton().getState() == nite::SKELETON_TRACKED && g_drawSkeleton)
        DrawSkeleton(user);
    }

    if (poseUser_ == 0 || poseUser_ == user.getId())
    {
      const nite::PoseData& pose = user.getPose(nite::POSE_CROSSED_HANDS);

      if (pose.isEntered())
      {
        // Start timer
        sprintf(g_generalMessage,
                "In exit pose. Keep it for %d second%s to exit\n",
                g_poseTimeoutToExit/1000, g_poseTimeoutToExit/1000 == 1 ? "" : "s");
        printf("Counting down %d second to exit\n", g_poseTimeoutToExit/1000);
        poseUser_ = user.getId();
        poseTime_ = userTrackerFrame.getTimestamp();
      }
      else if (pose.isExited())
      {
        memset(g_generalMessage, 0, sizeof(g_generalMessage));
        printf("Count-down interrupted\n");
        poseTime_ = 0;
        poseUser_ = 0;
      }
      else if (pose.isHeld())
      {
        // tick
        if (userTrackerFrame.getTimestamp() - poseTime_ > g_poseTimeoutToExit * 1000)
        {
          printf("Count down complete. Exit...\n");
          Finalize();
          exit(2);
        }
      }
    }
  }

  if (g_drawFrameId)
  {
    DrawFrameId(userTrackerFrame.getFrameIndex());
  }

  if (g_generalMessage[0] != '\0')
  {
    char *msg = g_generalMessage;
  }

}
