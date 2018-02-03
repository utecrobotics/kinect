/*********************************************************
*                                                        *
*   Based on:                                            *
*   PrimeSense NiTE 2.0 - User Viewer Sample             *
*                                                        *
*********************************************************/

#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include <iostream>


#include <GL/glut.h>
#include <Eigen/Dense>

#include "kinect_v2/skviewer-gl-rviz.hpp"
#include <kinect_v2/NiteSampleUtilities.h>

#define GL_WIN_SIZE_X 1280
#define GL_WIN_SIZE_Y 1024
#define TEXTURE_SIZE  512


SkViewerGlRviz* SkViewerGlRviz::self_ = NULL;

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


SkViewerGlRviz::SkViewerGlRviz(const char* strName,
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
  // Resize the markers size (for all the limbs)
  markers_.resize(15);
  // Initialize the markers
  double color_green[3] = {0.0, 1.0, 0.0};
  for (unsigned int i=0; i<markers_.size(); ++i)
  {
    markers_[i].init(nh_, color_green);
  }
  lines_.init(nh_, color_green);
}


SkViewerGlRviz::~SkViewerGlRviz()
{
  Finalize();
  delete[] pTexMap_;
  self_ = NULL;
}


void SkViewerGlRviz::glutIdle()
{
  glutPostRedisplay();
}


void SkViewerGlRviz::glutDisplay()
{
  SkViewerGlRviz::self_->Display();
}


void SkViewerGlRviz::glutKeyboard(unsigned char key, int x, int y)
{
  SkViewerGlRviz::self_->OnKey(key, x, y);
}


void SkViewerGlRviz::Finalize()
{
  // Delete the main object
  delete userTrackerPtr_;
  nite::NiTE::shutdown();
  openni::OpenNI::shutdown();
}


openni::Status SkViewerGlRviz::Init(int argc, char **argv)
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

  return InitOpenGL(argc, argv);
}


openni::Status SkViewerGlRviz::Run()	//Does not return
{
  glutMainLoop();
  return openni::STATUS_OK;
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


#ifndef USE_GLES
void glPrintString(void *font, const char *str)
{
  int i,l = (int)strlen(str);

  for(i=0; i<l; i++)
  {
    glutBitmapCharacter(font,*str++);
  }
}
#endif


void DrawStatusLabel(nite::UserTracker* pUserTracker, const nite::UserData& user)
{
  int color = user.getId() % colorCount;
  glColor3f(1.0f - Colors[color][0],
            1.0f - Colors[color][1],
            1.0f - Colors[color][2]);

  float x,y;
  pUserTracker->convertJointCoordinatesToDepth(user.getCenterOfMass().x,
                                               user.getCenterOfMass().y,
                                               user.getCenterOfMass().z, &x, &y);
  x *= GL_WIN_SIZE_X/(float)g_nXRes;
  y *= GL_WIN_SIZE_Y/(float)g_nYRes;
  char *msg = g_userStatusLabels[user.getId()];
  glRasterPos2i(x-((strlen(msg)/2)*8),y);
  glPrintString(GLUT_BITMAP_HELVETICA_18, msg);
}


void DrawFrameId(int frameId)
{
  char buffer[80] = "";
  sprintf(buffer, "%d", frameId);
  glColor3f(1.0f, 0.0f, 0.0f);
  glRasterPos2i(20, 20);
  glPrintString(GLUT_BITMAP_HELVETICA_18, buffer);
}


void DrawCenterOfMass(nite::UserTracker* pUserTracker, const nite::UserData& user)
{
  glColor3f(1.0f, 1.0f, 1.0f);

  float coordinates[3] = {0};

  pUserTracker->convertJointCoordinatesToDepth(user.getCenterOfMass().x,
                                               user.getCenterOfMass().y,
                                               user.getCenterOfMass().z,
                                               &coordinates[0],
                                               &coordinates[1]);

  coordinates[0] *= GL_WIN_SIZE_X/(float)g_nXRes;
  coordinates[1] *= GL_WIN_SIZE_Y/(float)g_nYRes;
  glPointSize(8);
  glVertexPointer(3, GL_FLOAT, 0, coordinates);
  glDrawArrays(GL_POINTS, 0, 1);
}


void DrawBoundingBox(const nite::UserData& user)
{
  glColor3f(1.0f, 1.0f, 1.0f);

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

  glPointSize(2);
  glVertexPointer(3, GL_FLOAT, 0, coordinates);
  glDrawArrays(GL_LINE_LOOP, 0, 4);
}


void PrintCoordinates(const nite::SkeletonJoint& joint,
                      const std::string& joint_name)
{
  std::cout << joint.getPosition().x << " "
            << joint.getPosition().y << " "
            << joint.getPosition().z << std::endl;
}


void SkViewerGlRviz::show_marker(const nite::SkeletonJoint& joint,
                          const unsigned int& marker_id)
{
  if (joint.getPositionConfidence() == 1)
  {
    // double color_green[3] = {0.0, 0.1, 0.0};

    // Convert from mm to m
    if (false)
    {
      // In the reference system of the Kinect
      p_ <<
        joint.getPosition().x/1000.0,
        joint.getPosition().y/1000.0,
        joint.getPosition().z/1000.0;
    }
    // In the system of RViz
    // markers_[marker_id].setColor(color_green);
    p_ <<
      -joint.getPosition().z/1000.0,
      joint.getPosition().x/1000.0,
      joint.getPosition().y/1000.0;
    // Publish the values to the markers
    markers_[marker_id].setPose(p_);
    markers_[marker_id].publish();
  }
  else if (joint.getPositionConfidence() < 0.5f)
  {
    return;
  }
  else
  {
    // double color_gray[3] = {0.5, 0.5, 0.5};
    // markers_[marker_id].setColor(color_gray);
    p_ <<
      -joint.getPosition().z/1000.0,
      joint.getPosition().x/1000.0,
      joint.getPosition().y/1000.0;
    markers_[marker_id].setPose(p_);
    markers_[marker_id].publish();
  }
}


void SkViewerGlRviz::DrawLimb(const nite::SkeletonJoint& joint1,
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
    glColor3f(1.0f - Colors[color][0],
              1.0f - Colors[color][1],
              1.0f - Colors[color][2]);

    // For lines in rviz
    double color_green[3] = {0.0, 1.0, 0.0};
    lines_.setColor(color_green);
    p_ <<
      -joint1.getPosition().z/1000.0,
      joint1.getPosition().x/1000.0,
      joint1.getPosition().y/1000.0;
    lines_.setPose(p_);
    p_ <<
      -joint2.getPosition().z/1000.0,
      joint2.getPosition().x/1000.0,
      joint2.getPosition().y/1000.0;
    lines_.setPose(p_);
  }
  else if (joint1.getPositionConfidence() < 0.5f ||
           joint2.getPositionConfidence() < 0.5f)
  {
    return;
  }
  else
  {
    glColor3f(.5, .5, .5);

    // For lines in rviz
    double color_gray[3] = {0.5, 0.5, 0.5};
    lines_.setColor(color_gray);
    p_ <<
      -joint1.getPosition().z/1000.0,
      joint1.getPosition().x/1000.0,
      joint1.getPosition().y/1000.0;
    lines_.setPose(p_);
    p_ <<
      -joint2.getPosition().z/1000.0,
      joint2.getPosition().x/1000.0,
      joint2.getPosition().y/1000.0;
    lines_.setPose(p_);

  }
  glPointSize(2);
  glVertexPointer(3, GL_FLOAT, 0, coordinates);
  glDrawArrays(GL_LINES, 0, 2);

  glPointSize(10);
  if (joint1.getPositionConfidence() == 1)
  {
    glColor3f(1.0f - Colors[color][0],
              1.0f - Colors[color][1],
              1.0f - Colors[color][2]);
  }
  else
  {
    glColor3f(.5, .5, .5);
  }
  glVertexPointer(3, GL_FLOAT, 0, coordinates);
  glDrawArrays(GL_POINTS, 0, 1);

  if (joint2.getPositionConfidence() == 1)
  {
    glColor3f(1.0f - Colors[color][0],
              1.0f - Colors[color][1],
              1.0f - Colors[color][2]);
  }
  else
  {
    glColor3f(.5, .5, .5);
  }
  glVertexPointer(3, GL_FLOAT, 0, coordinates+3);
  glDrawArrays(GL_POINTS, 0, 1);
}


void SkViewerGlRviz::DrawSkeleton(const nite::UserData& userData)
{

  lines_.reset();
  DrawLimb(userData.getSkeleton().getJoint(nite::JOINT_HEAD),
           userData.getSkeleton().getJoint(nite::JOINT_NECK),
           userData.getId() % colorCount);

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

  DrawLimb(userData.getSkeleton().getJoint(nite::JOINT_TORSO),
           userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP),
           userData.getId() % colorCount);
  DrawLimb(userData.getSkeleton().getJoint(nite::JOINT_TORSO),
           userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP),
           userData.getId() % colorCount);

  DrawLimb(userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP),
           userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP),
           userData.getId() % colorCount);

  DrawLimb(userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP),
           userData.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE),
           userData.getId() % colorCount);
  DrawLimb(userData.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE),
           userData.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT),
           userData.getId() % colorCount);

  DrawLimb(userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP),
           userData.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE),
           userData.getId() % colorCount);
  DrawLimb(userData.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE),
           userData.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT),
           userData.getId() % colorCount);

  lines_.publish();

  // Send Values to RVIZ Markers
  show_marker(userData.getSkeleton().getJoint(nite::JOINT_HEAD), 0);
  show_marker(userData.getSkeleton().getJoint(nite::JOINT_NECK), 1);
  show_marker(userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER), 2);
  show_marker(userData.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW), 3);
  show_marker(userData.getSkeleton().getJoint(nite::JOINT_LEFT_HAND), 4);
  show_marker(userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), 5);
  show_marker(userData.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW), 6);
  show_marker(userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND), 7);
  show_marker(userData.getSkeleton().getJoint(nite::JOINT_TORSO), 8);
  show_marker(userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), 9);
  show_marker(userData.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE), 10);
  show_marker(userData.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT), 11);
  show_marker(userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), 12);
  show_marker(userData.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE), 13);
  show_marker(userData.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT), 14);

}


void SkViewerGlRviz::Display()
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

  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(0, GL_WIN_SIZE_X, GL_WIN_SIZE_Y, 0, -1.0, 1.0);

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

  glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, nTexMapX_, nTexMapY_, 0, GL_RGB,
               GL_UNSIGNED_BYTE, pTexMap_);

  // Display the OpenGL texture map
  glColor4f(1,1,1,1);

  glEnable(GL_TEXTURE_2D);
  glBegin(GL_QUADS);

  g_nXRes = depthFrame.getVideoMode().getResolutionX();
  g_nYRes = depthFrame.getVideoMode().getResolutionY();

  // upper left
  glTexCoord2f(0, 0);
  glVertex2f(0, 0);
  // upper right
  glTexCoord2f((float)g_nXRes/(float)nTexMapX_, 0);
  glVertex2f(GL_WIN_SIZE_X, 0);
  // bottom right
  glTexCoord2f((float)g_nXRes/(float)nTexMapX_, (float)g_nYRes/(float)nTexMapY_);
  glVertex2f(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
  // bottom left
  glTexCoord2f(0, (float)g_nYRes/(float)nTexMapY_);
  glVertex2f(0, GL_WIN_SIZE_Y);

  glEnd();
  glDisable(GL_TEXTURE_2D);

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
        DrawStatusLabel(userTrackerPtr_, user);
      if (g_drawCenterOfMass)
        DrawCenterOfMass(userTrackerPtr_, user);
      if (g_drawBoundingBox)
        DrawBoundingBox(user);
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
    glColor3f(1.0f, 0.0f, 0.0f);
    glRasterPos2i(100, 20);
    glPrintString(GLUT_BITMAP_HELVETICA_18, msg);
  }

  // Swap the OpenGL display buffers
  glutSwapBuffers();
}


void SkViewerGlRviz::OnKey(unsigned char key, int /*x*/, int /*y*/)
{
  switch (key)
  {
  case 27:
    Finalize();
    exit (1);
  case 's':
    // Draw skeleton?
    g_drawSkeleton = !g_drawSkeleton;
    break;
  case 'l':
    // Draw user status label?
    g_drawStatusLabel = !g_drawStatusLabel;
    break;
  case 'c':
    // Draw center of mass?
    g_drawCenterOfMass = !g_drawCenterOfMass;
    break;
  case 'x':
    // Draw bounding box?
    g_drawBoundingBox = !g_drawBoundingBox;
    break;
  case 'b':
    // Draw background?
    g_drawBackground = !g_drawBackground;
    break;
  case 'd':
    // Draw depth?
    g_drawDepth = !g_drawDepth;
    break;
  case 'f':
    // Draw frame ID
    g_drawFrameId = !g_drawFrameId;
    break;
  }
}


openni::Status SkViewerGlRviz::InitOpenGL(int argc, char **argv)
{
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
  glutCreateWindow (strName_);
  glutSetCursor(GLUT_CURSOR_NONE);

  InitOpenGLHooks();

  glDisable(GL_DEPTH_TEST);
  glEnable(GL_TEXTURE_2D);

  glEnableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_COLOR_ARRAY);

  return openni::STATUS_OK;
}


void SkViewerGlRviz::InitOpenGLHooks()
{
  glutKeyboardFunc(glutKeyboard);
  glutDisplayFunc(glutDisplay);
  glutIdleFunc(glutIdle);
}
