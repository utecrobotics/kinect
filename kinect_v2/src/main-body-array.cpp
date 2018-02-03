
#include "ros/ros.h"
#include <Eigen/Dense>

#include <kinect_v2/skbody-array.hpp>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "visualize");
  ros::NodeHandle nh;

  openni::Status rc = openni::STATUS_OK;
  SkBodyArray SkViewer("Viewer printing data", nh);

  rc = SkViewer.Init(argc, argv);
  if (rc != openni::STATUS_OK)
  {
    return 1;
  }

  unsigned int cnt = 1;
  ros::Rate rate(100);
  while (ros::ok())
  {
    SkViewer.Display();

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
