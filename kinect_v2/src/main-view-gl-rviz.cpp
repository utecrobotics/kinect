
#include "ros/ros.h"
#include <Eigen/Dense>

#include <kinect_v2/markers.hpp>
#include <kinect_v2/skviewer-gl-rviz.hpp>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "visualize");
  ros::NodeHandle nh;

  openni::Status rc = openni::STATUS_OK;
  SkViewerGlRviz SkViewer("Viewer printing data", nh);

  rc = SkViewer.Init(argc, argv);
  if (rc != openni::STATUS_OK)
  {
    return 1;
  }

  // LineListMarker lineMarkers;
  // double color_green[3] = {0.0, 1.0, 0.0};
  // lineMarkers.init(nh, color_green);
  // Eigen::VectorXd p; p.resize(3);
  // p << 0.0, 0.0, 0.0; 
  // lineMarkers.setPose(p);
  // p << 1.0, 0.0, 0.0; 
  // lineMarkers.setPose(p);

  // p << 0.0, 0.0, 1.0; 
  // lineMarkers.setPose(p);
  // p << 1.0, 0.0, 1.0; 
  // lineMarkers.setPose(p);

  // std::vector<BallMarker> ballMarkers;
  // ballMarkers.resize(2);
  // ballMarkers[0].init(nh, color_green);
  // ballMarkers[1].init(nh, color_green);
  // ballMarkers[0].setPose(p);
  // p << 1.0, 1.0, 1.0; 
  // ballMarkers[1].setPose(p);

  unsigned int cnt = 1;
  ros::Rate rate(10);
  SkViewer.Run();
  std::cout << "111" << std::endl;

  // while (ros::ok())
  // {
  //   if (cnt == 1)
  //   {
  //     cnt = 0;
  //   }

  //   std::cout << "22222" << std::endl;

  //   // ballMarkers[0].publish();
  //   // ballMarkers[1].publish();
  //   // lineMarkers.publish();

  //   ros::spinOnce();
  //   rate.sleep();
  // }

  //ros::spinOnce();

  return 0;
}
