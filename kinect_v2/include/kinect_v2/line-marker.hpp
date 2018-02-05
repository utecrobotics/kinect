#ifndef _KINECTV2_LINE_MARKER_HPP_
#define _KINECTV2_LINE_MARKER_HPP_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <kinect_v2/marker.hpp>


/**
 * Class to visualize line markers
 *
 */
class LineMarker
  : public Marker
{
public:

  /**
   * Constructor.
   *
   * @param[in] nh ROS node handle
   * @param[in] color indication of color in RGB
   * @param[in] alpha indication of color transparency (alpha)
   * @param[in] scale ball size (scaled from 1m) in m
   */
  LineMarker(ros::NodeHandle& nh,
             double color[3],
             const double& alpha=1.0,
             const double& scale=0.01);

  /**
   * Reset the line: remove all existing segments
   *
   */
  void reset();

  /**
   * Set the color of the ball marker
   *
   * @param[in] color indication of color in RGB
   */
  void setColor(double color[3]);

  // Inherited members
  void setPose(const Eigen::VectorXd& position);
  void publish();

private:
  /// Ball marker message
  visualization_msgs::Marker marker_;
  /// Line marker unique identifier
  static unsigned int id_;
};

#endif
