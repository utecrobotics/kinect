/*
 * Copyright 2018
 * Oscar Ramos.
 * Universidad de Ingenieria y Tecnologia - UTEC
 *
 * This file is part of kinect_v2.
 * kinect_v2 is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * kinect_v2 is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details. You should
 * have received a copy of the GNU Lesser General Public License along
 * with kinect_v2. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _MARKERS_HPP_
#define _MARKERS_HPP_

#include <Eigen/Dense>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


class Marker
{
public:

  Marker(ros::NodeHandle& nh);
  virtual void setPose(const Eigen::VectorXd& position) = 0;
  virtual void publish() = 0;

protected:

  ros::Publisher marker_pub_;
  std::string reference_frame_;
  static unsigned int id_;
};


class LineListMarker
  : public Marker
{
public:
  LineListMarker(ros::NodeHandle& nh,
                 double color[3],
                 const double& scale=0.01);
  void reset();
  void setPose(const Eigen::VectorXd& position);
  void publish();
  void setColor(double color[3]);

private:
  visualization_msgs::Marker marker_;
};

class BallMarker
  : public Marker
{
public:
  BallMarker(ros::NodeHandle& nh,
             double color[3],
             const double& scale=0.05);

  void setPose(const Eigen::VectorXd& position);
  void setColor(double color[3]);
  void publish();

private:

  visualization_msgs::Marker marker_;
};


class FrameMarker
  : public Marker
{
public:
  FrameMarker(ros::NodeHandle& nh,
              const double& color_saturation=1.0,
              const double& scale=0.1);

  void setPose(const Eigen::VectorXd& pose);

  void publish();

private:

  visualization_msgs::Marker markerx_;
  visualization_msgs::Marker markery_;
  visualization_msgs::Marker markerz_;
  static unsigned int id_;
};


#endif
