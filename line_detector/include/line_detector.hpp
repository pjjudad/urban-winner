/*
    Formula Student Driverless Project (FSD-Project).
    Copyright (c) 2020:
     - chentairan <tairanchen@bitfsd.cn>

    FSD-Project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    FSD-Project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with FSD-Project.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef LINE_DETECTOR_HPP
#define LINE_DETECTOR_HPP

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Path.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/lmeds.h>

namespace ns_line_detector {

class LineDetector {

 public:
  visualization_msgs::Marker line_left, line_right;



  // Constructor
  LineDetector(ros::NodeHandle& nh);

	// Getters
  geometry_msgs::Point getendPoint();

	// Setters
  void setlidarCluster(sensor_msgs::PointCloud2 msgs);


  void publishVisualizationMarkers(const Eigen::VectorXd& left_coefficients, const Eigen::VectorXd& right_coefficients);

  Eigen::Vector2d calculateIntersection(const Eigen::Vector2d& line1, const Eigen::Vector2d& line2);

  geometry_msgs::Point calculateEndPoint(const Eigen::Vector2d& left_line, const Eigen::Vector2d& right_line, double distance);
 
  Eigen::VectorXf fitLineRANSAC(const pcl::PointCloud<pcl::PointXYZI>& points);

  Eigen::VectorXf fitLineLMS(const pcl::PointCloud<pcl::PointXYZI>& points);

  Eigen::Vector2d fitLine(const pcl::PointCloud<pcl::PointXYZI>& points) ;

  void runAlgorithm();



private:

	ros::NodeHandle& nh_;
	
  sensor_msgs::PointCloud2 cluster;
  geometry_msgs::Point end_point;


  bool getPath = false;
  double path_length;
  double allow_angle_error;

  void createPath();
};
}

#endif //LINE_DETECTOR_HPP
