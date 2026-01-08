/*
    Formula Student Driverless Project (FSD-Project).
    Copyright (c) 2021:
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

#pragma once
#include "Track/track_base.h"
#include <visualization_msgs/Marker.h>

namespace ns_path_generator {

class Line_Track : public Track {
public:
  bool genTraj();
  bool CalculateTraj(Trajectory &refline);
 // Trajectory getTrajectory() const { return trajectory_; }
 // 添加公共访问方法
 Line_Track(ros::NodeHandle& nh);  // 添加构造函数声明
private:
   // 在Line_Track类定义中添加
  ros::Publisher trajectory_points_pub_;
};

} // namespace ns_path_generator
