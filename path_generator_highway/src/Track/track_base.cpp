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

#include "Track/track_base.h"

namespace ns_path_generator {

//for trackdrive
void Track::setMap(const fsd_common_msgs::Map &map) { map_ = map; }      

//都用到了
void Track::setState(const VehicleState &state) { state_ = state; }

 //for acceletration
void Track::setEndPoint(const geometry_msgs::Point &endPoint) { endPoint_ = endPoint; } 

//for skidpad
void Track::setTransMat(const Eigen::Matrix4f &transMat) { transMat_ = transMat; }

//for track
void Track::SetMidpoint(const fsd_common_msgs::Trajectory &midpoint){ midPoint_ = midpoint; }

} // namespace ns_path_generator