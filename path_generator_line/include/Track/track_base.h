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

#include "fsd_common_msgs/Map.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point32.h"
#include "fsd_tools/cubic_spline.h"
#include "std_msgs/Int8.h"
#include "Utils/types.h"
#include "Utils/param.h"

#include <vector>

namespace ns_path_generator {

    class Track {
    public:
        Track() = default;

        ~Track() = default;

        virtual bool genTraj() = 0;   //三种方法用的是不同的  虚函数

        virtual bool CalculateTraj(Trajectory &refline) = 0; //三种方法用的是不同的  虚函数

        void setMap(const fsd_common_msgs::Map &map);  //描述锥桶的，参考系应该是车辆参考系,只有trackdriver用到

        void setState(const VehicleState &state);    //这个都要用到

        void setTransMat(const Eigen::Matrix4f &transMat);     //skidpad 用到

        void setEndPoint(const geometry_msgs::Point &endPoint);  //acceleration才要用到

    protected:
        
        fsd_common_msgs::Map map_;   //for trackdriver  //不管是Track类里面的map_,还是PathGenerator里面的lacal_map_ 实际传入的都是path_generator_handle订阅的 /map 
        VehicleState state_;      //这个应该都要用到
        Trajectory trajectory_;    //a trajectory for trackdriver 这个vector容器里面存放的是TrajectoryPoint。 trackdriver才需要吧
        geometry_msgs::Point endPoint_; // an endpoint for acceleration  终点坐标 x, y, z
        Eigen::Matrix4f transMat_;   //a transMat for skidpad 4x4的矩阵

    };

} // namespace ns_path_generator