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

#include "Track/line_track.h"
#include "ros/ros.h"


//都是在全局坐标系下计算的

namespace ns_path_generator {

    bool Line_Track::genTraj() {
        const double interval = param_.interval;
        const double desire_vel = param_.desire_vel;

        double distance = std::hypot(endPoint_.x, endPoint_.y);
         //gradient  斜率 坡度
        double gradient = endPoint_.y / endPoint_.x;     //规划出的直线的斜率

        TrajectoryPoint tmp_pt;
        trajectory_.clear();
        if (interval == 0){   //间隔
            ROS_ERROR("Interval is too small.");
            return false;
        }
        for (double i = 0; i < distance; i += interval) {  

            //根据弦长，以斜边比例计算临边长      弦长=│x1-x2│√(k^2+1)=│y1-y2│√[(1/k^2)+1]   
            tmp_pt.pts.x = i / std::hypot(1.0, gradient);    //hypot 输入直角三角形的两条直角边计算出斜边长
            tmp_pt.pts.y = tmp_pt.pts.x * gradient;        //y = kx
            tmp_pt.yaw = atan2(tmp_pt.pts.y, tmp_pt.pts.x);  //这里的yaw是直接通过坐标算出来的
            tmp_pt.curvature = 0;      //为什么可以直接等于0？因为这个是直线工况呀
            tmp_pt.velocity = desire_vel;   //这里的控制车辆的实际期望速度就是直线上的理想速度
            tmp_pt.r = 0;          //为什么可以直接等于0？
            trajectory_.push_back(tmp_pt);
        }
        return true;
    }

    bool Line_Track::CalculateTraj(Trajectory &refline) {

        if (trajectory_.empty()) {
            ROS_WARN("Trajectory is empty !");
            return false;
        }
        refline.clear();

        double px = state_.x;
        double py = state_.y;
        double psi = state_.yaw;
        
        //如果生成的轨迹点中有最小距离为负数的，就认为没有规划路径成功
        int count = 0;
        int index_min = -1;
        double min = 999;
        for (size_t i = 0; i < trajectory_.size(); i++) {
            double delta_x = trajectory_[i].pts.x - px;
            double delta_y = trajectory_[i].pts.y - py;
            double dist = std::hypot(delta_x, delta_y);
            if (dist < min) {
                min = dist;
                index_min = i;   //记录两个轨迹点的最小距离
            }
        }
        if (index_min < 0)   
            return false;


        const double desired_velocity = param_.desire_vel;
        const int N = param_.N;
        const double dt = param_.dt;
        TrajectoryPoint tmp;
        double v = std::fmax(state_.v, param_.initial_velocity);  //在 初始速度 和 当前速度 中取 大的值 作为当前轨迹点的期望速度

        int ii = 0;
        double s_tmp = 0;  
        for (int i = 0; i < N; i++) {

            ii =  ii + 1;
            if(ii >= 40)
            {//printf("ok");
            return false;}
   
            if(N==40){
                //printf("i=%d\n",i);
                }

            if (i != 0)
                s_tmp += v * dt;  

            int index = int(s_tmp / param_.interval);    

            //不超过trajcctory_的最后一个
            int next = index + index_min > trajectory_.size() ? trajectory_.size() - 1 : index + index_min;
            
            //计算x，y，yaw的偏差值，
            double delta_x = trajectory_[next].pts.x - px;
            double delta_y = trajectory_[next].pts.y - py;
            double delta_yaw = trajectory_[next].yaw - psi;

            //限制转向角的范围
            while ((delta_yaw) >= M_PI)     //当偏航角偏差大于pi时，要转化为0~(-pi)之间的值
                delta_yaw -= M_PI * 2.0;
            while ((delta_yaw) <= -1.0 * M_PI)  //当偏航角偏差小于-pi时，要转化为0~(pi)之间的值
                delta_yaw += M_PI * 2.0;
            
            // 根据state_.yaw生成新的轨迹点
            double temp_x, temp_y;   
            temp_x = delta_x * cos(psi) + delta_y * sin(psi);  //x cos() + y sin()
            temp_y = delta_y * cos(psi) - delta_x * sin(psi);  // y cos（） - x sin（）  这个为什么？

            tmp.curvature = fabs(trajectory_[next].curvature);  //曲率

            //现在的x，y和yaw 是考虑了车辆位姿的
            tmp.pts = cv::Point2f(temp_x, temp_y);
            tmp.yaw = delta_yaw;  //

            //param_.max_lat_acc是最大横向加速度  sqrt() 返回输入值的非负数平方根
            //在 最大横向加速度所限制的速度 和 先验期望速度 之间选个比较小的速度作为轨迹期望速度
            tmp.velocity = std::min(sqrt(param_.max_lat_acc / tmp.curvature), desired_velocity);  
            refline.push_back(tmp);    //最后都是放到refline里，实际控制使用的轨迹，基于这个生成的/planning/ref_path
        }
    }

}//namespace ns_path_generator
