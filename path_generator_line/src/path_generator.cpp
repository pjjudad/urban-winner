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

#include <ros/ros.h>
#include "path_generator.hpp"   //里面会把三种工况的所有头文件的包含进去
#include "Utils/visual.h"
#include <sstream>

namespace ns_path_generator {
// Constructor  这里只是获取世界坐标系下的锥桶，里面包含它们的坐标信息
    PathGenerator::PathGenerator(ros::NodeHandle &nh) : nh_(nh),line_track_(nh){
        mission_ = nh_.param<std::string>("mission", "acceleration");
        param_.getParams(nh_, mission_);
        if (mission_ == "trackdrive") { track_ = &trackdrive_track_; }
        else if (mission_ == "acceleration") { 
            // 传递节点句柄给Line_Track
        //line_track_.setNodeHandle(nh_);
        // 将原来的line_track_初始化改为通过nh_初始化
     //line_track_ = Line_Track(nh_);
            track_ = &line_track_; }
        else if (mission_ == "skidpad") { track_ = &skidpad_track_; }
        else {
            ROS_ERROR("Undefined Mission name !");
        }
    };

// Getters
    visualization_msgs::MarkerArray PathGenerator::getRefPath() { return RefPath_; }

// Setters
   //都会用到
    void PathGenerator::setCarState(const fsd_common_msgs::CarState &state) {
         car_state_ = state;
    }

    //trackdriver 的   会对成员变量local_map_赋值
    //如果是订阅的boundary_detector生成的基于车辆坐标系的边界的话，就需要将数据转化到全局坐标系下
    void PathGenerator::setLocalMap(const fsd_common_msgs::Map &map) {
        local_map_ = map;   //如果订阅的是fssim的基于全局坐标系的/map, 用这句直接赋值即可
        
        //如果是订阅的boundary_detector生成的基于车辆坐标系的边界的话，就需要将数据转化到全局坐标系下
        // local_map_.cone_unknow. = 

        ROS_INFO_STREAM("set Local Map OK");    //三种工况都有这个提示，但好像另外两个工况实际并没有用到local_map_
    }

    //acceleration 的
    void PathGenerator::setEndPoint(const geometry_msgs::Point &point) {
        endPoint_ = point;   //接着会被 PathGenerator::runAlgorithm()的Check()
        ROS_INFO("setEndPoint OK: %.3f, %.3f", point.x, point.y);
    }

    //skidpad 的   这个没有用 ROS_INFO   icp_finish在哪里打印的？在~/planning/skidpad_detector/src/skidpad_detector.cpp
    void PathGenerator::setTransMat(const std_msgs::Float64MultiArray &array) {
        int element_counter = 0;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                transMat_(i, j) = array.data[element_counter];
                element_counter++;
            }
        }
    }

// Methods   基于这个来理解所有数据的传递
    void PathGenerator::runAlgorithm() {
        if (!Check()) {
            ROS_ERROR("Message Check ERROR!");  //刚打开三种工况方法的launch的时候都会提示这个，过一段时间车才会动 
            return;
        }
        ROS_INFO_STREAM("set Track Start.");
        setTrack();  //生成了trajectory_    这个也是生成轨迹的啊
        
        ROS_INFO_STREAM("set Track Done.");
        track_->setState(VehicleState(car_state_, cmd_));     //这个真的用到了cmd_吗？都没有给它赋值呀？单纯补位？
        // 根据当前车辆位姿计算新的可供控制模块使用的轨迹
        track_->CalculateTraj(refline_);              // 不同方法会调用各自定义的同名函数， //赋值curvature、pts、yaw、velocity

        std::vector<float> color = {1, 1, 0};
        //将refline_变成可视化的RefPath_
        // visual_trajectory(refline_, RefPath_, "/base_link", color,     //根据refline_生成可视化的轨迹
        //                   car_state_.header, true); 
        visual_trajectory(refline_, RefPath_, "world", color,     //根据refline_生成可视化的轨迹
                          car_state_.header, true);  //第五个参数是为了给时间戳

    

    }

    //检查各轨迹生成算法是否正常运行生成输出   
    bool PathGenerator::Check() {
        if (mission_ == "trackdrive") {   //需要加载锥桶map，实际就是话题  /map 里面的数据 ,现在这里改成/local_map
        //改成订阅改成订阅/planning/boundary_detections     cone_left和cone_right

            if (local_map_.cone_red.empty() || local_map_.cone_blue.empty()) {
                ROS_WARN_STREAM("Local Map Empty !");   //trackdrive需要不停的设置生成Map
                return false;
            }
        }
        if (mission_ == "acceleration") {  //只设置起点和终点    //如果不订阅/lidar/cones的话，这个都过不去
            //fabs是求浮点数的绝对值
            //原来是if (fabs(endPoint_.y) > 6 || endPoint_.x <= 40.0) 超出范围则表示重点不合理，不进行控制
            //可能终点设置不对  为啥end point一直是0  0  ？  endPoint如何设置的？
            if (fabs(endPoint_.y) > 30 || endPoint_.x <= 40) {      //endPoint必须在左右不偏移6，纵向不小于40
                ROS_WARN("Acceleration end point is error, current end point is (%.3f,%.3f).", endPoint_.x,
                         endPoint_.y);  //但实际endPoint一直都是 0 , 0   生成就有问题
                return false;
            }
        }
        if (mission_ == "skidpad") {   //矩阵的方式OK
            if (transMat_(3, 3) != 1) {
                ROS_WARN_STREAM("transMatrix is not correct !");
                return false;
            }
        }
        ROS_DEBUG_STREAM("Successfully passing check");    //一直到正确生成path才返回true，不再报Message Check ERROR
        return true;
    }

    //设置路径,不同方法的genTraj()定义是不同的,  只有 trackdrive才用到了local_map_呀
    void PathGenerator::setTrack() {  //看看这个在哪里调用了
        // Just need to set once
        if (!is_init) {
         //   if (mission_ == "acceleration") {   //一次性生成即可？
            //    track_->setEndPoint(endPoint_);
           // }
            if (mission_ == "skidpad") {       //一次性生成即可？
                track_->setTransMat(transMat_);
            }
            track_->genTraj();  
        }
        is_init = true;  //会在哪里重新弄设为false
        if (mission_ == "acceleration") {   //一次性生成即可？
               track_->setEndPoint(endPoint_);
                track_->genTraj(); 
            }
        // Need to set every time   //也就是说每次都是重新生成的
        if (mission_ == "trackdrive") {
            //必须要给map_赋值，否则无法跑起来
            track_->setMap(local_map_);  //只有这里采用到  /map 里面的数据，直接就是拿到的全部的轨迹。。。给map_赋值。。。
            track_->genTraj();  
             //实际调用的是 ~/fsd_algorithm_ws/src/ros/planning/path_generator/src/Track/trackdrive_track.cpp
             //上述路径的 bool Autox_Track::genTraj()
        }
    }
    
    //简单的复制，应该是给到其他地方去处理，三个路径生成的路径都可以通过这个函数获取
    // getRefTrajectory()是PathGenerator类里面的成员函数
    //fsd_common_msgs::Trajectory是返回的数据类型Using end point in Line_Track
    //这里的坐标点参考系是 局部地图
    fsd_common_msgs::Trajectory PathGenerator::getRefTrajectory() {     
        fsd_common_msgs::Trajectory refTraj_;  //生成预测路径msgs对象，

        refTraj_.trajectory.clear();
        for (auto point : refline_) {     //括号里是啥意思？  自动类型的轨迹点point继承refline
            fsd_common_msgs::TrajectoryPoint ref_pt;         
            ref_pt.pts.x = point.pts.x;            
            ref_pt.pts.y = point.pts.y;
            ref_pt.acc.data = point.acc;
            ref_pt.curvature.data = point.curvature;
            ref_pt.r.data = point.r;
            ref_pt.yaw.data = point.yaw;
            ref_pt.velocity.data = point.velocity;
//            ROS_INFO_STREAM("point vel = "<<point.velocity);

            refTraj_.trajectory.push_back(ref_pt);   //规划出来的路径赋值了
        }
        refTraj_.header = car_state_.header;  
        // refTraj_.header.frame_id ="/base_link";   有yaw，输入必须是基于全局坐标系，需要将输入的boundary转为全局坐标系才行
        return refTraj_;   //基于局部地图规划出来的轨迹
    }
}
