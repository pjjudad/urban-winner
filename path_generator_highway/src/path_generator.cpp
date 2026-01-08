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
#include "path_generator.hpp"   
#include "Utils/visual.h"
#include <sstream>
#include "global_vars.h"  

namespace ns_path_generator {
// Constructor  
    PathGenerator::PathGenerator(ros::NodeHandle &nh) : nh_(nh) {
        mission_ = nh_.param<std::string>("mission", "skidpad");
        param_.getParams(nh_, mission_);
        if (mission_ == "skidpad") { track_ = &skidpad_track_; }
        else {
            ROS_ERROR("Undefined Mission name !");
        }
    };

    PathGenerator::~PathGenerator() {
        // 移除曲率文件相关代码
    }

// Getters
    visualization_msgs::MarkerArray PathGenerator::getRefPath() { return RefPath_; }

// Setters
   
    void PathGenerator::setCarState(const fsd_common_msgs::CarState &state) {
         car_state_ = state;
    }

    void PathGenerator::setMidPoint(const fsd_common_msgs::Trajectory &midPoint) {
         midPoint_ = midPoint;
         ROS_INFO_STREAM("Received trajectory with " << midPoint.trajectory.size() << " points");
    }

    
    void PathGenerator::setLocalMap(const fsd_common_msgs::Map &map) {
        local_map_ = map;   
        ROS_INFO_STREAM("set Local Map OK");    
    }

    //acceleration 
    void PathGenerator::setEndPoint(const geometry_msgs::Point &point) {
        endPoint_ = point;  
        ROS_INFO("setEndPoint OK: %.3f, %.3f", point.x, point.y);
    }

    //skidpad 
    void PathGenerator::setTransMat(const std_msgs::Float64MultiArray &array) {
        if(array.data.size() < 16) {
            ROS_ERROR("Invalid transform matrix size!");
            return;
        }
        
        int element_counter = 0;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                transMat_(i, j) = array.data[element_counter];
                element_counter++;
            }
        }
    }

// Methods   
    void PathGenerator::runAlgorithm() {
        if (!Check()) {
            ROS_ERROR("Message Check ERROR!");   
            return;
        }
        
        if(!track_) {
            ROS_ERROR("Track not initialized!");
            return; 
        }
        
        try {
            setTrack();
            track_->setState(VehicleState(car_state_, cmd_));
            track_->CalculateTraj(refline_);
            
            // 移除保存曲率到文件的代码
        } catch(const std::exception& e) {
            ROS_ERROR_STREAM("Exception in runAlgorithm: " << e.what());
            return;
        }
    }

       
    bool PathGenerator::Check() {
        if (mission_ == "skidpad") {   
            if (midPoint_.trajectory.empty()) {            
                ROS_WARN_STREAM("midPoint trajectory is empty!");  
                return false;
            }
        }
        ROS_DEBUG_STREAM("Successfully passing check");    
        return true;
    }

    
    void PathGenerator::setTrack() {  
        // Just need to set once
        if (!is_init) {
            if (mission_ == "acceleration") {   
                track_->setEndPoint(endPoint_);
            }
            if (mission_ == "skidpad") {       
                track_->SetMidpoint(midPoint_);
            }
            track_->genTraj();  
           
        }
        //is_init = true;  

        // Need to set every time   
        if (mission_ == "trackdrive") {
            
            track_->setMap(local_map_);  
            track_->genTraj();  
             //
             //
        }
    }

    
    
    
    
    fsd_common_msgs::Trajectory PathGenerator::getRefTrajectory() {     
        fsd_common_msgs::Trajectory refTraj_;  
        
        //检测是否为空
        if (refline_.empty())//数据为空时函数返回为true
        {
            ROS_DEBUG_STREAM("error!!");
        }
        refTraj_.trajectory.clear();
        #if 1
        //if(refline_.empty() != true)std::cout<<"have point!"<<std::endl;
        for (auto point : refline_) {    
            fsd_common_msgs::TrajectoryPoint ref_pt;         
            ref_pt.pts.x = point.pts.x;            
            ref_pt.pts.y = point.pts.y;
            ref_pt.acc.data = point.acc;
            ref_pt.curvature.data = point.curvature;
            ref_pt.r.data = point.r;
            ref_pt.yaw.data = point.yaw;
            ref_pt.velocity.data = point.velocity;
            //ROS_INFO_STREAM("point.pts.x = "<<point.pts.x);

            refTraj_.trajectory.push_back(ref_pt);   
        }
        #endif
        
        #if 0
        while(refline_.empty() != false){ 
            for (auto point : refline_) {    
                fsd_common_msgs::TrajectoryPoint ref_pt;         
            	ref_pt.pts.x = point.pts.x;            
            	ref_pt.pts.y = point.pts.y;
            	ref_pt.acc.data = point.acc;
            	ref_pt.curvature.data = point.curvature;
            	ref_pt.r.data = point.r;
            	ref_pt.yaw.data = point.yaw;
            	ref_pt.velocity.data = point.velocity;
//            ROS_INFO_STREAM("point vel = "<<point.velocity);

            	refTraj_.trajectory.push_back(ref_pt);   
            }
        }
         #endif   
        refTraj_.header = car_state_.header;  
        // refTraj_.header.frame_id ="/base_link";   
        
        return refTraj_;   
    }
}