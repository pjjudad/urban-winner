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
    along with FSD-Project. If not, see <https://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include "path_generator_handle.hpp"

namespace ns_path_generator {

// Constructor
    PathGeneratorHandle::PathGeneratorHandle(ros::NodeHandle &nodeHandle) :
            nodeHandle_(nodeHandle), path_generator_(nodeHandle) {
        ROS_INFO("Constructing Handle");
        loadParameters();
        subscribeToTopics();
        publishToTopics();
    }

// Getters
    int PathGeneratorHandle::getNodeRate() const { return node_rate_; }

// Methods  
    void PathGeneratorHandle::loadParameters() {
        try {
            // 检查必要参数
            if(!nodeHandle_.hasParam("mission")) {
                ROS_ERROR("Missing required parameter: mission");
                throw std::runtime_error("Missing required parameter");
            }
            ROS_INFO("loading handle parameters");     //不同的mission下面这些参数只用到各自对应要用到的参数，不是每次都需要全部参数
            if (!nodeHandle_.param<std::string>("car_state_topic_name",          //这个应该是都要的
                                                car_state_topic_name_,
                                                "/estimation/slam/state")) {
                ROS_WARN_STREAM("Did not load car_state_topic_name. Standard value is: " << car_state_topic_name_);
            }
            if (!nodeHandle_.param<std::string>("transform_matrix_topic_name",   //for skidpad
                                                transform_matrix_topic_name_, "/transform_matrix")) {
                ROS_WARN_STREAM(
                        "Did not load transform_matrix_topic_name. Standard value is: "
                                << transform_matrix_topic_name_);
            }
            if (!nodeHandle_.param<std::string>("end_point_topic_name",        //for acceleration
                                                end_point_topic_name_, "/planning/end_point")) {
                ROS_WARN_STREAM(
                        "Did not load end_point_topic_name. Standard value is: "
                                << end_point_topic_name_);
            }
            if (!nodeHandle_.param("node_rate", node_rate_, 10)) {           
                ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
            }
            if (!nodeHandle_.param<std::string>("map_topic_name",           
                                                map_topic_name_,         
                                                "/map")) {               //默认值
                ROS_WARN_STREAM("Did not load map_topic_name. Standard value is : " << map_topic_name_);
            }
            if (!nodeHandle_.param<std::string>("ref_path_topic_name",     
                                                ref_path_topic_name_,
                                                "/visual/ref_path")) {
                ROS_WARN_STREAM("Did not load ref_path_topic_name_. Standard value is: " << ref_path_topic_name_);
            }
        if (!nodeHandle_.param<std::string>("path_generate_topic_name",      
                                            path_generate_topic_name_,
                                            "/planning/ref_path")) {
            ROS_WARN_STREAM("Did not load path_generate_topic_name. Standard value is: "<< path_generate_topic_name_);
        }
        } catch(const std::exception& e) {
            ROS_ERROR_STREAM("Exception in loadParameters: " << e.what());
            throw;
        }
    }

    //
    void PathGeneratorHandle::subscribeToTopics() {
        try {
            // 对每个订阅器添加检查
            transMatSubscriber_ = nodeHandle_.subscribe(
                transform_matrix_topic_name_, 1, &PathGeneratorHandle::transMatCallback, this);
            if(!transMatSubscriber_) {
                ROS_ERROR("Failed to subscribe to transform matrix topic");
                return;
            }
            
            localMapSubscriber_ = nodeHandle_.subscribe(
                map_topic_name_, 10, &PathGeneratorHandle::localMapCallback, this);
            if(!localMapSubscriber_) {
                ROS_ERROR("Failed to subscribe to local map topic");
                return;
            }
            //from line_detector
            endPointSubscriber_ = nodeHandle_.subscribe(
                end_point_topic_name_, 1, 
                &PathGeneratorHandle::endPointCallback, this
            );
            if(!endPointSubscriber_) {
                ROS_ERROR("Failed to subscribe to endpoint topic");
                return;
            }
        } catch(const ros::Exception& e) {
            ROS_ERROR_STREAM("Exception in subscribeToTopics: " << e.what());
            return;
        }
        //others   
        carStateSubscriber_ =    
                nodeHandle_.subscribe(car_state_topic_name_, 10, &PathGeneratorHandle::carStateCallback, this); 

        mid_point_ = nodeHandle_.subscribe("/spline_cloud", 10, &PathGeneratorHandle::midPointCallback, this); ///mid_point

    }

    void PathGeneratorHandle::publishToTopics() {
        try {
            refPathVisualPublisher_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>(
                ref_path_topic_name_, 10);
            if(!refPathVisualPublisher_) {
                ROS_ERROR("Failed to create ref path visual publisher");
                return;
            }
            
            refPathPublisher_ = nodeHandle_.advertise<fsd_common_msgs::Trajectory>(
                path_generate_topic_name_, 10);
            if(!refPathPublisher_) {
                ROS_ERROR("Failed to create ref path publisher");
                return;
            }
        } catch(const ros::Exception& e) {
            ROS_ERROR_STREAM("Exception in publishToTopics: " << e.what());
            return;
        }
    }

    void PathGeneratorHandle::run() {   
        try {
            if(!&path_generator_) {
                ROS_ERROR("path_generator_ not initialized in run()!");
                return;
            }
            path_generator_.runAlgorithm();
            //std::cout << "complete algorithm!!" << std::endl;
            sendMsg();
        } catch(const std::exception& e) {
            ROS_ERROR_STREAM("Exception in run(): " << e.what());
            return;
        }
    }

    void PathGeneratorHandle::sendMsg() {
        try {
            auto refPath = path_generator_.getRefPath();
            if(!refPath.markers.empty()) {
                refPathVisualPublisher_.publish(refPath);
            }
            
            auto refTraj = path_generator_.getRefTrajectory();
            if(!refTraj.trajectory.empty()) {
                refPathPublisher_.publish(refTraj);
            }
        } catch(const std::exception& e) {
            ROS_ERROR_STREAM("Error publishing messages: " << e.what());
        }
    }

    void PathGeneratorHandle::endPointCallback(const geometry_msgs::Point &msg) {
        path_generator_.setEndPoint(msg);   //for acceleration
    } 

    //会对local_map_赋值  
    void PathGeneratorHandle::localMapCallback(const fsd_common_msgs::Map &msg) {
        if(!&path_generator_) {
            ROS_ERROR("path_generator_ not initialized in localMapCallback!");
            return;
        }
        path_generator_.setLocalMap(msg);
    }

    void PathGeneratorHandle::carStateCallback(const fsd_common_msgs::CarState &msg) {
        if(!&path_generator_) {
            ROS_ERROR("path_generator_ not initialized!");
            return;
        }
        path_generator_.setCarState(msg);
    }

    void PathGeneratorHandle::transMatCallback(const std_msgs::Float64MultiArray &msg) {
        path_generator_.setTransMat(msg);
    }

    void PathGeneratorHandle::midPointCallback(const fsd_common_msgs::Trajectory &msg ){
        path_generator_.setMidPoint(msg);
    }
}