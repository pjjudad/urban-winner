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
        }else {
        ROS_INFO_STREAM("Loaded end_point_topic_name: " << end_point_topic_name_);
    }
        if (!nodeHandle_.param("node_rate", node_rate_, 10)) {           //这个应该是都要的
            ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
        }
        if (!nodeHandle_.param<std::string>("map_topic_name",           //for trackdrive
                                            map_topic_name_,         //存放话题名的变量，可以在yaml文件里面通过第一个参数“map_topic_name”进行修改
                                            "/map")) {               //默认值
            ROS_WARN_STREAM("Did not load map_topic_name. Standard value is : " << map_topic_name_);
        }
        if (!nodeHandle_.param<std::string>("ref_path_topic_name",     //这个不是关键，只是为了可视化
                                            ref_path_topic_name_,
                                            "/visual/ref_path")) {
            ROS_WARN_STREAM("Did not load ref_path_topic_name_. Standard value is: " << ref_path_topic_name_);
        }
    if (!nodeHandle_.param<std::string>("path_generate_topic_name",       //这个应该是都要的吧
                                        path_generate_topic_name_,
                                        "/planning/ref_path")) {
        ROS_WARN_STREAM("Did not load path_generate_topic_name. Standard value is: "<< path_generate_topic_name_);
    }
    }

    //只要用了这个函数，就会尝试订阅 /estimation/slam/state 、  /map 、/transform_matrix、/planning/end_point 四个话题 后三个是对应method才会用到的
    void PathGeneratorHandle::subscribeToTopics() {
        ROS_INFO("subscribe to topics");
        //from line_detector
        endPointSubscriber_ = nodeHandle_.subscribe(
                end_point_topic_name_, 1, &PathGeneratorHandle::endPointCallback, this);   //等待endPoint话题
        //from skidpad_detector
        transMatSubscriber_ = nodeHandle_.subscribe(
                transform_matrix_topic_name_, 1, &PathGeneratorHandle::transMatCallback, this);
        //for trackdrive  
        //实际是 /map  一路到 local_map_ 都是这个话题的 
        //只要成功订阅到 /map话题，都会在相应的回调函数调用setLocalMap(),对local_map_赋值，并打印set Local Map OK,故而其他两种工况还是可能没实际用到 /map
        
        //改成订阅/planning/boundary_detections
        localMapSubscriber_ =    //fssim_interface发布的   trackdrive_track.yaml里面设为了/map ,,  最好改成/planning/boundary_detectionsy
                nodeHandle_.subscribe(map_topic_name_, 10, &PathGeneratorHandle::localMapCallback, this);  //实际是 /map
        
        //others   都要用到的
        carStateSubscriber_ =    //fssim_interface发布的   trackdrive_track.yaml里面设为了/estimation/slam/state
                nodeHandle_.subscribe(car_state_topic_name_, 10, &PathGeneratorHandle::carStateCallback, this); //实际是 /estimation/slam/state
    }

    void PathGeneratorHandle::publishToTopics() {
        ROS_INFO("publish to topics");
        refPathVisualPublisher_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>(ref_path_topic_name_, 10);   //用来在rviz那查看轨迹的
        refPathPublisher_ = nodeHandle_.advertise<fsd_common_msgs::Trajectory>(path_generate_topic_name_, 10);      //实际控制要按照这个话题的轨迹走的
        
    }

    void PathGeneratorHandle::run() {   //跑相应路径规划算法并将规划的路径发布出去
        path_generator_.runAlgorithm();
        sendMsg();
    }

    void PathGeneratorHandle::sendMsg() {      //看一下这些数据如组织 
        refPathVisualPublisher_.publish(path_generator_.getRefPath());      //可视化的
        refPathPublisher_.publish(path_generator_.getRefTrajectory());   //实际控制使用的参考路径发布者，这个参考系是世界坐标系还是车辆坐标系？
   
    }

    void PathGeneratorHandle::endPointCallback(const geometry_msgs::Point &msg) {
        path_generator_.setEndPoint(msg);   //for acceleration
    } 

    //会对local_map_赋值  
    void PathGeneratorHandle::localMapCallback(const fsd_common_msgs::Map &msg) {
        path_generator_.setLocalMap(msg);    //for trackdriver
    }

    void PathGeneratorHandle::carStateCallback(const fsd_common_msgs::CarState &msg) {
        path_generator_.setCarState(msg);    //都要用到的
    }

    void PathGeneratorHandle::transMatCallback(const std_msgs::Float64MultiArray &msg) {
        path_generator_.setTransMat(msg);    //for skidpad
    }
}