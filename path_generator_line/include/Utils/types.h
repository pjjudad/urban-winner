#ifndef TYPE_H
#define TYPE_H

#include <iostream>
#include "opencv2/opencv.hpp"

// ros package
#include "std_msgs/Float64MultiArray.h"

// custom messages
#include "fsd_common_msgs/CarState.h"
#include "fsd_common_msgs/ControlCommand.h"
#include "fsd_common_msgs/Map.h"

// STL
#include <cmath>
#include <vector>

namespace ns_path_generator { //VehicleState和TrajectoryPoint是三种工况都会用到吗？
    
    /*根据`src/ros/planning/path_generator/src/Track/trackdrive_track.cpp`里的`
       Autox_Track::CalculateTraj(Trajectory &refline)` 函数的代码：
     `double px = state_.x;  double py = state_.y;  double psi = state_.yaw;`，
     下面用的应该是用同一个参考系*/ //visual_trajectory

    //VehicleState和TrajectoryPoint都是在局部地图中得到的
    struct VehicleState {   //下面几个变量真的都用到了吗？
        double x; //x坐标
        double y; //y坐标
        double yaw; //方位⻆
        double v; //速度大小
        double r; //方位⻆速度
        double a; //加速度大小
        double w; //方位⻆加速度大小
        double Delta; //转向⻆控制值
        double D; //油⻔控制值

        //state进去要计算v(它由dt.x和dt.y合成)和 a(它由a.x和a.y合成,MPC控制才用)，其他都只是简单赋值.
        //pure_pursuit 只用到 yaw 和 v
        VehicleState(fsd_common_msgs::CarState state, fsd_common_msgs::ControlCommand cmd) {
            x = state.car_state.x;
            y = state.car_state.y;
            yaw = state.car_state.theta;
            v = std::hypot(state.car_state_dt.car_state_dt.x, state.car_state_dt.car_state_dt.y);  //速度v用x，y方向速度组合而成
            r = state.car_state_dt.car_state_dt.theta;   //用到了.car_state_dt.theta
            
            a = std::hypot(state.car_state_dt.car_state_a.x, state.car_state_dt.car_state_a.y); //加速度a用x，y方向的加速度组合而成
            w = state.car_state_dt.car_state_a.theta;   //这个应该实际可以不用吧

            D = cmd.throttle.data;             //一定要用加速度吗？  可以只用速度吗？
            Delta = cmd.steering_angle.data;
        }

        VehicleState() {

        }
    };

    //这里的yaw是通过锥桶计算得来的
    struct TrajectoryPoint {
        cv::Point2f pts;
        double yaw;         
        double curvature;   //斜率
        double velocity;    //速度
        double r;          //航向角角速度
        double acc;        //航向角角加速度   这个实际用到了吗？
    };

    typedef std::vector<TrajectoryPoint> Trajectory;

}

#endif
