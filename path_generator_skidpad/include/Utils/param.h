#pragma once

#include "ros/ros.h"
struct Param {

    int N;
    double dt;
    bool simulation;

    double car_length;

    double initial_velocity;

    // Trajectory Interval
    double interval;

    double forward_distance;

    double circle_radius;

    double test;

    double max_lat_acc;  //最大横向加速度

    // For Pure Pursuit
    double desire_vel;

    // Get Parameters from yaml  如果没设置就使用下面设定的默认值
    void getParams(ros::NodeHandle &nh, const std::string &mission) {
        car_length = nh.param("car_length", 1.88);
        N = nh.param("N", 40);          //这个是啥？
        dt = nh.param("dt", 0.04);    //单位时间   1/0.04 = 25
        simulation = nh.param("simulation", false);
        interval = nh.param("interval", 0.08);       //间隔，这个是啥？
        forward_distance = nh.param("forward_distance", 15.0);
        circle_radius = nh.param("circle_radius", 9.125);
        max_lat_acc = nh.param("max_lat_acc", 3.0);
        initial_velocity = nh.param("initial_velocity", 2.0);   
        desire_vel = nh.param("desire_vel", 3);
        test = nh.param("test", 12); //为什么要把左圆右移呢？
    }
};

extern Param param_;
