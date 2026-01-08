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

#include "Track/trackdrive_track.h"
#include "ros/ros.h"

namespace ns_path_generator {

//都是在全局坐标系下计算的

//生成轨道，要用到fsd::Spline2D
///只有trackdrive_trac才要继续用到锥桶map,应为后续需要基于delaunnay三角剖分找路径中点
//在哪调用？在path_generator.cpp里面调用
//这里的map是在哪调用的呀？,
 
//这个好像已经默认识别好红蓝锥桶后,直接去中点生成的轨迹。 实车不能这样搞吧。 
bool Autox_Track::genTraj() {
  if (map_.cone_blue.empty() || map_.cone_red.empty())
    return false;

  fsd::Vec_f wx, wy;
  wx.push_back(0);
  wy.push_back(0);
  // Find Center Line     
  //这一部分代码通过”Delaunay三角剖分“来离散搜索空间，实现图形搜树以获得所有可能的中心线。给后续计算成本使用
  //没太看懂下面一大段代码的原理
  {   //为什么括号可以这样用？ 这种主要是为了增加可读性，限制变量范围
    for (const auto &red : map_.cone_red) {

      const auto it_blue = std::min_element(
          map_.cone_blue.begin(), map_.cone_blue.end(),      //ccon_blue是vector容器
          [&](const fsd_common_msgs::Cone &a, const fsd_common_msgs::Cone &b) {   //传进来的是两个桩桶对象
            const double da = std::hypot(red.position.x - a.position.x,
                                         red.position.y - a.position.y);
            const double db = std::hypot(red.position.x - b.position.x,
                                         red.position.y - b.position.y);

            return da < db;
          });
      cv::Point2f tmp;
      tmp.x = static_cast<float>((red.position.x + it_blue->position.x) / 2.0);  //直接直接除于2，找出两个不同颜色桩桶之间的中点
      tmp.y = static_cast<float>((red.position.y + it_blue->position.y) / 2.0);
      wx.push_back(tmp.x);
      wy.push_back(tmp.y);
    }
  }

  fsd::Spline2D spline(wx, wy);  //传入的参数是vector容器 之后spline会基于锥桶直接算出理想的yaw、curvature等参数，而r 一直是0

  TrajectoryPoint tmp_pt;
  trajectory_.clear();
  const double interval = param_.interval;    //interval 间隔、穿插出现的间隙

  for (float i = 0; i < spline.s.back(); i += interval) {   //vector的back()是返回最后一个元素
    std::array<float, 2> point_ = spline.calc_postion(i);
    tmp_pt.pts.x = point_[0];
    tmp_pt.pts.y = point_[1];
    tmp_pt.yaw = spline.calc_yaw(i);  //这个如何算出很关键
    tmp_pt.r = 0;     //一直等于0?
    tmp_pt.curvature = spline.calc_curvature(i);  //curevature 曲率
    trajectory_.push_back(tmp_pt);    //填进去了
  }
  double dis_min = 1; // judge only in 1 meters  太短的就抛弃吗？
  bool flag_1 = false;
  bool flag_2 = false;
  double index = 0;
  for (float i = trajectory_.size() - 1; i>=0; i-= interval) {
    //hypot()函数是用来计算三角形斜边长的。 理解一下下面的计算过程
    double dis = std::hypot(trajectory_[trajectory_.size()-1].pts.x - trajectory_[i].pts.x, 
                            trajectory_[trajectory_.size()-1].pts.y - trajectory_[i].pts.y);
    if (flag_1 == false && dis <= dis_min)
    continue;
    if (flag_1 == false && dis > dis_min)
    flag_1 = true;
    if (flag_1 == true && dis <= dis_min)
    flag_2 = true;                         
    if (flag_2 == true && dis <= dis_min)
    dis_min = dis;                             
    if (flag_2 == true && dis > dis_min) {
      index = i;
      break;
    }
  }
  //干嘛要将trajectoty里面数据拿出来有重新放回去？是为了缩小其所占内存空间？
  Trajectory tmp;  //存放临时的预测轨迹数据
  for (float i=index; i<trajectory_.size(); i++) {
    tmp.push_back(trajectory_[i]);
  }
  trajectory_.clear();  
  trajectory_ = tmp;       //返回生成的这一条路径
}


//原本是基于地图坐标系生成的轨迹吗？  现在ref_path的起点一直在原点
//计算轨道  计算啥？根据初步生成的道路中线计算出TrajectoryPoint的其他参数   // 用的是世界坐标系？

//根据当前车辆位姿重新生成轨迹
bool Autox_Track::CalculateTraj(Trajectory &refline) {

  if (trajectory_.empty()) {
    ROS_WARN("Trajectory is empty !");
    return false;
  }
  refline.clear();

  double px = state_.x;    //世界坐标系
  double py = state_.y; 
  double psi = state_.yaw;  //车辆相对于位姿的偏航角

  //找出最小的两个轨迹点距离 并 在相应位置加上索引。
  int count = 0;
  int index_min = -1;
  double min = 999;
  for (int i = 0; i < trajectory_.size(); i++) {
    double delta_x = trajectory_[i].pts.x - px;
    double delta_y = trajectory_[i].pts.y - py;
    double dist = std::hypot(delta_x, delta_y);   //hypot()函数是用来计算三角形斜边长的。 如果hypot()方法的参数有一个负数,方法会直接返回NaN
    if (dist < min) {
      min = dist;
      index_min = i;  //index 索引、指标
    }
  }

  if (index_min < 0) //如果索引值为负，就返回false
    return false;

  const double desired_velocity = param_.desire_vel;
  const int N = param_.N;       //两轨迹点之间的插值个数？  40
  const double dt = param_.dt;  //时间间隔              0.04
  TrajectoryPoint tmp; 
  double v = std::fmax(state_.v, param_.initial_velocity);  //在 初始速度 和 当前速度中 取较大的值 作为当前轨迹点的期望速度


  //距离，dt*state.v
  double s_tmp = 0;
  for (int i = 0; i < N; i++) {   //N设为了40
    if (i != 0)
      s_tmp += v * dt;    //dt设为了 0.04

    int index = int(s_tmp / param_.interval);  //索引      param_.interval设为了0.05

    //计算整条路径的 x，y，yaw 的 变化最大值
    double delta_x = trajectory_[(index + index_min) % trajectory_.size()].pts.x - px;
    double delta_y = trajectory_[(index + index_min) % trajectory_.size()].pts.y - py;
    double delta_yaw = trajectory_[(index + index_min) % trajectory_.size()].yaw - psi;  //yaw 偏航角
    
    //转向角偏差限制在正负pi/2之间
    while ((delta_yaw) >= M_PI)   //这个是干嘛的？
      delta_yaw -= M_PI * 2.0;
    while ((delta_yaw) <= -1.0 * M_PI)
      delta_yaw += M_PI * 2.0;

    //根据 当前车辆偏航角 和 计算的x，y偏差，重新生成点坐标
    double temp_x, temp_y;
    temp_x = delta_x * cos(psi) + delta_y * sin(psi);
    temp_y = delta_y * cos(psi) - delta_x * sin(psi);

    tmp.curvature = fabs(trajectory_[(index + index_min) % trajectory_.size()].curvature);  //求浮点数的绝对值
    tmp.pts = cv::Point2f(temp_x, temp_y);
    tmp.yaw = delta_yaw;  //到下一点所需偏航角

    //param_.max_lat_acc是最大横向加速度  sqrt() 返回输入值的非负数平方根
     //在 最大横向加速度所限制的速度 和 先验期望速度 之间选个比较小的速度作为轨迹期望速度

    //切向加速度是速度对时间的一阶导数；
    //法向加速度表示速度方向变化的快慢
    //曲率半径是速度的平方除以法向加速度  a = V^2 / R   a 是法向加速度，V是曲线上的切线方向速度，R是曲率半径

    tmp.velocity = std::min(sqrt(param_.max_lat_acc / tmp.curvature), desired_velocity);  

    //最后都是放到refline里，实际控制使用的轨迹，基于这个生成的/planning/ref_path
    refline.push_back(tmp);       
  }
}

}//namespace ns_path_generator
