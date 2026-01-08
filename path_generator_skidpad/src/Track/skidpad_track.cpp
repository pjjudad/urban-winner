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

#include "Track/skidpad_track.h"
#include "ros/ros.h"
#include "global_vars.h"

Param param_;

namespace ns_path_generator
{

  bool Skidpad_Track::genTraj()
  {

    // The front side to the gravity point    重力点的前侧  ??
    const double car_length = param_.simulation ? 0 : param_.car_length;

    double interval = param_.interval; // 可能是创建环形虚拟路径时路径点的间隔
    double forward_distance = param_.forward_distance;
    double circle_radius = param_.circle_radius;
    double right_circle_x = forward_distance; // 这是在表示左右圆的圆心位置吧
    double right_circle_y = -circle_radius;
    double left_circle_x = right_circle_x - 0;
    // double left_circle_y = param_.test;
    double left_circle_y = circle_radius;

    TrajectoryPoint tmp_pt;
    TrajectoryPoint test;
    trajectory_.clear();

    // line need discrete  线路需要离散  //discrete 离散
    for (double i = 0; i < (forward_distance); i += interval)
    {
      tmp_pt.pts.x = i;
      tmp_pt.pts.y = 0;
      tmp_pt.yaw = 0;
      trajectory_.push_back(tmp_pt);
    }

    // right_circle
    for (double i = 0; i < 4 * M_PI + interval / circle_radius * 40; i += interval / circle_radius)
    { // / circle_radius
      tmp_pt.pts.x = circle_radius * std::cos(90 * M_PI / 180 - i) + right_circle_x;
      tmp_pt.pts.y = circle_radius * std::sin(90 * M_PI / 180 - i) + right_circle_y;
      tmp_pt.yaw = -i;
      trajectory_.push_back(tmp_pt);
    }
/*-----------------------------------------------------*/
    double j = interval / circle_radius * 40;
    // test.pts.x = circle_radius * std::cos(-90 * M_PI / 180 + j) + left_circle_x;
    // test.pts.y = circle_radius * std::sin(-90 * M_PI / 180 + j) + left_circle_y;
    // test.yaw = -j;

    // // 获取最后两个点
    // auto last_point = trajectory_.back();
    // auto second_last_point = *(trajectory_.rbegin() + 1); // 或者使用 *(trajectory_.end() - 2)

    //     // 计算两点间距离
    // double dis = sqrt(pow(last_point.pts.x - second_last_point.pts.x, 2) + pow(last_point.pts.y - second_last_point.pts.y, 2));
    // double all_dis = sqrt(pow(last_point.pts.x - test.pts.x, 2) + pow(last_point.pts.y - test.pts.y, 2));

    // int num_points = static_cast<int>(all_dis / dis);
    // for (int k = 1; k <= num_points; k++)
    // {
    //   double ratio = static_cast<double>(k) / (num_points + 1);
    //   tmp_pt.pts.x = last_point.pts.x + ratio * (test.pts.x - last_point.pts.x);
    //   tmp_pt.pts.y = last_point.pts.y + ratio * (test.pts.y - last_point.pts.y);

    //   // 计算方向角（可以使用线性插值或直接计算两点间的角度）
    //   double dx = test.pts.x - last_point.pts.x;
    //   double dy = test.pts.y - last_point.pts.y;
    //   tmp_pt.yaw = atan2(dy, dx); // 或者使用线性插值: last_point.yaw + ratio * (test.yaw - last_point.yaw)

    //   trajectory_.push_back(tmp_pt);
    // }

    // // 最后添加test点
    // trajectory_.push_back(test);
/*-------------------------------------------------------------*/
    // left_circle
    for (double i = j; i < 4 * M_PI; i += interval / circle_radius)
    {
      tmp_pt.pts.x = circle_radius * std::cos(-90 * M_PI / 180 + i) + left_circle_x;
      tmp_pt.pts.y = circle_radius * std::sin(-90 * M_PI / 180 + i) + left_circle_y; // param_.test
      tmp_pt.yaw = i;
      trajectory_.push_back(tmp_pt);
    }
/*    --------------------------------------------------------- */
    // // line again  为什么要再次离散？
    for (float i = forward_distance ; i < (forward_distance) + 20; i += interval)
    { //+18
      tmp_pt.pts.x = i;
      tmp_pt.pts.y = 0;
      tmp_pt.yaw = 0;
      trajectory_.push_back(tmp_pt);
    }

    /*假设起始点为 (forward_distance, 0)，向左旋转n度（弧度）
    int n = 60;
float start_x = forward_distance;
float start_y = 0;
float angle_rad = n * M_PI / 180.0f; // 如果n是角度，转换为弧度

for (float i = forward_distance; i < (forward_distance) + 20; i += interval)
{
    tmp_pt.pts.x = start_x + (i - start_x) * cos(angle_rad) - (0 - start_y) * sin(angle_rad);
    tmp_pt.pts.y = start_y + (i - start_x) * sin(angle_rad) + (0 - start_y) * cos(angle_rad);
    tmp_pt.yaw = angle_rad; // 设置朝向为旋转角度
    trajectory_.push_back(tmp_pt);
}*/ 


/* --------------------------------------------------------------------*/
    // Transform the trajectory

    for (size_t i = 0; i < trajectory_.size(); i++)
    {
      double temp_x, temp_y;
      temp_x = trajectory_[i].pts.x;
      temp_y = trajectory_[i].pts.y;
      Eigen::Vector4f temp(temp_x, temp_y, 0, 1);
      Eigen::Vector4f result = transMat_ * temp; // 用到了ns_path_generator::Track::transMat_
      trajectory_[i].pts.y = result[1] / result[3];
      trajectory_[i].pts.x = result[0] / result[3];

      point_cloud.header.stamp = ros::Time::now();
      point_cloud.header.frame_id = "world"; // 设置你需要的坐标系

      // 添加点
      geometry_msgs::Point32 point;
      point.x = trajectory_[i].pts.x;
      point.y = trajectory_[i].pts.y;
      point.z = 0.0; // 假设 z 坐标为 0
      point_cloud.points.push_back(point);
    }

    return true;
  }

  bool Skidpad_Track::CalculateTraj(Trajectory &refline)
  {

    if (trajectory_.empty())
    {
      ROS_WARN("Trajectory is empty !");
      return false;
    }
    refline.clear();
    TrajectoryPoint tmp_pt;

    double px = state_.x;
    double py = state_.y;
    double psi = state_.yaw;

    int count = 0;
    int index_min = -1;
    double min = 999;
    for (int i = now_state; i < trajectory_.size() && i < now_state + 400; i++)
    {
      double delta_x = trajectory_[i].pts.x - px;
      double delta_y = trajectory_[i].pts.y - py;
      double dist = std::hypot(delta_x, delta_y);
      if (dist < min)
      {
        min = dist;
        index_min = i;
      }
    }
    if (index_min < 0)
      return false;

    now_state = index_min;

    const double desired_velocity = param_.desire_vel;
    const int N = param_.N;
    const double dt = param_.dt;
    TrajectoryPoint tmp;
    //state_.v = 0;
    std::cout << "stateV : " << state_.v << "   paramv : " << param_.initial_velocity << std::endl; 
    double v = std::fmax(state_.v, param_.initial_velocity);

    double s_tmp = 0;
    for (int i = 0; i < N; i++)
    {
      if (i != 0)
        s_tmp += v * dt;

      int index = int(s_tmp / param_.interval);
      int next = (index + index_min) > trajectory_.size() ? trajectory_.size() - 1 : index + index_min;

      double delta_x = trajectory_[next].pts.x - px;
      double delta_y = trajectory_[next].pts.y - py;
      double delta_yaw = trajectory_[next].yaw - psi;

      while ((delta_yaw) >= M_PI)
        delta_yaw -= M_PI * 2.0;
      while ((delta_yaw) <= -1.0 * M_PI)
        delta_yaw += M_PI * 2.0;

      double temp_x, temp_y;
      temp_x = delta_x * cos(psi) + delta_y * sin(psi);
      temp_y = delta_y * cos(psi) - delta_x * sin(psi);

      tmp.curvature = fabs(trajectory_[next].curvature);
      tmp.pts = cv::Point2f(temp_x, temp_y);
      tmp.yaw = delta_yaw;
      tmp.velocity = std::min(sqrt(param_.max_lat_acc / tmp.curvature), desired_velocity);
      refline.push_back(tmp);
    }
    return true;
  }

} // namespace ns_path_generator
