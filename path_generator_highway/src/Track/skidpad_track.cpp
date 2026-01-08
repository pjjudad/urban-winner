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
#include <algorithm>

Param param_;

/*
此代码订阅来自三次样条程序的spline_cloud话题

*/
namespace ns_path_generator
{

  // 此函数获取三次样条后的中点，数据类型是fsd_common_msgs::Trajectory,
  bool Skidpad_Track::genTraj()
  {

    // The front side to the gravity point    重力点的前侧  ??
    const double car_length = param_.simulation ? 0 : param_.car_length;
    double interval = param_.interval; // 可能是创建环形虚拟路径时路径点的间隔
    double forward_distance = param_.forward_distance;
    double circle_radius = param_.circle_radius;
    double right_circle_x = forward_distance + car_length; // 这是在表示左右圆的圆心位置吧
    double right_circle_y = -circle_radius;
    double left_circle_x = right_circle_x;
    double left_circle_y = circle_radius;

    TrajectoryPoint tmp_pt;
    trajectory_.clear();

    if (midPoint_.trajectory.empty()) {
        ROS_WARN("Midpoint trajectory is empty!");
        return false;
    }

    double previous_yaw = 0.0; // 初始 yaw

    for (size_t i = 0; i < midPoint_.trajectory.size(); ++i)
    {
      const auto &point = midPoint_.trajectory[i];
      tmp_pt.pts.x = point.pts.x;
      tmp_pt.pts.y = point.pts.y;
      tmp_pt.yaw = point.yaw.data;
      tmp_pt.velocity = point.velocity.data;
      tmp_pt.curvature = point.curvature.data;

      trajectory_.push_back(tmp_pt);
    }

    ROS_INFO_STREAM("Generated trajectory with " << trajectory_.size() << " points");
    return true;
  }
  bool Skidpad_Track::CalculateTraj(Trajectory &refline)
  {
    if (trajectory_.empty())
    {
      ROS_WARN("Trajectory is empty!");
      return false;
    }

    // 获取车辆状态和轨迹信息
    double px = state_.x;
    double py = state_.y;
    double psi = state_.yaw;
    int size = trajectory_.size();

    // 验证轨迹质量
    bool valid_trajectory = true;
    if (trajectory_.empty())
    {
      ROS_WARN("Trajectory is empty");
      valid_trajectory = false;
    }

    // 检查轨迹点是否有NaN值
    for (const auto &point : trajectory_)
    {
      if (std::isnan(point.pts.x) || std::isnan(point.pts.y))
      {
        ROS_WARN("Trajectory contains NaN values");
        valid_trajectory = false;
        break;
      }
    }

    if (!valid_trajectory)
    {
      ROS_WARN("Trajectory validation failed");
      return false;
    }

    // 使用改进的搜索算法
    int index_min = -1;
    double min_dist = std::numeric_limits<double>::max();
    int search_window = 100;

    // 根据速度动态调整搜索窗口大小
    if (state_.v > 1.0)
    {
      search_window = static_cast<int>(search_window * (1.0 + state_.v * 0.5));
    }
    search_window = std::min(search_window, size / 2);

    // 确定搜索起点
    int start_index = now_state;
    if (state_.v > 0.5)
    {
      double predicted_progress = state_.v * 0.2;
      int predicted_index_offset = static_cast<int>(predicted_progress / param_.interval);
      start_index = (now_state + predicted_index_offset) % size;
    }

    // 执行环形搜索
    for (int offset = -search_window; offset <= search_window; ++offset)
    {
      int current_index = (start_index + offset + size) % size;

      const double dx = trajectory_[current_index].pts.x - px;
      const double dy = trajectory_[current_index].pts.y - py;
      const double dist = std::hypot(dx, dy);

      if (dist < min_dist)
      {
        min_dist = dist;
        index_min = current_index;
      }
    }

    // 如果局部搜索失败，进行全局搜索
    if (min_dist > 2.0)
    {
      ROS_WARN("Local search failed (min_dist=%.2f), performing global search", min_dist);
      index_min = -1;
      min_dist = std::numeric_limits<double>::max();

      for (int i = 0; i < size; ++i)
      {
        const double dx = trajectory_[i].pts.x - px;
        const double dy = trajectory_[i].pts.y - py;
        const double dist = std::hypot(dx, dy);

        if (dist < min_dist)
        {
          min_dist = dist;
          index_min = i;
        }
      }

      if (min_dist > 3.0)
      {
        ROS_ERROR("Global search also failed, cannot find nearby trajectory point");
        return false;
      }
    }

    // 更新状态
    now_state = index_min;

    // 添加调试信息
    ROS_INFO_THROTTLE(1.0, "Nearest point: index=%d, distance=%.2f, vehicle_pos=(%.2f,%.2f)",
                      index_min, min_dist, px, py);

    // 生成参考轨迹
    refline.clear();
    const int N = param_.N;
    const double dt = param_.dt;
    const double desired_velocity = param_.desire_vel;
    double v = std::fmax(state_.v, param_.initial_velocity);
    double s_tmp = 0;

    for (int i = 0; i < N; ++i)
    {
      if (i != 0)
      {
        s_tmp += v * dt;
      }

      const int index = static_cast<int>(s_tmp / param_.interval);
      int next_index = (index + index_min) % size;

      const double delta_x = trajectory_[next_index].pts.x - px;
      const double delta_y = trajectory_[next_index].pts.y - py;
      double delta_yaw = trajectory_[next_index].yaw - psi;

      delta_yaw = std::fmod(delta_yaw + M_PI, 2 * M_PI);
      if (delta_yaw < 0)
        delta_yaw += 2 * M_PI;
      delta_yaw -= M_PI;

      const double temp_x = delta_x * cos(psi) + delta_y * sin(psi);
      const double temp_y = delta_y * cos(psi) - delta_x * sin(psi);

      TrajectoryPoint tmp;
      tmp.curvature = std::fabs(trajectory_[next_index].curvature);
      tmp.pts = cv::Point2f(temp_x, temp_y);
      tmp.yaw = delta_yaw;
      tmp.velocity = std::min(std::sqrt(param_.max_lat_acc / tmp.curvature), desired_velocity);

      refline.push_back(tmp);
    }
    ROS_INFO("轨迹点数量： %d", refline.size());
    return true;
  }
} // namespace ns_path_generator