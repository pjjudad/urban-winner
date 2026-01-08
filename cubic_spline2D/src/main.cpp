#include "../include/CatmullRom.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point32.h"
#include "fsd_common_msgs/Trajectory.h"
#include "fsd_common_msgs/TrajectoryPoint.h"
#include "fsd_common_msgs/CarState.h"
#include <visualization_msgs/Marker.h>      // 新增头文件
#include <visualization_msgs/MarkerArray.h> // 新增头文件
#include <std_msgs/String.h>
using namespace std;
using namespace Eigen;

// 全局变量声明
bool getPath = false;
vector<double> x;       // 排序后的路径点x坐标
vector<double> y;       // 排序后的路径点y坐标
vector<double> temp_x;  // 临时存储原始点x坐标
vector<double> temp_y;  // 临时存储原始点y坐标
vector<double> total_x; // 最终闭合路径x坐标
vector<double> total_y; // 最终闭合路径y坐标

fsd_common_msgs::CarState state;   // 车辆状态
ros::Publisher control_points_pub; // 控制点发布器
ros::Publisher marker_pub;         // 新增：标记发布器
ros::Publisher string_pub;
bool string_published_once = false;

// 计算两点间距离
double distance(double x1, double y1, double x2, double y2)
{
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

// 获取两点间的方向向量
Vector2d getDirectionVector(double x1, double y1, double x2, double y2)
{
    return Vector2d(x2 - x1, y2 - y1);
}

// 规范化角度到 [0, 2π) 范围
double normalizeAngle(double angle)
{
    while (angle < 0)
        angle += 2 * M_PI;
    while (angle >= 2 * M_PI)
        angle -= 2 * M_PI;
    return angle;
}

// 计算两个角度之间的最小差值（考虑圆周性）
double angleDifference(double a, double b)
{
    double diff = fabs(normalizeAngle(a) - normalizeAngle(b));
    return min(diff, 2 * M_PI - diff);
}

// ==================== 改进的距离过滤函数 ====================
void filterClosePoints(vector<double> &x, vector<double> &y, double min_distance_ratio = 0.3)
{
    if (x.size() < 3)
        return; // 至少需要3个点才能进行有意义的过滤

    // 保存原始点数
    size_t original_size = x.size();

    // 计算所有相邻点之间的距离
    vector<double> distances;
    double total_distance = 0.0;

    for (size_t i = 1; i < x.size(); i++)
    {
        double dist = distance(x[i - 1], y[i - 1], x[i], y[i]);
        distances.push_back(dist);
        total_distance += dist;
    }

    // 计算平均距离
    double avg_distance = total_distance / distances.size();

    // 计算距离的标准差
    double variance = 0.0;
    for (double dist : distances)
    {
        variance += pow(dist - avg_distance, 2);
    }
    double std_dev = sqrt(variance / distances.size());

    // 设置动态阈值：平均距离的一定比例，但不低于最小绝对值
    double dynamic_threshold = max(avg_distance * min_distance_ratio, 0.5);

    ROS_INFO("距离统计: 平均=%.2f, 标准差=%.2f, 阈值=%.2f", avg_distance, std_dev, dynamic_threshold);

    vector<double> filtered_x, filtered_y;

    // 保留第一个点
    filtered_x.push_back(x[0]);
    filtered_y.push_back(y[0]);

    // 遍历所有点，只保留距离大于动态阈值的点
    for (size_t i = 1; i < x.size(); i++)
    {
        double dist = distance(x[i], y[i], filtered_x.back(), filtered_y.back());

        if (dist >= dynamic_threshold)
        {
            filtered_x.push_back(x[i]);
            filtered_y.push_back(y[i]);
        }
        else
        {
            // 检查这个点是否是一个重要的拐点
            if (i < x.size() - 1)
            {
                // 计算前后方向变化
                Vector2d v1 = getDirectionVector(x[i - 1], y[i - 1], x[i], y[i]);
                Vector2d v2 = getDirectionVector(x[i], y[i], x[i + 1], y[i + 1]);

                v1.normalize();
                v2.normalize();

                double angle_change = acos(v1.dot(v2)); // 角度变化（弧度）

                // 如果角度变化较大，保留这个点（可能是拐点）
                if (angle_change > M_PI / 6)
                { // 30度阈值
                    filtered_x.push_back(x[i]);
                    filtered_y.push_back(y[i]);
                    ROS_DEBUG("保留点 %zu 由于显著的方向变化: %.1f 度",
                              i, angle_change * 180 / M_PI);
                    continue;
                }
            }

            ROS_WARN("跳过点 %zu 由于与前一节点距离过近 (距离: %.2f < %.2f)",
                     i, dist, dynamic_threshold);
        }
    }

    // 更新原始点集
    x = filtered_x;
    y = filtered_y;

    // 使用正确的原始点数
    ROS_INFO("近距离点已过滤: 剩余 %zu 个点 (原有点数 %zu)", x.size(), original_size);
}
// ==================== 改进的距离过滤函数结束 ====================

// 发布控制点云和序列号标记
void publishControlPoints(const vector<double> &x, const vector<double> &y)
{
    // 创建PCL point cloud
    pcl::PointCloud<pcl::PointXYZ> control_cloud_pcl;
    control_cloud_pcl.header.frame_id = "world";
    control_cloud_pcl.is_dense = false;

    for (size_t i = 0; i < x.size(); i++)
    {
        pcl::PointXYZ pt;
        pt.x = x[i];
        pt.y = y[i];
        pt.z = 0.5; // 设置一定高度以便可视化
        control_cloud_pcl.points.push_back(pt);
    }

    // 转换为ROS消息
    sensor_msgs::PointCloud2 control_cloud;
    pcl::toROSMsg(control_cloud_pcl, control_cloud);
    control_cloud.header.frame_id = "world";
    control_cloud.header.stamp = ros::Time::now();

    control_points_pub.publish(control_cloud);

    // 发布序列号标记
    visualization_msgs::MarkerArray marker_array;
    for (size_t i = 0; i < x.size(); i++)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "point_labels";
        marker.id = i;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x[i];
        marker.pose.position.y = y[i];
        marker.pose.position.z = 1.0; // 在点上方显示
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // 设置文本大小
        marker.scale.z = 0.5; // 文字高度

        // 设置颜色（蓝色）
        marker.color.a = 1.0; // 不透明度
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        // 设置文本内容为序列号
        marker.text = to_string(i);

        // 设置标记的生存时间（0表示永不过期）
        marker.lifetime = ros::Duration(0);

        marker_array.markers.push_back(marker);
    }

    // 实际发布标记数组
    marker_pub.publish(marker_array);
}

// 点云回调函数
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if (getPath)
        return;

    // 清空之前的数据
    x.clear();
    y.clear();
    temp_x.clear();
    temp_y.clear();

    // 转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // 收集不重复点，跳过(0,0)点
    for (const auto &point : cloud->points)
    {
        // 跳过接近(0,0)的点
        if (fabs(point.x) < 1e-5 && fabs(point.y) < 1e-5)
        {
            continue;
        }

        // 检查是否重复
        bool found = false;
        for (size_t j = 0; j < temp_x.size(); ++j)
        {
            if (distance(point.x, point.y, temp_x[j], temp_y[j]) < 0.5)
            {
                found = true;
                break;
            }
        }
        if (!found)
        {
            temp_x.push_back(point.x);
            temp_y.push_back(point.y);
        }
    }

    if (temp_x.empty())
    {
        ROS_WARN("未收到有效点");
        return;
    }

    // 创建点索引和距离的列表
    vector<pair<size_t, double>> point_distances;
    for (size_t i = 0; i < temp_x.size(); ++i)
    {
        double dist = distance(0, 0, temp_x[i], temp_y[i]);
        point_distances.push_back({i, dist});
    }

    // 按距离从近到远排序
    sort(point_distances.begin(), point_distances.end(),
         [](const pair<size_t, double> &a, const pair<size_t, double> &b)
         {
             return a.second < b.second;
         });

    // 按距离顺序检查每个点是否符合条件
    size_t start_index = 0;
    bool found_start = false;

    for (const auto &point : point_distances)
    {
        size_t idx = point.first;

        // 条件1: x为正数
        if (temp_x[idx] <= 0)
            continue;

        // 条件2: 在x的正方向上 (与x轴夹角小于45度)
        Vector2d dir_vector = getDirectionVector(0, 0, temp_x[idx], temp_y[idx]);
        dir_vector.normalize();
        Vector2d x_axis(1.0, 0.0);
        double angle = acos(dir_vector.dot(x_axis));

        if (angle < M_PI / 4) // 45度阈值
        {
            start_index = idx;
            found_start = true;
            ROS_INFO("找到符合条件的起点，索引: %zu, 坐标: (%.2f, %.2f), 距离: %.2f米",
                     start_index, temp_x[start_index], temp_y[start_index], point.second);
            break;
        }
    }

    if (!found_start)
    {
        ROS_ERROR("未找到符合条件的起点，无法继续处理");
        return;
    }

    x.push_back(temp_x[start_index]);
    y.push_back(temp_y[start_index]);
    temp_x.erase(temp_x.begin() + start_index);
    temp_y.erase(temp_y.begin() + start_index);

    // 当前前进方向（初始为x轴正方向）
    Vector2d current_dir(1.0, 0.0);

    // 输出排序前的点数量
    ROS_INFO("排序前临时点数量: %zu", temp_x.size());

    // 排序点云：使用最近邻算法，加入方向约束和距离过滤
    int max_attempts = 3;            // 最大尝试次数
    double distance_threshold = 8.0; // 初始距离阈值

    for (int attempt = 0; attempt < max_attempts && !temp_x.empty(); attempt++)
    {
        if (attempt > 0)
        {
            // 减小距离阈值
            distance_threshold *= 1.1;
            ROS_WARN("第 %d 次尝试，减小距离阈值至 %.2f 米", attempt + 1, distance_threshold);
        }

        // 保存当前临时点集，以便在多次尝试中使用
        vector<double> temp_x_copy = temp_x;
        vector<double> temp_y_copy = temp_y;

        // 清空已排序的点（除了起点）
        vector<double> sorted_x = {x[0]};
        vector<double> sorted_y = {y[0]};

        Vector2d current_dir(1.0, 0.0); // 重置当前方向

        while (!temp_x_copy.empty())
        {
            double min_dist = numeric_limits<double>::max();
            size_t min_index = numeric_limits<size_t>::max();

            // 寻找下一个点：同时考虑距离和方向
            for (size_t i = 0; i < temp_x_copy.size(); ++i)
            {
                Vector2d cand_dir = getDirectionVector(sorted_x.back(), sorted_y.back(), temp_x_copy[i], temp_y_copy[i]);
                double cand_dist = cand_dir.norm();

                // 跳过距离大于当前阈值的点
                if (cand_dist > distance_threshold)
                {
                    continue;
                }

                cand_dir.normalize();

                // 方向约束：点积大于0（夹角小于90度）
                // 对于第一个点（1号点），使用从0号点指向候选点的方向与x轴正方向的点积
                if (sorted_x.size() == 1)
                {
                    // 计算从0号点指向候选点的方向
                    Vector2d from_start = getDirectionVector(sorted_x[0], sorted_y[0], temp_x_copy[i], temp_y_copy[i]);
                    from_start.normalize();

                    // 与x轴正方向进行点积
                    Vector2d x_axis(1.0, 0.0);
                    if (x_axis.dot(from_start) <= 0.707)
                    {
                        continue; // 跳过反方向的点
                    }
                }
                else
                {
                    // 对于后续点，使用当前前进方向与候选点方向的点积
                    if (current_dir.dot(cand_dir) <= 0.2588)
                    {
                        continue;
                    }
                }

                if (cand_dist < min_dist)
                {
                    min_dist = cand_dist;
                    min_index = i;
                }
            }

            // 添加找到的点
            if (min_index != numeric_limits<size_t>::max())
            {
                sorted_x.push_back(temp_x_copy[min_index]);
                sorted_y.push_back(temp_y_copy[min_index]);

                // 更新当前
                if (sorted_x.size() >= 2)
                {
                    current_dir = getDirectionVector(sorted_x[sorted_x.size() - 2], sorted_y[sorted_y.size() - 2],
                                                     sorted_x.back(), sorted_y.back());
                    current_dir.normalize();
                }

                temp_x_copy.erase(temp_x_copy.begin() + min_index);
                temp_y_copy.erase(temp_y_copy.begin() + min_index);
            }
            else
            {
                // 输出无法找到合适点的警告
                ROS_WARN("无法找到满足条件的点，排序提前结束");
                break;
            }
        }

        // 分析剩余未处理点中被跳过的原因
        size_t skipped_by_distance = 0;
        size_t skipped_by_direction = 0;

        for (size_t i = 0; i < temp_x_copy.size(); ++i)
        {
            Vector2d cand_dir = getDirectionVector(sorted_x.back(), sorted_y.back(), temp_x_copy[i], temp_y_copy[i]);
            double cand_dist = cand_dir.norm();

            // 检查距离约束
            if (cand_dist > distance_threshold)
            {
                skipped_by_distance++;
                continue;
            }

            cand_dir.normalize();

            // 检查方向约束
            if (sorted_x.size() == 1)
            {
                Vector2d from_start = getDirectionVector(sorted_x[0], sorted_y[0], temp_x_copy[i], temp_y_copy[i]);
                from_start.normalize();
                Vector2d x_axis(1.0, 0.0);
                if (x_axis.dot(from_start) <= 0.0)
                {
                    skipped_by_direction++;
                    continue;
                }
            }
            else
            {
                if (current_dir.dot(cand_dir) <= 0.5)
                {
                    skipped_by_direction++;
                    continue;
                }
            }
        }

        // 输出排序结束时的点数量和跳过统计
        ROS_INFO("第 %d 次尝试排序完成: 已排序点数量 %zu, 剩余未处理点数量 %zu",
                 attempt + 1, sorted_x.size(), temp_x_copy.size());
        ROS_INFO("剩余未处理点分析: 因距离约束跳过 %zu 点, 因方向约束跳过 %zu 点, 其他原因 %zu 点",
                 skipped_by_distance, skipped_by_direction,
                 temp_x_copy.size() - skipped_by_distance - skipped_by_direction);

        // 如果这次尝试处理了大部分点，则接受结果
        if (temp_x_copy.size() < temp_x.size() * 0.3)
        { // 剩余点少于原有点数的30%
            x = sorted_x;
            y = sorted_y;
            temp_x = temp_x_copy;
            temp_y = temp_y_copy;
            ROS_INFO("接受第 %d 次尝试的结果", attempt + 1);
            break;
        }
        else if (attempt == max_attempts - 1)
        {
            // 最后一次尝试，即使结果不理想也接受
            x = sorted_x;
            y = sorted_y;
            temp_x = temp_x_copy;
            temp_y = temp_y_copy;
            ROS_WARN("已达到最大尝试次数，接受当前结果");
        }
    }
    // ==================== 距离过滤 ====================
    // 去除距离过近的点
    filterClosePoints(x, y, 0.5);
    // ==================== 距离过滤结束 ====================

    // 发布排序后的控制点
    publishControlPoints(x, y);
}

// 车辆状态回调函数
void carstateCallback(const fsd_common_msgs::CarState &msg)
{
    state = msg;
}

// 主函数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle n;
    ros::Rate loop_rate(20);
    setlocale(LC_ALL, ""); // 用于显示中文字符

    // 订阅者和发布者初始化
    ros::Subscriber sub = n.subscribe("/midpoints", 10, pointCloudCallback);
    ros::Subscriber carstate_sub = n.subscribe("/estimation/slam/state", 10, carstateCallback);

    // 修改：将spline_cloud话题的消息格式改为fsd_common_msgs::Trajectory
    ros::Publisher pub = n.advertise<fsd_common_msgs::Trajectory>("spline_cloud", 1000);

    // 修改其他可视化话题名称
    control_points_pub = n.advertise<sensor_msgs::PointCloud2>("visualization/control_points", 1000);
    marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization/point_markers", 10);

    // 新增：为轨迹可视化添加额外的点云发布器（可选）
    ros::Publisher trajectory_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("visualization/trajectory_cloud", 1000);

    //新增发布字符串话题
    string_pub = n.advertise<std_msgs::String>("string_topic",1);

    // 主循环
    while (ros::ok())
    {
        // 检查是否闭合路径
        if (!getPath && x.size() > 20)
        {
            double dist_to_start = distance(x.back(), y.back(), x[0], y[0]);
            if (dist_to_start < 5)
            {
                // 获取车辆当前状态
                double car_x = state.car_state.x;
                double car_y = state.car_state.y;
                double car_yaw = state.car_state.theta;

                // 计算车辆到路径起点的距离
                double car_to_start_dist = distance(car_x, car_y, x[0], y[0]);
                // 计算路径起点的朝向（使用前两个点）
                double start_yaw = atan2(y[1] - y[0], x[1] - x[0]);

                // 规范化角度到 [0, 2π) 范围
                car_yaw = normalizeAngle(car_yaw);
                start_yaw = normalizeAngle(start_yaw);

                // 计算车辆朝向与起点朝向的最小差值
                double yaw_diff = angleDifference(car_yaw, start_yaw);

                // 设置阈值
                double dist_threshold = 7.0;     // 车辆到起点的距离阈值（米）
                double yaw_threshold = M_PI / 2; // 朝向差值阈值（30度）

                // 检查车辆是否在起点附近且朝向匹配
                if (car_to_start_dist < dist_threshold && yaw_diff < yaw_threshold)
                {
                    // 确保控制点序列首尾相连
                    if (dist_to_start > 0.05)
                    {
                        x.push_back(x[0]);
                        y.push_back(y[0]);
                    }
                    total_x = x;
                    total_y = y;
                    getPath = true;
                    ROS_WARN("路径已成功闭合! 最终点数: %zu", total_x.size());

                    //发布一个字符串
                    if(!string_published_once)
                    {
                        std_msgs::String msg;
                        msg.data = "succeed";
                        string_pub.publish(msg);
                        ROS_INFO("已发布一个字符串succeed！");

                    }
                }
            }
        }

        // 生成路径
        vector<double> &current_x = getPath ? total_x : x;
        vector<double> &current_y = getPath ? total_y : y;

        if (current_x.size() >= 2 && current_y.size() >= 2 && current_x.size() == current_y.size())
        {
            if (current_x.size() <= 40 && current_y.size() <= 40 && current_x.size() == current_y.size())
            {
                current_x[0] = 0;
                current_y[0] = 0;
            }
            CatmullRom path_generator;
            vector<vector<double>> sp_path = path_generator.calc_path_with_xy_yaw_curv(
                current_x, current_y, 0.05);

            // 添加调试信息
            // ROS_INFO("控制点: %zu, 生成的路径点: %zu", current_x.size(), sp_path[0].size());

            if (!sp_path.empty() && !sp_path[0].empty())
            {
                // 创建Trajectory消息
                fsd_common_msgs::Trajectory trajectory_msg;
                trajectory_msg.header.stamp = ros::Time::now();
                trajectory_msg.header.frame_id = "world";

                // 填充轨迹点
                for (size_t i = 0; i < sp_path[0].size(); i++)
                {
                    fsd_common_msgs::TrajectoryPoint point;
                    point.pts.x = sp_path[0][i];
                    point.pts.y = sp_path[1][i];
                    point.yaw.data = sp_path[2][i];       // 航向角
                    point.curvature.data = sp_path[3][i]; // 曲率

                    // 修改这一行，使用正确的字段名
                    trajectory_msg.trajectory.push_back(point); // 根据参考代码，字段名应该是"trajectory"
                }

                // 发布Trajectory消息（保留spline_cloud话题）
                pub.publish(trajectory_msg);

                // 可选：同时发布点云用于可视化
                pcl::PointCloud<pcl::PointXYZ> trajectory_cloud_pcl;
                trajectory_cloud_pcl.header.frame_id = "world";
                trajectory_cloud_pcl.is_dense = false;

                for (size_t i = 0; i < sp_path[0].size(); i++)
                {
                    pcl::PointXYZ pt;
                    pt.x = sp_path[0][i];
                    pt.y = sp_path[1][i];
                    pt.z = 0;
                    trajectory_cloud_pcl.points.push_back(pt);
                }

                sensor_msgs::PointCloud2 trajectory_cloud;
                pcl::toROSMsg(trajectory_cloud_pcl, trajectory_cloud);
                trajectory_cloud.header.frame_id = "world";
                trajectory_cloud.header.stamp = ros::Time::now();
                trajectory_cloud_pub.publish(trajectory_cloud);
            }
        }
        else
        {
            ROS_WARN("生成路径失败");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
