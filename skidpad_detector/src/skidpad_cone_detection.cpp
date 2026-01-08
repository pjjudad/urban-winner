#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>

// 定义点的结构体
struct Point {
    float x, y, z;
};

Point center_of_midpoints = {0, 0, 0};

// 定义左右锥桶的点云向量
std::vector<Point> left_cones, right_cones;

ros::Publisher transformMatrixPublisher;

// 点云回调函数，处理接收到的点云数据
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // 将ROS消息转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    // 清空左右锥桶的点云向量
    left_cones.clear();
    right_cones.clear();

    // 遍历点云中的每个点
    for (const auto& point : cloud) {
        // 筛选出x坐标在12到18之间的点
        if (point.x > 12 && point.x < 18) {
            // 根据y坐标将点分为左右锥桶
            if (point.y > 0) {
                left_cones.push_back({point.x, point.y, point.z});
            } else {
                right_cones.push_back({point.x, point.y, point.z});
            }
        }
    }

    // 打印左锥桶的点
    ROS_INFO("Left cones:");
    for (const auto& cone : left_cones) {
        ROS_INFO("(%.2f, %.2f, %.2f)", cone.x, cone.y, cone.z);
    }

    // 打印右锥桶的点
    ROS_INFO("Right cones:");
    for (const auto& cone : right_cones) {
        ROS_INFO("(%.2f, %.2f, %.2f)", cone.x, cone.y, cone.z);
    }

    // 定义匹配对的向量
    std::vector<std::pair<Point, Point>> matched_pairs;

    // 以最小的距离进行两两匹配
    for (const auto& left : left_cones) {
        float min_distance = std::numeric_limits<float>::max();
        const Point* best_match = nullptr;

        // 遍历右锥桶的点，寻找与当前左锥桶点距离最小的点
        for (const auto& right : right_cones) {
            float distance = std::sqrt(std::pow(right.x - left.x, 2) + std::pow(right.y - left.y, 2));
            if (distance < min_distance) {
                min_distance = distance;
                best_match = &right;
            }
        }

        // 如果找到了匹配的右锥桶点
        if (best_match != nullptr) {
            // 计算匹配对的斜率
            float slope = std::abs(best_match->x - left.x) / (best_match->y - left.y);
            // 如果斜率小于等于0.3且距离小于3.6，则认为匹配成功
            if (slope <= 0.3 && slope >= -0.3 && min_distance < 3.6) {
                matched_pairs.push_back({left, *best_match});
                ROS_INFO("Matched pair: Left (%.2f, %.2f, %.2f) - Right (%.2f, %.2f, %.2f)",
                         left.x, left.y, left.z, best_match->x, best_match->y, best_match->z);
                ROS_INFO("Distance: %.2f, Slope: %.2f", min_distance, slope);
                // 从右锥桶向量中移除已匹配的点，避免重复匹配
                right_cones.erase(std::remove_if(right_cones.begin(), right_cones.end(),
                                                 [&](const Point& p) { return p.x == best_match->x && p.y == best_match->y; }),
                                  right_cones.end());
            }
        }
    }

    // 如果没有找到匹配对
    if (matched_pairs.empty()) {
        ROS_INFO("No matched pairs found.");
        return;
    }

    // 计算匹配对的中点
    Point t_left_midpoint = {0, 0, 0}, t_right_midpoint = {0, 0, 0};
    for (const auto& pair : matched_pairs) {
        Point left_point = {pair.first.x, pair.first.y, pair.first.z};
        Point right_point = {pair.second.x, pair.second.y, pair.second.z};

        t_left_midpoint.x += left_point.x;
        t_left_midpoint.y += left_point.y;
        t_left_midpoint.z += left_point.z;

        t_right_midpoint.x += right_point.x;
        t_right_midpoint.y += right_point.y;
        t_right_midpoint.z += right_point.z;
    }

    Point left_midpoint, right_midpoint;
    left_midpoint.x = t_left_midpoint.x / matched_pairs.size();
    left_midpoint.y = t_left_midpoint.y / matched_pairs.size();
    left_midpoint.z = t_left_midpoint.z / matched_pairs.size();

    right_midpoint.x = t_right_midpoint.x / matched_pairs.size();
    right_midpoint.y = t_right_midpoint.y / matched_pairs.size();
    right_midpoint.z = t_right_midpoint.z / matched_pairs.size();

    // 打印左右两边各自的中点
    ROS_INFO("Left midpoint: (%.2f, %.2f, %.2f)", left_midpoint.x, left_midpoint.y, left_midpoint.z);
    ROS_INFO("Right midpoint: (%.2f, %.2f, %.2f)", right_midpoint.x, right_midpoint.y, right_midpoint.z);

    // 目标点
    Point target_left = {15, 1.5, 0};
    Point target_right = {15, -1.5, 0};

    // 目标点和当前点反了，必须这样做才能运行下面的程序
    Point t_left = target_left;
    Point t_right = target_right;
    target_left = left_midpoint;
    target_right = right_midpoint;
    left_midpoint = t_left;
    right_midpoint = t_right;

    // 计算平均点
    Eigen::Vector3f target_mean = (Eigen::Vector3f() << target_left.x, target_left.y, target_left.z).finished() +
                                  (Eigen::Vector3f() << target_right.x, target_right.y, target_right.z).finished();
    target_mean /= 2.0;

    Eigen::Vector3f current_mean = (Eigen::Vector3f() << left_midpoint.x, left_midpoint.y, left_midpoint.z).finished() +
                                   (Eigen::Vector3f() << right_midpoint.x, right_midpoint.y, right_midpoint.z).finished();
    current_mean /= 2.0;

    // 计算协方差矩阵
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    H += ((Eigen::Vector3f() << left_midpoint.x, left_midpoint.y, left_midpoint.z).finished() - current_mean) *
         ((Eigen::Vector3f() << target_left.x, target_left.y, target_left.z).finished() - target_mean).transpose();
    H += ((Eigen::Vector3f() << right_midpoint.x, right_midpoint.y, right_midpoint.z).finished() - current_mean) *
         ((Eigen::Vector3f() << target_right.x, target_right.y, target_right.z).finished() - target_mean).transpose();

    // 通过SVD计算旋转矩阵
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();

    // 计算平移向量
    Eigen::Vector3f T = target_mean - R * current_mean;

    // 打印旋转矩阵和平移向量
    ROS_INFO("Rotation matrix:");
    ROS_INFO("| %.4f %.4f %.4f |", R(0, 0), R(0, 1), R(0, 2));
    ROS_INFO("| %.4f %.4f %.4f |", R(1, 0), R(1, 1), R(1, 2));
    ROS_INFO("| %.4f %.4f %.4f |", R(2, 0), R(2, 1), R(2, 2));

    ROS_INFO("Translation vector:");
    ROS_INFO("| %.4f |", T(0));
    ROS_INFO("| %.4f |", T(1));
    ROS_INFO("| %.4f |", T(2));

    // 创建4x4变换矩阵
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    transformation.block<3, 3>(0, 0) = R;
    transformation.block<3, 1>(0, 3) = T;

    // 将变换矩阵发布为Float64MultiArray消息
    std_msgs::Float64MultiArray trans_matrix_in_1D;
    trans_matrix_in_1D.data.clear();
    for (int i = 0; i < transformation.rows(); i++) {
        for (int j = 0; j < transformation.cols(); j++) {
            trans_matrix_in_1D.data.push_back(transformation(i, j));
        }
    }
    transformMatrixPublisher.publish(trans_matrix_in_1D);

    // 计算所有中点的中心点
    Point center_of_midpoints = {0, 0, 0};

    center_of_midpoints.x = (left_midpoint.x + right_midpoint.x) / 2;
    center_of_midpoints.y = (left_midpoint.y + right_midpoint.y) / 2;
    center_of_midpoints.z = (left_midpoint.z + right_midpoint.z) / 2;

    // 打印中点的中心点
    ROS_INFO("Center of midpoints: (%.2f, %.2f, %.2f)", center_of_midpoints.x, center_of_midpoints.y, center_of_midpoints.z);
    ROS_INFO("-------------------------------");
}

// 主函数
int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "cone_detection");
    ros::NodeHandle nh;

    // 订阅点云话题
    ros::Subscriber sub = nh.subscribe("/perception/cluster_cloud", 1, pointCloudCallback);

    // 发布变换矩阵
    transformMatrixPublisher = nh.advertise<std_msgs::Float64MultiArray>("/transform_matrix", 1);

    // 进入ROS事件循环
    ros::spin();

    return 0;
}
