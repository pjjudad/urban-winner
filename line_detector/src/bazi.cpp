#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <vector>
#include <random>

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

// ROS 全局变量
ros::Subscriber point_cloud_sub;
ros::Publisher end_point_pub;
ros::Publisher transformMatrixPublisher;

bool getPathOk = false;



struct Point {
    float x, y, z;
};

// 直线结构体，包含一个点和方向向量
struct Line {
    Eigen::Vector3f point;
    Eigen::Vector3f direction;
};

float distanceToLine(const Eigen::Vector3f& point, const Eigen::Vector3f& point1, const Eigen::Vector3f& direction) {  
    Eigen::Vector3f v = point - point1;  
    Eigen::Vector3f w = direction.cross(v).normalized();  
    float d = w.dot(v);  
    return std::abs(d) / direction.norm();  
}  

// RANSAC 直线拟合函数
bool fitLineRANSAC(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, Line& line, float threshold, int maxIterations) {
    std::random_device rd; // 随机数生成器
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, cloud->size() - 1);

    int bestInliersCount = 0;
    Line bestLine;

    // 迭代次数
    for (int i = 0; i < maxIterations; ++i) {
        int index1 = dis(gen); // 随机选择第一个点
        int index2 = dis(gen); // 随机选择第二个点
        while (index1 == index2) { // 确保两个点不同
            index2 = dis(gen);
        }

        const pcl::PointXYZ& p1 = cloud->points[index1];
        const pcl::PointXYZ& p2 = cloud->points[index2];

        Eigen::Vector3f point1(p1.x, p1.y, p1.z);
        Eigen::Vector3f point2(p2.x, p2.y, p2.z);

        Eigen::Vector3f direction = (point2 - point1).normalized(); // 计算直线方向

        int inliersCount = 0;
        // 计算内点数量

        for (const auto& point : cloud->points) {
            Eigen::Vector3f p(point.x, point.y, point.z);
            Eigen::Vector3f v = p - point1;
            float distance = std::abs(v.cross(direction).norm()); // 点到直线的距离
            if (distance < threshold) {
                ++inliersCount;
            }
        }

        // 如果当前拟合直线的内点数量大于之前记录的最佳内点数量，则更新 bestInliersCount 和 bestLine
        if (inliersCount > bestInliersCount) {
            bestInliersCount = inliersCount;
            bestLine.point = point1;
            bestLine.direction = direction;
        }
    }

    // 如果找到最佳拟合直线，返回 true
    if (bestInliersCount > 0) {
        line = bestLine;
        return true;
    }

    return false;
}

// 计算两条直线的交点
bool findIntersection(const Line& line1, const Line& line2, Eigen::Vector3f& intersection) {
    Eigen::Vector3f cross_product = line1.direction.cross(line2.direction);
    if (cross_product.norm() == 0) { // 如果两条直线平行
        ROS_WARN("parallel");
        return false;
    }

    Eigen::Vector3f w = line1.point - line2.point;
    double s = (line2.direction.cross(w)).dot(cross_product) / cross_product.squaredNorm();
    intersection = line1.point + s * line1.direction;
    ROS_INFO("have point of intersection");
    return true;
}

// 计算终点
Eigen::Vector3f findEndPoint(const Line& line1, const Line& line2, double distance) {
    // 计算每条线在 x 为 distance 处的 y 值
    double y1 = line1.point.y() + (distance - line1.point.x()) * (line1.direction.y() / line1.direction.x());
    double y2 = line2.point.y() + (distance - line2.point.x()) * (line2.direction.y() / line2.direction.x());

    // 求这两个 y 值的平均值
    double avg_y = (y1 + y2) / 2.0;

    return Eigen::Vector3f(distance, avg_y, 0);
}


// 点云回调函数
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    //if(getPathOk == true)
    //{
    //    return;
    //}

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud); // 将 ROS 消息转换为 PCL 点云

    pcl::PointCloud<pcl::PointXYZ>::Ptr left_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr right_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 分离左右点云
    for (const auto& point : cloud->points) {
        if (point.y < 0) {
            left_cloud->points.push_back(point);
        } else if (point.y > 0) {
            right_cloud->points.push_back(point);
        }
    }

    // 如果左右点云为空，返回
/*
    if (left_cloud->points.empty() || right_cloud->points.empty()) { //后面改称小于2
        ROS_WARN("have not enough point");
        return;
    }
*/
    if (left_cloud->points.size() < 2 || right_cloud->points.size() < 2) { //后面改称小于2
        ROS_WARN("have not enough point");
        return;
    }

    Line left_line, right_line;
    // 使用 RANSAC 拟合左右直线
    if (!fitLineRANSAC(left_cloud, left_line, 0.1, 100) || !fitLineRANSAC(right_cloud, right_line, 0.1, 100)) {
        ROS_WARN("fit warn");
        return;
    }

    Eigen::Vector3f intersection;
    // 计算交点
    if (!findIntersection(left_line, right_line, intersection)) {
        ROS_WARN("line disjoint");
        return;
    }

    double intersection_distance = intersection.norm();
    // 如果交点距离小于 200 米，返回
    if (intersection_distance < 100) {
        ROS_WARN("<200m");
        return;
    }

    // 计算终点
    Eigen::Vector3f end_point = findEndPoint(left_line, right_line, 30);
    Eigen::Vector3f start_point = findEndPoint(left_line, right_line, 0);
    geometry_msgs::Point end_point_msg start_point;
    end_point_msg.x = end_point.x();
    end_point_msg.y = end_point.y();
    end_point_msg.z = end_point.z();
    start_point_msg.x = start_point_msg.x();
    start_point_msg.y = start_point_msg.y();
    start_point_msg.z = start_point_msg.z();
    if(end_point_msg.x > 25 && end_point_msg.x < 35 && end_point_msg.y > -5 && end_point_msg.y < 5 &&
       start_point_msg.x > -2 && start_point_msg.x < 2 && start_point_msg.y > -2 && start_point_msg.y < 2 )
    {
        getPathOk = true;
    } 

    // 目标点
    Point target_end_point = {30, 0, 0};
    Point target_start_point = {0, 0, 0};

    Point end_point_p = {};
    Point start_point_p ={};

    end_point_p.x = end_point_msg.x;
    end_point_p.y = end_point_msg.y;
    end_point_p.z = end_point_msg.z;

    start_point_p.x = start_point_msg.x;
    start_point_p.y = start_point_msg.y;
    start_point_p.z = start_point_msg.z;

    // 目标点和当前点反了，必须这样做才能运行下面的程序
    Point t_left = target_end_point;
    Point t_right = target_start_point;
    target_end_point = end_point_p;
    target_start_point = start_point_p;
    end_point_p = t_left;
    start_point_p = t_right;

    // 计算平均点
    Eigen::Vector3f target_mean = (Eigen::Vector3f() << target_end_point.x, target_end_point.y, target_end_point.z).finished() +
                                  (Eigen::Vector3f() << target_start_point.x, target_start_point.y, target_start_point.z).finished();
    target_mean /= 2.0;

    Eigen::Vector3f current_mean = (Eigen::Vector3f() << end_point_p.x, end_point_p.y, end_point_p.z).finished() +
                                   (Eigen::Vector3f() << start_point_p.x, start_point_p.y, start_point_p.z).finished();
    current_mean /= 2.0;

    // 计算协方差矩阵
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    H += ((Eigen::Vector3f() << end_point_p.x, end_point_p.y, end_point_p.z).finished() - current_mean) *
         ((Eigen::Vector3f() << target_end_point.x, target_end_point.y, target_end_point.z).finished() - target_mean).transpose();
    H += ((Eigen::Vector3f() << start_point_p.x, start_point_p.y, start_point_p.z).finished() - current_mean) *
         ((Eigen::Vector3f() << target_start_point.x, target_start_point.y, target_start_point.z).finished() - target_mean).transpose();

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
}

// 主函数
int main(int argc, char** argv) {
    ros::init(argc, argv, "lane_detector"); // 初始化 ROS 节点
    ros::NodeHandle nh;

    // 订阅点云消息，发布终点消息和可视化标记
    point_cloud_sub = nh.subscribe("/perception/cluster_cloud", 1, pointCloudCallback);
    // 发布变换矩阵
    transformMatrixPublisher = nh.advertise<std_msgs::Float64MultiArray>("/transform_matrix", 1);

    ROS_INFO("start");
    ros::spin(); // 进入事件循环
    return 0;
}
