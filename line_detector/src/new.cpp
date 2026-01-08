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
#include <set>

// ROS 全局变量
ros::Subscriber point_cloud_sub;
ros::Publisher end_point_pub;
ros::Publisher marker_pub;

//bool getPathOk = false;
// 存储最接近0的点
geometry_msgs::Point closest_to_zero_point;
bool has_closest_point = false;

// 新增：存储上一次的点云用于比较
pcl::PointCloud<pcl::PointXYZ>::Ptr last_cloud(new pcl::PointCloud<pcl::PointXYZ>);
bool has_last_cloud = false;
bool should_refit = true;  // 初始需要拟合

// 新增：存储上一次的拟合结果
struct Line {
    Eigen::Vector3f point;
    Eigen::Vector3f direction;
    float slope;
};
Line last_left_line, last_right_line;
bool has_last_fit_result = false;
geometry_msgs::Point last_end_point;

// 新增：点云哈希函数，用于快速比较点云是否相同
std::size_t computePointCloudHash(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    std::size_t hash = 0;
    for (const auto& point : cloud->points) {
        // 将点的坐标转换为哈希，使用简单的哈希组合
        hash ^= std::hash<float>{}(point.x) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        hash ^= std::hash<float>{}(point.y) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        hash ^= std::hash<float>{}(point.z) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    }
    return hash;
}

// 新增：比较两个点云是否相同
bool arePointCloudsEqual(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1, 
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2) {
    if (cloud1->size() != cloud2->size()) {
        return false;
    }
    
    // 使用哈希进行快速比较
    std::size_t hash1 = computePointCloudHash(cloud1);
    std::size_t hash2 = computePointCloudHash(cloud2);
    
    if (hash1 != hash2) {
        return false;
    }
    
    // 哈希相同的情况下，进一步精确比较（可选，如果担心哈希冲突）
    for (size_t i = 0; i < cloud1->size(); ++i) {
        if (cloud1->points[i].x != cloud2->points[i].x ||
            cloud1->points[i].y != cloud2->points[i].y ||
            cloud1->points[i].z != cloud2->points[i].z) {
            return false;
        }
    }
    
    return true;
}

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
            bestLine.slope = direction.y() / direction.x(); // 存储斜率
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

// 发布可视化标记
void publishVisualizationMarkers(const Line& left_line, const Line& right_line) {
    visualization_msgs::Marker line_left, line_right;

    // 设置标记的通用属性
    line_left.header.frame_id = line_right.header.frame_id = "world";
    line_left.header.stamp = line_right.header.stamp = ros::Time::now();
    line_left.ns = line_right.ns = "line_detector";
    line_left.action = line_right.action = visualization_msgs::Marker::ADD;
    line_left.pose.orientation.w = line_right.pose.orientation.w = 1.0;
    line_left.id = 0;
    line_right.id = 1;
    line_left.type = line_right.type = visualization_msgs::Marker::LINE_STRIP;
    line_left.scale.x = line_right.scale.x = 0.2; // 线宽
    line_left.color.r = 1.0;
    line_left.color.a = 1.0;
    line_right.color.g = 1.0;
    line_right.color.a = 1.0;

    double path_length = 100.0; // 路径长度

    // 左直线标记点
    geometry_msgs::Point p_left_start, p_left_end;
    p_left_start.x = 0;
    p_left_start.y = left_line.point.y() + (0 - left_line.point.x()) * left_line.direction.y() / left_line.direction.x();
    p_left_start.z = left_line.point.z() + (0 - left_line.point.x()) * left_line.direction.z() / left_line.direction.x();
    p_left_end.x = path_length;
    p_left_end.y = left_line.point.y() + (path_length - left_line.point.x()) * left_line.direction.y() / left_line.direction.x();
    p_left_end.z = left_line.point.z() + (path_length - left_line.point.x()) * left_line.direction.z() / left_line.direction.x();
    line_left.points.push_back(p_left_start);
    line_left.points.push_back(p_left_end);

    // 右直线标记点
    geometry_msgs::Point p_right_start, p_right_end;
    p_right_start.x = 0;
    p_right_start.y = right_line.point.y() + (0 - right_line.point.x()) * right_line.direction.y() / right_line.direction.x();
    p_right_start.z = right_line.point.z() + (0 - right_line.point.x()) * right_line.direction.z() / right_line.direction.x();
    p_right_end.x = path_length;
    p_right_end.y = right_line.point.y() + (path_length - right_line.point.x()) * right_line.direction.y() / right_line.direction.x();
    p_right_end.z = right_line.point.z() + (path_length - right_line.point.x()) * right_line.direction.z() / right_line.direction.x();
    line_right.points.push_back(p_right_start);
    line_right.points.push_back(p_right_end);

    // 发布标记
    marker_pub.publish(line_left);
    marker_pub.publish(line_right);
}

// 点云回调函数
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    ROS_INFO("Received point cloud with %d points", cloud_msg->width * cloud_msg->height);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud); // 将 ROS 消息转换为 PCL 点云

    // 新增：检查点云是否更新
    if (has_last_cloud) {
        if (arePointCloudsEqual(cloud, last_cloud)) {
            ROS_INFO("Point cloud unchanged, using previous fit result");
            should_refit = false;
        } else {
            ROS_INFO("Point cloud updated, refitting lines");
            should_refit = true;
        }
    } else {
        // 第一次运行，需要拟合
        should_refit = true;
    }

    // 更新上一次的点云
    *last_cloud = *cloud;
    has_last_cloud = true;

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

    if (left_cloud->points.size() < 2 || right_cloud->points.size() < 2) {
        ROS_WARN("have not enough point");
        return;
    }

    Line left_line, right_line;
    geometry_msgs::Point end_point_msg;

    // 新增：根据是否需要重新拟合来选择执行路径
    if (should_refit) {
        // 需要重新拟合
        ROS_INFO("Performing RANSAC line fitting");

        // 使用 RANSAC 拟合左右直线
        if (!fitLineRANSAC(left_cloud, left_line, 0.12, 100) || !fitLineRANSAC(right_cloud, right_line, 0.12, 100)) {
            ROS_WARN("fit warn");
            
            // 如果拟合失败但有上一次的结果，使用上一次的结果
            if (has_last_fit_result) {
                ROS_WARN("Using last valid fit result due to fitting failure");
                left_line = last_left_line;
                right_line = last_right_line;
                end_point_msg = last_end_point;
            } else {
                return;
            }
        } else {
            // 拟合成功，计算交点和终点
            Eigen::Vector3f intersection;
            if (!findIntersection(left_line, right_line, intersection)) {
                ROS_WARN("line disjoint");
                if (has_last_fit_result) {
                    ROS_WARN("Using last valid fit result due to disjoint lines");
                    left_line = last_left_line;
                    right_line = last_right_line;
                    end_point_msg = last_end_point;
                } else {
                    return;
                }
            } else {
                double intersection_distance = intersection.norm();
                if (intersection_distance < 100) {
                    ROS_WARN("<100m");
                    if (has_last_fit_result) {
                        ROS_WARN("Using last valid fit result due to close intersection");
                        left_line = last_left_line;
                        right_line = last_right_line;
                        end_point_msg = last_end_point;
                    } else {
                        return;
                    }
                } else {
                    ROS_INFO(">100m");
                    // 计算终点
                    Eigen::Vector3f end_point = findEndPoint(left_line, right_line, 180);
                    end_point_msg.x = end_point.x();
                    end_point_msg.y = end_point.y();
                    end_point_msg.z = end_point.z();
                    
                    // 保存当前的拟合结果
                    last_left_line = left_line;
                    last_right_line = right_line;
                    last_end_point = end_point_msg;
                    has_last_fit_result = true;
                    
                    ROS_INFO("Left line slope: %f", left_line.slope);
                    ROS_INFO("Right line slope: %f", right_line.slope);
                }
            }
        }
    } else {
        // 使用上一次的拟合结果
        ROS_INFO("Using previous fit result");
        left_line = last_left_line;
        right_line = last_right_line;
        end_point_msg = last_end_point;
    }

    // 新逻辑：如果新点比旧点更接近0，则更新并发送新点
    if (!has_closest_point || std::abs(end_point_msg.y) < std::abs(closest_to_zero_point.y)) {
        closest_to_zero_point = end_point_msg;
        has_closest_point = true;
        ROS_INFO("Found closer point to zero: x=%f, y=%f", end_point_msg.x, end_point_msg.y);
    }

    // 总是发送最接近0的点
    end_point_pub.publish(closest_to_zero_point);
    ROS_INFO("Publishing closest to zero point: x=%f, y=%f", closest_to_zero_point.x, closest_to_zero_point.y);

    // 发布可视化标记
    publishVisualizationMarkers(left_line, right_line);
}

// 主函数
int main(int argc, char** argv) {
    ros::init(argc, argv, "line_detector"); // 初始化 ROS 节点
    ros::NodeHandle nh;

    // 订阅点云消息，发布终点消息和可视化标记
    point_cloud_sub = nh.subscribe("/cluster_cloud", 1, pointCloudCallback);
    end_point_pub = nh.advertise<geometry_msgs::Point>("/planning/end_point", 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    ROS_INFO("start");
    ros::spin(); // 进入事件循环
    return 0;
}
