#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <algorithm>
#include <vector>
#include <cmath>
#include <limits>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;
typedef K::Point_2 Point;
typedef Delaunay::Finite_faces_iterator Finite_faces_iterator;

ros::Publisher all_triangles_pub;
ros::Publisher inner_triangles_pub;
ros::Publisher midpoint_cloud_pub;

std::vector<Point> left_points;
std::vector<Point> right_points;
std::vector<Point> yellow_points;

// 赛道尺寸参数
double max_triangle_size = 10; // 最大三角形边长 (米)

// 辅助函数：计算两点间距离
double distanceBetweenPoints(const Point &p1, const Point &p2)
{
    double dx = p1.x() - p2.x();
    double dy = p1.y() - p2.y();
    return std::sqrt(dx * dx + dy * dy);
}

// 检查点是否在向量中
bool pointInVector(const std::vector<Point> &vec, const Point &point)
{
    return std::find(vec.begin(), vec.end(), point) != vec.end();
}

// 创建并发布所有三角形
void publishAllTriangles(Delaunay &triangulation)
{
    visualization_msgs::Marker triangles_marker;

    triangles_marker.header.frame_id = "world";
    triangles_marker.header.stamp = ros::Time::now();
    triangles_marker.ns = "all_triangles";
    triangles_marker.id = 0;
    triangles_marker.type = visualization_msgs::Marker::LINE_LIST;
    triangles_marker.action = visualization_msgs::Marker::ADD;
    triangles_marker.pose.orientation.w = 1.0;
    triangles_marker.scale.x = 0.1;
    triangles_marker.color.a = 1.0;
    triangles_marker.color.r = 0.0;
    triangles_marker.color.g = 0.0;
    triangles_marker.color.b = 1.0;

    for (Finite_faces_iterator it = triangulation.finite_faces_begin(); it != triangulation.finite_faces_end(); ++it)
    {
        for (int i = 0; i < 3; i++)
        {
            Point v1 = it->vertex(i)->point();
            Point v2 = it->vertex((i + 1) % 3)->point();
            geometry_msgs::Point p1, p2;
            p1.x = v1.x();
            p1.y = v1.y();
            p2.x = v2.x();
            p2.y = v2.y();
            triangles_marker.points.push_back(p1);
            triangles_marker.points.push_back(p2);
        }
    }

    all_triangles_pub.publish(triangles_marker);
}

// 创建并发布内部三角形和中点
void createAndPublishInnerTrianglesAndMidpoints(
    const std::vector<Point> &left_points,
    const std::vector<Point> &right_points)
{
    ROS_INFO("使用参数: 最大三角形尺寸=%.2f米", max_triangle_size);

    Delaunay triangulation;
    std::vector<Point> all_points = left_points;
    all_points.insert(all_points.end(), right_points.begin(), right_points.end());
    triangulation.insert(all_points.begin(), all_points.end());

    // 初始化可视化标记
    visualization_msgs::Marker inner_triangles_marker;
    sensor_msgs::PointCloud2 midpoint_cloud;
    midpoint_cloud.header.frame_id = "world";
    midpoint_cloud.header.stamp = ros::Time::now();

    inner_triangles_marker.header.frame_id = "world";
    inner_triangles_marker.header.stamp = ros::Time::now();
    inner_triangles_marker.ns = "inner_triangles";
    inner_triangles_marker.id = 0;
    inner_triangles_marker.type = visualization_msgs::Marker::LINE_LIST;
    inner_triangles_marker.action = visualization_msgs::Marker::ADD;
    inner_triangles_marker.pose.orientation.w = 1.0;
    inner_triangles_marker.scale.x = 0.1;
    inner_triangles_marker.color.a = 1.0;
    inner_triangles_marker.color.r = 1.0;
    inner_triangles_marker.color.g = 0.0;
    inner_triangles_marker.color.b = 0.0;

    int valid_triangles = 0;
    int invalid_triangles = 0;
    int edges_processed = 0;

    // 创建PCL点云用于存储中点
    pcl::PointCloud<pcl::PointXYZ> midpoints_pcl;
    midpoints_pcl.header.frame_id = "world";
    midpoints_pcl.is_dense = false;

    // 遍历所有三角形
    for (Finite_faces_iterator it = triangulation.finite_faces_begin();
         it != triangulation.finite_faces_end();
         ++it)
    {
        Point v1 = it->vertex(0)->point();
        Point v2 = it->vertex(1)->point();
        Point v3 = it->vertex(2)->point();

        // 检查顶点是否属于左侧点集
        bool v1_in_left = pointInVector(left_points, v1);
        bool v2_in_left = pointInVector(left_points, v2);
        bool v3_in_left = pointInVector(left_points, v3);

        // 计算左侧顶点数量
        int count_left = (v1_in_left ? 1 : 0) +
                         (v2_in_left ? 1 : 0) +
                         (v3_in_left ? 1 : 0);

        // 右侧顶点数量 = 总顶点数(3) - 左侧顶点数
        int count_right = 3 - count_left;

        // 仅处理跨越左右边界的三角形
        if (count_left > 0 && count_right > 0)
        {
            // 计算三角形尺寸
            double edge1 = distanceBetweenPoints(v1, v2);
            double edge2 = distanceBetweenPoints(v2, v3);
            double edge3 = distanceBetweenPoints(v3, v1);
            double max_edge = std::max({edge1, edge2, edge3});

            // 只处理符合尺寸的三角形
            if (max_edge <= max_triangle_size)
            {
                valid_triangles++;
                std::vector<Point> vertices = {v1, v2, v3, v1};

                // 绘制三角形三边
                for (size_t i = 0; i < vertices.size() - 1; ++i)
                {
                    geometry_msgs::Point p1, p2;
                    p1.x = vertices[i].x();
                    p1.y = vertices[i].y();
                    p2.x = vertices[i + 1].x();
                    p2.y = vertices[i + 1].y();
                    inner_triangles_marker.points.push_back(p1);
                    inner_triangles_marker.points.push_back(p2);
                }

                // 处理边界边
                for (int i = 0; i < 3; ++i)
                {
                    Point p1 = vertices[i];
                    Point p2 = vertices[(i + 1) % 3];

                    // 检查是否为跨边界边
                    bool is_crossing_edge = false;
                    if (pointInVector(left_points, p1) && pointInVector(right_points, p2))
                    {
                        is_crossing_edge = true;
                    }
                    else if (pointInVector(left_points, p2) && pointInVector(right_points, p1))
                    {
                        is_crossing_edge = true;
                    }

                    if (is_crossing_edge)
                    {
                        edges_processed++;
                        Point midpoint = CGAL::midpoint(p1, p2);
                        pcl::PointXYZ mid;
                        mid.x = midpoint.x();
                        mid.y = midpoint.y();
                        mid.z = 0.0; // 添加z坐标
                        midpoints_pcl.points.push_back(mid);
                    }
                }
            }
            else
            {
                invalid_triangles++;
                ROS_DEBUG("拒绝三角形: 最大边长=%.2f米 (最大允许=%.2f)",
                          max_edge, max_triangle_size);
            }
        }
    }

    // 转换PCL点云为ROS PointCloud2消息
    pcl::toROSMsg(midpoints_pcl, midpoint_cloud);

    // 发布结果
    ROS_INFO("处理结果: 有效三角形=%d, 无效三角形=%d | 处理的边界边=%d",
             valid_triangles, invalid_triangles, edges_processed);

    inner_triangles_pub.publish(inner_triangles_marker);
    midpoint_cloud_pub.publish(midpoint_cloud);
}

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
{
    left_points.clear();
    right_points.clear();
    yellow_points.clear();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    for (const auto &p : cloud->points)
    {
        float intensity = p.intensity;

        if (intensity == 1.0)
        { // 红色
            left_points.push_back(Point(p.x, p.y));
        }
        else if (intensity == 255.0)
        { // 蓝色
            right_points.push_back(Point(p.x, p.y));
        }
        else if (intensity == 128.0)
        { // 黄色
            yellow_points.push_back(Point(p.x, p.y));
        }
    }

    if (!left_points.empty() && !right_points.empty())
    {
        createAndPublishInnerTrianglesAndMidpoints(left_points, right_points);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delaunay_triangulation");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~"); // 私有节点句柄
    setlocale(LC_ALL, "");           // 用于显示中文字符

    // 从私有命名空间获取参数
    private_nh.param("max_triangle_size", max_triangle_size, 5.0);

    ROS_INFO("启动Delaunay三角剖分节点，使用参数:");
    ROS_INFO("  最大三角形尺寸: %.2f 米", max_triangle_size);

    all_triangles_pub = nh.advertise<visualization_msgs::Marker>("all_triangles", 1);
    inner_triangles_pub = nh.advertise<visualization_msgs::Marker>("inner_triangles", 1);
    midpoint_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("midpoints", 1);

    ros::Subscriber sub = nh.subscribe("/mapped/aft_lidar_cluster", 1, pointCloudCallback);

    ros::spin();
    return 0;
}