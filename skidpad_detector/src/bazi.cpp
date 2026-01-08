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
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>

#include <std_msgs/Float64MultiArray.h>


#include <pcl/io/pcd_io.h>    //新增
#include <pcl/registration/ndt.h> //新
#include <pcl/registration/icp.h>  //新
#include <fstream> // 新增头文件
#include <memory>  // 添加头文件
// 新增头文件（在文件头部添加）
#include <pcl/filters/statistical_outlier_removal.h>  // 统计离群值去除
#include <pcl/filters/voxel_grid.h>  // 体素网格滤波
#include <std_msgs/Float32.h>
#include <limits> // 用于 std::numeric_limits
#include <pcl/common/pca.h>

// 全局最佳RMSE和变换矩阵
float best_rmse = std::numeric_limits<float>::max();
Eigen::Matrix4f best_transform = Eigen::Matrix4f::Identity();
// 添加函数声明
pcl::PointCloud<pcl::PointXYZ>::Ptr loadGlobalMap(const std::string& path);
void publishTransform(const Eigen::Matrix4f& transform);
void handleDegenerateCase(const Eigen::Matrix4f& init_guess);


// 在文件顶部添加全局发布者声明

ros::Publisher filtered_pub;
ros::Publisher fitness_score_pub; // 添加在全局变量声明处
ros::Publisher registered_cloud_pub;
ros::Publisher radar_cloud_pub;  // 新增：雷达点云发布者
ros::Publisher global_map_pub;

// ROS 全局变量
ros::Subscriber point_cloud_sub;
ros::Publisher transformMatrixPublisher;
bool getPathOk = false;
// 在全局变量区域添加一个标志位
bool first_radar_cloud_published = false;


float calculateFitnessScore(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
    const Eigen::Matrix4f& transformation);

// 全局地图点云
pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_cloud(new pcl::PointCloud<pcl::PointXYZ>);

struct Point { float x, y, z; };


/*

//去除噪声点，提高后续配准精度。
pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input) 
{
    // 离群点滤波（保留所有点但去除噪声）
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input);
    sor.setMeanK(4);       // 邻近点数量设为3
    sor.setStddevMulThresh(2.0); // 更宽松的阈值
    sor.filter(*input);

    
    
    return input;
}


pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> configureICP(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target)
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    
    // icp动态参数设置（先验地图作为source，雷达点云作为target）
    float ratio = static_cast<float>(target->size()) / source->size();  // 雷达点数/先验地图点数
    ratio = std::clamp(ratio, 0.5f, 2.0f);
    
    icp.setMaximumIterations(50 + 100*(1 - ratio)); // 点数差异大时增加迭代
    icp.setMaxCorrespondenceDistance(0.5 * ratio);  // 动态匹配距离
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-5);
    
    // 使用SVD分解加速收敛
    //icp.setTransformationEstimation(
       // make_shared<pcl::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>>());
       
    // 修改configureICP函数中的代码
    icp.setTransformationEstimation(
    boost::make_shared<pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>>());
    
    return icp;
}
*/



//质心平移对齐
Eigen::Matrix4f geometricInitialAlignment(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target)
{
    if (source->empty() || target->empty()) {
        ROS_ERROR("Empty input cloud in geometric alignment!");
        return Eigen::Matrix4f::Identity();
    }
    // 如果源点云点数过少（例如少于3个点），使用固定点作为初始位姿
    if (target->size() < 10) {
        ROS_WARN("Using fixed point (0.8, 0.1, 0) for initial alignment due to insufficient points");
        Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
        init_guess.block<3,1>(0,3) = Eigen::Vector3f(0.0f, 0.1f, 0.0f);
        return init_guess;
    }//
 /*   
    // 直接计算质心对齐Original accumulated point
    Eigen::Vector4f src_centroid4, tgt_centroid4;
    pcl::compute3DCentroid(*source, src_centroid4);
    pcl::compute3DCentroid(*target, tgt_centroid4);
    
    Eigen::Vector3f src_centroid = src_centroid4.head<3>();
    Eigen::Vector3f tgt_centroid = tgt_centroid4.head<3>();

*/
/*
    // 构建变换矩阵（只有平移，没有旋转）
    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    init_guess.block<3,1>(0,3) = tgt_centroid - src_centroid;
    
    ROS_INFO("Centroid alignment: [%.2f, %.2f, %.2f]", 
            init_guess(0,3), init_guess(1,3), init_guess(2,3));
  
  */
  
  // 使用简化的ICP进行初始对齐
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    
    // 交换源和目标：将雷达点云对齐到先验地图
    icp.setInputSource(target);  // 雷达点云作为源
    icp.setInputTarget(source);  // 先验地图作为目标
    
    // 设置ICP参数
    icp.setMaxCorrespondenceDistance(2.0);  // 较大的匹配距离
    icp.setMaximumIterations(80);           // 较少的迭代次数
    icp.setTransformationEpsilon(1e-6);
    
    // 执行ICP
    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned);
    
    if (icp.hasConverged()) {
        ROS_INFO("ICP initial alignment converged with fitness: %.3f", icp.getFitnessScore());
        
        // 获取将雷达点云变换到先验地图的变换矩阵
        Eigen::Matrix4f radar_to_map = icp.getFinalTransformation();
        
        // 我们需要的是将先验地图变换到雷达点云的变换
        // 对于刚体变换，逆变换可以通过分解为旋转和平移来计算
        
        // 分解变换矩阵为旋转和平移
        Eigen::Matrix3f R = radar_to_map.block<3,3>(0,0);
        Eigen::Vector3f t = radar_to_map.block<3,1>(0,3);
        
        // 计算逆变换：R_inv = R^T, t_inv = -R^T * t
        Eigen::Matrix3f R_inv = R.transpose();
        Eigen::Vector3f t_inv = -R_inv * t;
        
        // 构建逆变换矩阵
        Eigen::Matrix4f map_to_radar = Eigen::Matrix4f::Identity();
        map_to_radar.block<3,3>(0,0) = R_inv;
        map_to_radar.block<3,1>(0,3) = t_inv;
        
        ROS_INFO("Initial alignment - Translation: [%.2f, %.2f, %.2f]", 
                t_inv[0], t_inv[1], t_inv[2]);
        
        return map_to_radar;
    } else {
        ROS_WARN("ICP initial alignment did not converge, using identity matrix");
        return Eigen::Matrix4f::Identity();
    }
    
  
  //  return init_guess;
}




// 实现精度计算函数（添加在performICP函数之后）
float calculateFitnessScore(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
    const Eigen::Matrix4f& transformation) {
    
    if (source->empty() || target->empty()) {
        ROS_WARN("Cannot calculate fitness score: empty clouds");
        return -1.0f;
    }

    // 变换先验地图（source）来匹配雷达点云（target）
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*source, *transformed_source, transformation);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(target);

    float total_squared_distance = 0.0f;
    int valid_points = 0;
    float max_search_distance = 2.0f; // 最大搜索距离，避免匹配到太远的点
    
    for (const auto& point : transformed_source->points) {
        std::vector<int> indices(1);
        std::vector<float> distances(1);
        if (kdtree.nearestKSearch(point, 1, indices, distances) == 1) {
            // 只计算在合理距离内的对应点
            if (distances[0] <= max_search_distance * max_search_distance) {
                total_squared_distance += distances[0]; // distances[0]已经是平方距离
                valid_points++;
            }
        }
    }

    if (valid_points == 0) return -1.0f;
    
    // 计算RMSE：根均方误差
    float rmse = std::sqrt(total_squared_distance / valid_points);
    
    ROS_INFO("Fitness calculation: %d valid correspondences, total squared distance: %.3f, RMSE: %.3f", 
             valid_points, total_squared_distance, rmse);
    
    return rmse;
}






Eigen::Matrix4f performCoarseICP(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
    const Eigen::Matrix4f& init_guess) {
    
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // 根据点云密度调整参数
    float avg_density = std::min(source_cloud->size(), target_cloud->size()) / 100.0f;
    float max_dist = std::max(4.0f, std::min(2.0f, avg_density * 0.1f));//往大里调
    
    icp.setMaxCorrespondenceDistance(max_dist);  // 动态匹配距离
    icp.setMaximumIterations(80);                // 减少迭代次数
    icp.setTransformationEpsilon(1e-4);
    icp.setEuclideanFitnessEpsilon(1e-3);
    
    // 先验地图作为source，雷达点云作为target
    icp.setInputSource(source_cloud);  // 先验地图
    icp.setInputTarget(target_cloud);  // 雷达点云
    
    pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
    icp.align(aligned_cloud, init_guess);
    
    return icp.getFinalTransformation();
}

Eigen::Matrix4f performFineICP(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
    const Eigen::Matrix4f& coarse_transform) {
    
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // 对于锥桶点云，使用更严格的参数
    icp.setMaxCorrespondenceDistance(0.5);  // 更小的匹配距离，适合锥桶精度，往小里调
    icp.setMaximumIterations(150);          // 适中的迭代次数
    icp.setTransformationEpsilon(1e-8);     // 更严格的收敛条件
    icp.setEuclideanFitnessEpsilon(1e-8);   // 更严格的收敛条件
    
    // 应用粗配准结果到先验地图
    pcl::PointCloud<pcl::PointXYZ>::Ptr coarse_aligned(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*source_cloud, *coarse_aligned, coarse_transform);
    
    // 先验地图作为source，雷达点云作为target
    icp.setInputSource(coarse_aligned);  // 变换后的先验地图
    icp.setInputTarget(target_cloud);    // 雷达点云
    
    pcl::PointCloud<pcl::PointXYZ> final_aligned;
    icp.align(final_aligned);
    
    return coarse_transform * icp.getFinalTransformation(); // 组合变换矩阵
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    static pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    static int frame_count = 0;
    static Eigen::Matrix4f last_valid_transform = Eigen::Matrix4f::Identity();
    const int MAX_FRAMES = 5;  // 增加累积帧数
    if (frame_count >= MAX_FRAMES) {
    accumulated_cloud->clear(); // 清空后重新累积
    frame_count = 0;
    }

    // 1. 转换为PCL格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // 2. 预处理（离群点去除）
    /*pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(4);         //稀疏就往小里调
    sor.setStddevMulThresh(5);   //
    sor.filter(*cloud);*/

   

    // 4. 累积过滤后的点云（使用filtered_cloud）
    *accumulated_cloud += *cloud;
    frame_count++;

   

    // 5. 累积足够帧数再处理
    if (frame_count < MAX_FRAMES) {
        ROS_WARN("Accumulating frames: %d/%d", frame_count, MAX_FRAMES);
        return;
    }

    // 6. 对累积点云进行降采样和去重处理
    /*pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // 使用体素网格滤波进行降采样
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(accumulated_cloud);
    voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f); // 10cm的体素大小
    voxel_filter.filter(*downsampled_cloud);
    
    // 再次进行离群点去除
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
    sor2.setInputCloud(downsampled_cloud);
    sor2.setMeanK(10);
    sor2.setStddevMulThresh(1.0);
    sor2.filter(*downsampled_cloud);
    
    ROS_INFO("Original accumulated points: %zu, After downsampling: %zu", 
             accumulated_cloud->size(), downsampled_cloud->size());
    
    // 使用降采样后的点云进行后续处理
    accumulated_cloud = downsampled_cloud;*/

    // 6. RANSAC拟合直线并生成初始位姿（使用ransac_cloud）
   // Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    // 5. 直接使用几何质心对齐获取初始位姿
    Eigen::Matrix4f init_guess = geometricInitialAlignment(global_map_cloud, accumulated_cloud);

    // 7. 执行ICP配准（传入初始猜测和完整累积点云）
    //Eigen::Matrix4f transformation = performICP(accumulated_cloud, global_map_cloud, init_guess);
    // 7.1 粗配准（先验地图作为source，雷达点云作为target）
    //Eigen::Matrix4f coarse_transform = performCoarseICP(global_map_cloud, accumulated_cloud, init_guess);

    // 7.2 精配准（先验地图作为source，雷达点云作为target）
    //Eigen::Matrix4f fine_transform = performFineICP(global_map_cloud, accumulated_cloud, coarse_transform);
    Eigen::Matrix4f fine_transform = performFineICP(global_map_cloud, accumulated_cloud, init_guess);
    // 7.3 计算最终精度（先验地图作为source，雷达点云作为target）
    float rmse = calculateFitnessScore(global_map_cloud, accumulated_cloud, fine_transform);
    
    // 8. 异常处理
    if (fine_transform == Eigen::Matrix4f::Identity()) {
        ROS_WARN("ICP failed, using last valid transform");
        fine_transform = last_valid_transform;
    } else {
        last_valid_transform = fine_transform;
    }

    // 9. 计算并发布精度
    ROS_INFO("Point cloud info - Global map: %zu points, Radar cloud: %zu points", 
             global_map_cloud->size(), accumulated_cloud->size());
    ROS_INFO("Registration RMSE: %.3f meters", rmse);
    
    // 添加ICP收敛状态检查
    static pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_check;
    icp_check.setInputSource(global_map_cloud);
    icp_check.setInputTarget(accumulated_cloud);
    pcl::PointCloud<pcl::PointXYZ> aligned_check;
    icp_check.align(aligned_check, fine_transform);
    
    if (icp_check.hasConverged()) {
        ROS_INFO("ICP converged with fitness score: %.6f", icp_check.getFitnessScore());
    } else {
        ROS_WARN("ICP did not converge!");
    }
    std_msgs::Float64MultiArray score_msg;
    score_msg.data.clear();
    score_msg.data.push_back(rmse);        // 第一个元素：当前RMSE
    score_msg.data.push_back(best_rmse);   // 第二个元素：最佳RMSE
    fitness_score_pub.publish(score_msg);
    
    // 更新最佳变换矩阵的条件：RMSE有效且更优
if (rmse >= 0 && rmse < best_rmse) {
    best_rmse = rmse;
    best_transform = fine_transform;
    ROS_INFO("New best RMSE: %.3f, updating transform.", best_rmse);
}
// 决定使用哪个变换矩阵进行可视化
Eigen::Matrix4f transform_to_use = fine_transform;
if (rmse >= 0.7f && best_rmse < std::numeric_limits<float>::max()) {
    ROS_WARN("RMSE too high (%.3f >= 0.7), using best transform with RMSE: %.3f", 
             rmse, best_rmse);
    transform_to_use = best_transform;
}
    // 10. 转换点云并添加颜色
    // 现在变换先验地图来对齐到雷达点云位置
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_map(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*global_map_cloud, *transformed_map, transform_to_use); // 变换先验地图

    // 11. 分别发布雷达点云和配准后的先验地图
    // 11.1 发布原始雷达点云（蓝色）
    if (!first_radar_cloud_published && !accumulated_cloud->empty()) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr radar_cloud_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& pt : accumulated_cloud->points) {
        pcl::PointXYZRGB colored_pt;
        colored_pt.x = pt.x;
        colored_pt.y = pt.y;
        colored_pt.z = pt.z;
        colored_pt.r = 0;   // 蓝色（雷达点云）
        colored_pt.g = 0;
        colored_pt.b = 255;
        radar_cloud_colored->push_back(colored_pt);
    }
    
    sensor_msgs::PointCloud2 radar_msg;
    pcl::toROSMsg(*radar_cloud_colored, radar_msg);
    radar_msg.header = cloud_msg->header;
    radar_msg.header.frame_id = "world";
    radar_cloud_pub.publish(radar_msg);
    
    first_radar_cloud_published = true;
    ROS_INFO("Published first radar cloud with %zu points", accumulated_cloud->size());
}
    /*
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr radar_cloud_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& pt : accumulated_cloud->points) {
        pcl::PointXYZRGB colored_pt;
        colored_pt.x = pt.x;
        colored_pt.y = pt.y;
        colored_pt.z = pt.z;
        colored_pt.r = 0;   // 蓝色（雷达点云）
        colored_pt.g = 0;
        colored_pt.b = 255;
        radar_cloud_colored->push_back(colored_pt);
    }
    
    sensor_msgs::PointCloud2 radar_msg;
    pcl::toROSMsg(*radar_cloud_colored, radar_msg);
    radar_msg.header = cloud_msg->header;
    radar_msg.header.frame_id = "camera_init";
    radar_cloud_pub.publish(radar_msg);
   */ 
    // 11.2 发布变换后的先验地图（红色）
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr registered_map_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& pt : transformed_map->points) {
        pcl::PointXYZRGB colored_pt;
        colored_pt.x = pt.x;
        colored_pt.y = pt.y;
        colored_pt.z = pt.z;
        colored_pt.r = 255; // 红色（变换后的先验地图）
        colored_pt.g = 0;
        colored_pt.b = 0;
        registered_map_colored->push_back(colored_pt);
    }
    
    sensor_msgs::PointCloud2 registered_msg;
    pcl::toROSMsg(*registered_map_colored, registered_msg);
    registered_msg.header = cloud_msg->header;
    registered_msg.header.frame_id = "world";
    registered_cloud_pub.publish(registered_msg);

    // 12. 发布变换矩阵
    // 检查变换矩阵是否有效（不是单位矩阵且RMSE在合理范围内）
    bool is_valid_transform = (best_transform != Eigen::Matrix4f::Identity()) && 
                         (rmse >= 0) && (rmse < 1.2f); // RMSE小于2米才认为有效
if (is_valid_transform) {
    std_msgs::Float64MultiArray trans_matrix;
    for (int i = 0; i < best_transform.rows(); i++) {
        for (int j = 0; j < best_transform.cols(); j++) {
            trans_matrix.data.push_back(best_transform(i, j));
        }
    }
    transformMatrixPublisher.publish(trans_matrix);
    ROS_INFO("Publishing valid transform matrix with RMSE: %.3f", best_rmse);
} else {
    ROS_WARN("Not publishing transform matrix - invalid registration (RMSE: %.3f)", rmse);
    
    // 发布一个空的矩阵表示无效
    std_msgs::Float64MultiArray empty_matrix;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            empty_matrix.data.push_back(0.0);
        }
    }
    transformMatrixPublisher.publish(empty_matrix);
}
    
   
    // 13. 重置累积
    accumulated_cloud->clear();
    frame_count = 0;

    // 14. 调试：发布中间点云（可选）
    //static ros::Publisher ransac_pub = nh.advertise<sensor_msgs::PointCloud2>("/debug/ransac_cloud", 1);
    //static ros::Publisher filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/debug/filtered_cloud", 1);
    
    sensor_msgs::PointCloud2 ransac_debug_msg, filtered_debug_msg;
    //pcl::toROSMsg(*ransac_cloud, ransac_debug_msg);
   // pcl::toROSMsg(*filtered_cloud, filtered_debug_msg);
    ransac_debug_msg.header = cloud_msg->header;
    filtered_debug_msg.header = cloud_msg->header;
    ransac_debug_msg.header.frame_id = "world";
    filtered_debug_msg.header.frame_id = "world";
   // ransac_pub.publish(ransac_debug_msg);
   // filtered_pub.publish(filtered_debug_msg);
}
//无用代码
/*
visualization_msgs::Marker lines;
lines.type = visualization_msgs::Marker::LINE_LIST;
for(size_t i=0; i<correspondences.size(); ++i) {
    geometry_msgs::Point src_pt, tgt_pt;
    // 填充对应点坐标
    lines.points.push_back(src_pt);
    lines.points.push_back(tgt_pt);
}
*/
// 加载全局地图的实现
pcl::PointCloud<pcl::PointXYZ>::Ptr loadGlobalMap(const std::string& path) {
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1) {
        ROS_ERROR("Failed to load global map from %s", path.c_str());
    }
    return cloud;
}

// 发布变换矩阵的实现
void publishTransform(const Eigen::Matrix4f& transform) {
    std_msgs::Float64MultiArray msg;
    for (int i = 0; i < transform.rows(); i++) {
        for (int j = 0; j < transform.cols(); j++) {
            msg.data.push_back(transform(i, j));
        }
    }
    transformMatrixPublisher.publish(msg);
}

// 处理退化情况的空实现
void handleDegenerateCase(const Eigen::Matrix4f& init_guess) {
    ROS_WARN("Degenerate case detected. Using fallback strategy.");
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "skid_detector");
    ros::NodeHandle nh;

    // ================= 初始化所有 Publisher =================
    transformMatrixPublisher = nh.advertise<std_msgs::Float64MultiArray>("/transform_matrix", 1);
    registered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/registered_cloud", 1);
    radar_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/radar_cloud", 1);  // 新增
    global_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/global_map_cloud", 1);
    //fitness_score_pub = nh.advertise<std_msgs::Float32>("/registration_rmse", 1);
    fitness_score_pub = nh.advertise<std_msgs::Float64MultiArray>("/registration_scores", 1);
    
   


    // ================= 加载全局地图点云 =================
    const std::string global_map_path = "/home/test/skidpad/skidpad_ws/src/skidpad/src/config/point.pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(global_map_path, *global_map_cloud) == -1){
        ROS_ERROR_STREAM("Failed to load global map PCD file at: " << global_map_path);
        ROS_ERROR("Possible causes: Incorrect path, file permissions, or corrupted PCD.");
        return -1;
    }
    
    // ********** 在此处添加坐标偏移补偿 **********
    float initial_offset_x = 0.0;  // 根据实际测量调整
    float initial_offset_y = 0.0;
    for (auto& pt : global_map_cloud->points) {
    pt.x += initial_offset_x;
    pt.y += initial_offset_y;
    }

    ROS_INFO("Successfully loaded %zu points from global map", global_map_cloud->size());
   
    // ================= 转换全局地图为带颜色的点云 (红色) =================
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& pt : global_map_cloud->points) {
        pcl::PointXYZRGB colored_pt;
        colored_pt.x = pt.x;
        colored_pt.y = pt.y;
        colored_pt.z = pt.z;
        colored_pt.r = 255; // 红色通道
        colored_pt.g = 0;
        colored_pt.b = 0;
        global_map_colored->push_back(colored_pt);
    }

    // ================= 初始化全局地图消息 =================
    sensor_msgs::PointCloud2 global_map_msg;
    pcl::toROSMsg(*global_map_colored, global_map_msg);
    //global_map_msg.header.frame_id = "aloam_init_on_base"; // 确保与雷达坐标系一致
    global_map_msg.header.frame_id = "world";
    // ================= 设置订阅者 (必须在循环前注册!) =================
    point_cloud_sub = nh.subscribe("/mapped/aft_lidar_cluster", 10, pointCloudCallback);
    //point_cloud_sub = nh.subscribe("cluster_cloud", 10, pointCloudCallback);
    
    // 重置最佳RMSE和变换矩阵
    best_rmse = std::numeric_limits<float>::max();
    best_transform = Eigen::Matrix4f::Identity();
    
    // ================= 主循环控制 =================
    ros::Rate loop_rate(1); // 控制全局地图发布频率为1Hz
    while (ros::ok()) {
        // 注释掉持续发布原始先验地图，避免重复显示
        // global_map_msg.header.stamp = ros::Time::now();
        // global_map_pub.publish(global_map_msg);

        // 处理回调队列
        ros::spinOnce();

        // 控制循环频率
        loop_rate.sleep();
    }

    return 0;
}



//1.将点云分开，记为左右点云------>点数太少，所以合并
//2.根据RANSAC将左右点云拟合出直线
//3.计算出左右直线交点对应的坐标，计算出中点坐标
//4.将点云先NDT粗配准，再ICP细配准----------NDT为大量点云的粗配准，不适合锥桶质心点云，舍弃
//5.列出变换矩阵，存在容器中
