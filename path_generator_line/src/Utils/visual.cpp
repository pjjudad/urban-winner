#include "Utils/visual.h"

//轨迹和桩桶可视化
namespace ns_path_generator {
    //看一下实际传入的，都是基于世界地图的
    void visual_trajectory(const Trajectory &traj,
                           visualization_msgs::MarkerArray &visual,
                           const std::string &frame,
                           const std::vector<float> &color,
                           const std_msgs::Header &header, bool is_vel) {
        visual.markers.clear();
        std::vector<float> color_tmp;

        for (size_t i = 0; i < traj.size() - 1; i++) {
            visualization_msgs::Marker tmp;
            tmp.header.frame_id = "world";   //为什么只有这个是可以在局部坐标系base_link下
            tmp.header.stamp = header.stamp;
            tmp.ns = "line_" + std::to_string(i);
            tmp.action = visualization_msgs::Marker::ADD;
            tmp.type = visualization_msgs::Marker::LINE_STRIP;
            tmp.scale.x = 0.4;
            if (is_vel)
                color_map(traj[i].velocity, color_tmp);
            else
                color_tmp = color;
            tmp.color.r = color_tmp[0];
            tmp.color.g = color_tmp[1];
            tmp.color.b = color_tmp[2];
            tmp.color.a = 1.0;
            geometry_msgs::Point p;
            p.x = traj[i].pts.x;
            p.y = traj[i].pts.y;
            tmp.points.push_back(p);
            p.x = traj[i + 1].pts.x;
            p.y = traj[i + 1].pts.y;
            tmp.points.push_back(p);
            visual.markers.push_back(tmp);
        }
    }

    //根据速度改变轨迹颜色，应该是有 红 和 蓝 两种  这个会将锥桶用颜色高亮出来
    void color_map(double vel, std::vector<float> &color) {
    /**
     * Change the color of each point in the trajectory according to the velocity  
     */

        double temp = (vel - param_.initial_velocity) /
                      (param_.desire_vel - param_.initial_velocity);
        if (temp > 1)
            temp = 1;
        if (temp < 0)
            temp = 0;
        color.clear();
        color.resize(3);
        color[0] = temp;
        color[1] = 0;
        color[2] = 1 - temp;
    }

    //下面会对不同颜色的桩桶做处理，记录下它们在二维地图上的坐标，基于世界坐标系的 
    void visual_map(const fsd_common_msgs::Map &map,
                    visualization_msgs::MarkerArray &visual) {
        visual.markers.clear();
        for (size_t i = 0; i < map.cone_red.size(); i++) {
            visualization_msgs::Marker tmp;
            tmp.header.frame_id = "world";
            tmp.header.stamp = ros::Time::now();
            tmp.ns = "red_" + std::to_string(i);
            tmp.action = visualization_msgs::Marker::ADD;
            tmp.type = visualization_msgs::Marker::POINTS;
            tmp.scale.x = 0.8;
            tmp.scale.y = 0.8;
            tmp.scale.z = 0.8;

            tmp.color.r = 1;
            tmp.color.g = 0;
            tmp.color.b = 0;
            tmp.color.a = 1.0;
            geometry_msgs::Point p;
            p.x = map.cone_red[i].position.x;
            p.y = map.cone_red[i].position.y;
            tmp.points.push_back(p);
            visual.markers.push_back(tmp);
        }

        for (size_t i = 0; i < map.cone_blue.size(); i++) {
            visualization_msgs::Marker tmp;
            tmp.header.frame_id = "world";
            tmp.header.stamp = ros::Time::now();
            tmp.ns = "blue_" + std::to_string(i);
            tmp.action = visualization_msgs::Marker::ADD;
            tmp.type = visualization_msgs::Marker::POINTS;
            tmp.scale.x = 0.8;
            tmp.scale.y = 0.8;
            tmp.scale.z = 0.8;

            tmp.color.r = 0;
            tmp.color.g = 0;
            tmp.color.b = 1;
            tmp.color.a = 1.0;
            geometry_msgs::Point p;
            p.x = map.cone_blue[i].position.x;
            p.y = map.cone_blue[i].position.y;
            tmp.points.push_back(p);
            visual.markers.push_back(tmp);
        }
    }
} // namespace ns_mpc
