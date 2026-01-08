#ifndef CATMULL_ROM_H
#define CATMULL_ROM_H

#include <vector>
#include <Eigen/Dense>
#include <cmath>

class CatmullRom {
public:
    std::vector<std::vector<double>> calc_path_with_xy_yaw_curv(const std::vector<double>& x, 
                                                               const std::vector<double>& y, 
                                                               double ds = 0.1);
    
    std::vector<std::vector<double>> calc_path_with_xy_yaw_curv(double ds = 0.1) {
        return calc_path_with_xy_yaw_curv(x_, y_, ds);
    }
    
    void set_control_points(const std::vector<double>& x, const std::vector<double>& y) {
        x_ = x;
        y_ = y;
    }

private:
    Eigen::Vector2d interpolate(const Eigen::Vector2d& p0, 
                               const Eigen::Vector2d& p1, 
                               const Eigen::Vector2d& p2, 
                               const Eigen::Vector2d& p3, 
                               double t);
    
    double distance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);
    double calc_heading(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);
    double calc_curvature(const Eigen::Vector2d& p0, 
                         const Eigen::Vector2d& p1, 
                         const Eigen::Vector2d& p2);

    std::vector<double> x_;
    std::vector<double> y_;
};

#endif // CATMULL_ROM_H