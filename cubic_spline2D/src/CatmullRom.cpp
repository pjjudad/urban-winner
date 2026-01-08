#include "CatmullRom.h"
#include <vector>
#include <cmath>
#include <algorithm>

using namespace Eigen;
using namespace std;

double CatmullRom::distance(const Vector2d& p1, const Vector2d& p2) {
    return (p2 - p1).norm();
}

double CatmullRom::calc_heading(const Vector2d& p1, const Vector2d& p2) {
    return atan2(p2.y() - p1.y(), p2.x() - p1.x());
}

double CatmullRom::calc_curvature(const Vector2d& p0, 
                                 const Vector2d& p1, 
                                 const Vector2d& p2) {
    Vector2d v1 = p1 - p0;
    Vector2d v2 = p2 - p1;
    
    double dx1 = v1.x(), dy1 = v1.y();
    double dx2 = v2.x(), dy2 = v2.y();
    
    double cross = dx1 * dy2 - dy1 * dx2;
    double dot = dx1 * dx2 + dy1 * dy2;
    
    double ds1 = v1.norm();
    double ds2 = v2.norm();
    
    if (ds1 < 1e-5 || ds2 < 1e-5) return 0.0;
    
    double angle = atan2(cross, dot);
    return angle / ((ds1 + ds2) / 2.0);
}

Vector2d CatmullRom::interpolate(const Vector2d& p0, 
                                const Vector2d& p1, 
                                const Vector2d& p2, 
                                const Vector2d& p3, 
                                double t) {
    double t2 = t * t;
    double t3 = t2 * t;

    Vector2d a = 2 * p1;
    Vector2d b = p2 - p0;
    Vector2d c = 2 * p0 - 5 * p1 + 4 * p2 - p3;
    Vector2d d = -p0 + 3 * p1 - 3 * p2 + p3;

    return 0.5 * (a + b * t + c * t2 + d * t3);
}

vector<vector<double>> CatmullRom::calc_path_with_xy_yaw_curv(const vector<double>& x, 
                                                             const vector<double>& y, 
                                                             double ds) {
    x_ = x;
    y_ = y;
    
    vector<Vector2d> control_points;
    for (size_t i = 0; i < x.size(); i++) {
        control_points.push_back(Vector2d(x[i], y[i]));
    }

    if (control_points.size() < 2) {
        return vector<vector<double>>();
    }
    
    vector<Vector2d> path;
    vector<double> headings;
    vector<double> curvatures;
    
    // 检查是否闭合路径 (严格阈值)
    bool is_closed = (control_points.size() > 3) && 
                    (distance(control_points.front(), control_points.back()) < 0.05);
    
    if (control_points.size() == 2) {
        Vector2d start = control_points[0];
        Vector2d end = control_points[1];
        double dist = distance(start, end);
        int num_segments = max(1, static_cast<int>(dist / ds));
        
        for (int i = 0; i <= num_segments; i++) {
            double t = static_cast<double>(i) / num_segments;
            Vector2d pt = start + t * (end - start);
            path.push_back(pt);
            headings.push_back(calc_heading(start, end));
            curvatures.push_back(0.0);
        }
    } else if (control_points.size() == 3) {
        Vector2d virtual_start = 2 * control_points[0] - control_points[1];
        Vector2d virtual_end = 2 * control_points[2] - control_points[1];
        
        control_points.insert(control_points.begin(), virtual_start);
        control_points.push_back(virtual_end);
    }
    
    // 四点及以上使用完整Catmull-Rom
    if (control_points.size() >= 4) {
        vector<Vector2d> extended_points;
        
        if (is_closed) {
            // 闭环处理：完美闭环控制点序列
            extended_points.push_back(control_points[control_points.size()-2]); // P[n-2]
            extended_points.insert(extended_points.end(), control_points.begin(), control_points.end());
            extended_points.push_back(control_points[1]); // P1
        } else {
            // 开环处理
            extended_points.push_back(2 * control_points[0] - control_points[1]);
            extended_points.insert(extended_points.end(), control_points.begin(), control_points.end());
            extended_points.push_back(2 * control_points.back() - control_points[control_points.size()-2]);
        }
        
        // 分段生成曲线
        for (size_t i = 1; i < extended_points.size() - 2; i++) {
            Vector2d p0 = extended_points[i-1];
            Vector2d p1 = extended_points[i];
            Vector2d p2 = extended_points[i+1];
            Vector2d p3 = extended_points[i+2];
            
            double segment_length = distance(p1, p2);
            int num_segments = max(3, static_cast<int>(segment_length / ds)); // 最小3段
            
            for (int j = 0; j <= num_segments; j++) {
                double t = static_cast<double>(j) / num_segments;
                path.push_back(interpolate(p0, p1, p2, p3, t));
            }
        }
        
        // 计算航向角和曲率
        if (path.size() > 2) {
            // 第一个点
            Vector2d v0 = path[1] - path[0];
            headings.push_back(atan2(v0.y(), v0.x()));
            curvatures.push_back(0.0);
            
            // 中间点
            for (size_t i = 1; i < path.size() - 1; i++) {
                // 使用三点平均计算更准确的方向
                Vector2d v1 = path[i] - path[i-1];
                Vector2d v2 = path[i+1] - path[i];
                Vector2d avg = (v1.normalized() + v2.normalized()).normalized();
                headings.push_back(atan2(avg.y(), avg.x()));
                
                curvatures.push_back(calc_curvature(path[i-1], path[i], path[i+1]));
            }
            
            // 最后一个点
            Vector2d v_end = path[path.size()-1] - path[path.size()-2];
            headings.push_back(atan2(v_end.y(), v_end.x()));
            curvatures.push_back(0.0);
        }
    }
    
    // 提取x,y坐标
    vector<double> path_x, path_y;
    for (const auto& pt : path) {
        path_x.push_back(pt.x());
        path_y.push_back(pt.y());
    }
    
    return {path_x, path_y, headings, curvatures};
}
