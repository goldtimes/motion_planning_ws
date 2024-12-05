#pragma once

#include <ros/ros.h>
#include <Eigen/Core>
#include <array>
#include <utility>
#include <vector>

namespace dubins_curve {
// dubins的圆心 x,y,theta组成
using point_type = std::pair<std::pair<double, double>, double>;
// 曲线类型LSL,LSR,RSR,RSL，已经路径上的所有点
using curve_type = std::pair<std::array<point_type, 4>, double>;
class DubinsTool {
   public:
    DubinsTool() = default;
    ~DubinsTool() = default;
    /**
     * @brief 用于求解切点，或是左转右转的圆心，利用几何关系
     *
     * @param input_pos 求切点时，输入的是对应圆心；求圆心则输入起点终点坐标
     * @param theta 计算得到的几何角度
     * @param radius1 求切点时的半径
     * @param radius2 求圆心时的半径
     * @param dir 主要用于求圆心时，左转右转圆心有区别
     * @param output_pos 计算出的点
     * @return true
     * @return false
     */
    bool CoordTransform(const point_type& input_pos, const double& theta, const double& radius1, const double& radius2,
                        const std::string& dir, point_type& output_pos);
    double GetDistance(const point_type& pos1, const point_type& pos2);
    std::vector<double> GetDiffVec(const point_type& pos1, const point_type& pos2);
    double GetDelta(const std::vector<double>& diff_vec1, const std::vector<double>& diff_vec2, const std::string& dir);
};
}  // namespace dubins_curve