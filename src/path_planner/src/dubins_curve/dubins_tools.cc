#include "dubins_curve/dubins_tools.hh"
#include <cmath>
#include "ros/console.h"
namespace dubins_curve {

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
bool DubinsTool::CoordTransform(const point_type& input_pos, const double& theta, const double& radius1,
                                const double& radius2, const std::string& dir, point_type& output_pos) {
    if (dir == "left") {
        // 这里把求圆心和求切点的统一在一块了
        output_pos.first.first = input_pos.first.first + radius1 * std::cos(theta) - radius2 * sin(theta);
        output_pos.first.second = input_pos.first.second + radius1 * std::sin(theta) + radius2 * cos(theta);
    } else if (dir == "right") {
        output_pos.first.first = input_pos.first.first + radius1 * std::cos(theta) + radius2 * sin(theta);
        output_pos.first.second = input_pos.first.second + radius1 * std::sin(theta) - radius2 * cos(theta);
    } else {
        ROS_ERROR("wrong dir:%s", dir.c_str());
        return false;
    }
    return true;
}

double DubinsTool::GetDistance(const point_type& pos1, const point_type& pos2) {
    return sqrt(pow(pos1.first.first - pos2.first.first, 2) + pow(pos1.first.second - pos2.first.second, 2));
}

std::vector<double> DubinsTool::GetDiffVec(const point_type& pos1, const point_type& pos2) {
    std::vector<double> diff_vec;
    diff_vec.push_back(pos1.first.first - pos2.first.first);
    diff_vec.push_back(pos1.first.second - pos2.first.second);

    return diff_vec;
}
//  [-pi~pi]画出向量可能得情况，计算两个向量的夹角
double DubinsTool::GetDelta(const std::vector<double>& diff_vec1, const std::vector<double>& diff_vec2,
                            const std::string& dir) {
    double theta1 = atan2(diff_vec1[1], diff_vec1[0]);
    double theta2 = atan2(diff_vec2[1], diff_vec2[0]);

    double delta = theta2 - theta1;

    if (dir == "left" && theta1 > theta2) {
        delta = theta2 + 2 * M_PI - theta1;
    } else if (dir == "right" && theta1 < theta2) {
        delta = theta1 + 2 * M_PI - theta2;
    }

    return delta;
}
}  // namespace dubins_curve