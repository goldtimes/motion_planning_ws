#include <Eigen/Dense>
#include <cassert>
#include <climits>
#include <iostream>
#include <limits>

#include "nav_common/geometry/dubins_curve.hh"
#include "nav_common/geometry/point.hh"
#include "nav_common/math/math_utils.hh"

namespace mp::common::geometry {
namespace {
#define UNPACK_DUBINS_INPUT(alpha, beta) \
    double sin_a = sin(alpha);           \
    double sin_b = sin(beta);            \
    double cos_a = cos(alpha);           \
    double cos_b = cos(beta);            \
    double cos_a_b = cos(alpha - beta);
}  // namespace
enum {
    DUBINS_NONE = -1,  // 无解，计算的dubins距离为负
    DUBINS_L = 0,
    DUBINS_S = 1,
    DUBINS_R = 2,
};

DubinsCurve::DubinsCurve(double step, double max_curv) : Curve(step), max_curv_(max_curv) {
}
DubinsCurve::DubinsCurve() : Curve(0.1), max_curv_(0.25) {
}

DubinsCurve::~DubinsCurve() {
}

void DubinsCurve::setMaxCurv(double max_curv) {
    assert(max_curv > 0);
    max_curv_ = max_curv;
}

// HybridA*中调用的接口
bool DubinsCurve::run(const Points2d points, Points2d& path) {
    // 传入的points必须有起始点和目标点
    if (points.size() < 2) {
        return false;
    }
    Points3d poses;
    poses.emplace_back(points.begin()->x(), points.begin()->y());
    // 有多个点的时候
    for (size_t i = 1; i < points.size() - 1; ++i) {
        double theta1 = std::atan2(points[i].y() - points[i - 1].y(), points[i].x() - points[i - 1].x());
        double theta2 = std::atan2(points[i + 1].y() - points[i].y(), points[i + 1].x() - points[i].x());
        poses.emplace_back(points[i].x(), points[i].y(), (theta1 + theta2) / 2);
    }
    poses.emplace_back(points.back().x(), points.back().y(), 0);
    return run(poses, path);
}

bool DubinsCurve::run(const Points3d points, Points2d& path) {
    if (points.size() < 2) {
        return false;
    }
    path.clear();
    for (size_t i = 0; i < points.size() - 1; ++i) {
        // 生成dubins曲线
        Points2d path_find = generation(points[i], points[i + 1]);
        if (!path_find.empty()) {
            path.insert(path.end(), path_find.begin(), path_find.end());
        }
    }
    return !path.empty();
}

/**
 * @brief Generate the path. 需要做几何图来推导公式
 * @param start Initial pose (x, y, yaw)
 * @param goal  Target pose (x, y, yaw)
 * @return path The smoothed trajectory points
 */
Points2d DubinsCurve::generation(Point3d start, Point3d goal) {
    Points2d path;
    double sx = start.x(), sy = start.y(), syaw = start.theta();
    double gx = goal.x(), gy = goal.y(), gyaw = goal.theta();
    //  坐标变换,以起点为0点
    gx -= sx;
    gy -= sy;

    double theta = mp::common::math::mod2pi(atan2(gy, gx));
    double dist = std::hypot(gx, gy) * max_curv_;
    double alpha = mp::common::math::mod2pi(syaw - theta);
    double beta = mp::common::math::mod2pi(gyaw - theta);

    //  找到曲线的路径最短
    DubinsMode best_mode;
    DubinsMode mode;

    double best_cost = std::numeric_limits<double>::max();
    DubinsLength length;
    DubinsLength best_length = {DUBINS_NONE, DUBINS_NONE, DUBINS_NONE};
    // 遍历functinal函数求解
    for (const auto& solver : dubins_solvers) {
        solver(alpha, beta, dist, length, mode);
        _update(length, mode, best_length, best_mode, best_cost);
    }
    if (best_cost == std::numeric_limits<double>::max()) {
        return path;
    }
}

// cost的计算是越大越好？
void DubinsCurve::_update(DubinsLength length, DubinsMode mode, DubinsLength& best_length, DubinsMode& best_mode,
                          double& best_cost) {
    if (std::get<0>(length) != DUBINS_NONE) {
        double t, p, q;
        std::tie(t, p, q) = length;
        double cost = fabs(t) + fabs(p) + fabs(q);
        if (best_cost > cost) {
            best_length = length;
            best_mode = mode;
            best_cost = cost;
        }
    }
}

}  // namespace mp::common::geometry