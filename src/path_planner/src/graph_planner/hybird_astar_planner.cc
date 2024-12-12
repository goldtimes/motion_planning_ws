#include "path_planner/graph_planner/hybird_astar_planner.hh"
#include <cmath>
#include "dubins_curve/dubins_curve.hh"
#include "path_planner/graph_planner/astar_planner.hh"

namespace mp::path_planner {
namespace {
// 转弯的惩罚
constexpr double kPentaltyTurning = 1.05;
constexpr double kPentaltyCod = 1.5;
// 后退
constexpr double kPentaltyReversing = 1.5;
constexpr int kHeadings = 72;
constexpr double kDeltaHeading = (2 * M_PI / kHeadings);
// double R = 1.0;
constexpr double R = 1.3;

// // 14 deg 0.349065 rad
constexpr double alpha = 14 * M_PI / 180;

const std::vector<HybridAStarPathPlanner::Node> motions = {
    {0, 1, 1.0},          {1, 0, 1.0},           {0, -1, 1.0},          {-1, 0, 1.0},
    {1, 1, std::sqrt(2)}, {1, -1, std::sqrt(2)}, {-1, 1, std::sqrt(2)}, {-1, -1, std::sqrt(2)},
};
}  // namespace

HybridAStarPathPlanner::HybridNode::HybridNode(double x, double y, double t, double g, double h, int id, int pid,
                                               int prim)
    : Node(x, y, g, h, id, pid), theta_(t), prim_(prim) {
}

double HybridAStarPathPlanner::HybridNode::theta() const {
    return theta_;
}

void HybridAStarPathPlanner::HybridNode::set_theta(double theta) {
    theta_ = theta;
}

// 计算允许的运动方向
std::vector<HybridAStarPathPlanner::HybridNode> HybridAStarPathPlanner::HybridNode::getMotion() {
    // 以R为半径 alpha为弧度
    double dx[] = {alpha * R, R * sin(alpha), R * sin(alpha)};
    double dy[] = {0, -R * (1 - cos(alpha)), R * (1 - cos(alpha))};
    double dt[] = {0, alpha, -alpha};
    // 6个运动方向 0，1，2前向运动方向
    return {
        HybridNode(dx[0], dy[0], dt[0], 0, 0, 0, 0, 0),   HybridNode(dx[1], dy[1], dt[1], 0, 0, 0, 0, 1),
        HybridNode(dx[2], dy[2], dt[2], 0, 0, 0, 0, 2),   HybridNode(-dx[0], dy[0], -dt[0], 0, 0, 0, 0, 3),
        HybridNode(-dx[1], dy[1], -dt[1], 0, 0, 0, 0, 4), HybridNode(-dx[2], dy[2], -dt[2], 0, 0, 0, 0, 5),
    };
}

/**
 * @brief Overloading operator + for Node class
 * @param n another Node
 * @return Node with current node's and input node n's values added
 */
HybridAStarPathPlanner::HybridNode HybridAStarPathPlanner::HybridNode::operator+(const HybridNode& n) const {
    HybridNode result;

    result.x_ = x_ + n.x_ * cos(theta_) - n.y_ * sin(theta_);
    result.y_ = y_ + n.x_ * sin(theta_) + n.y_ * cos(theta_);
    // result.theta_ = mp::path_planner::math::mod2pi(theta_ + n.theta_);
    result.prim_ = n.prim_;
    // forward driving
    if (prim_ < 3) {
        // 需要转弯
        if (n.prim_ != prim_) {
            // 下一个节点的运动方向相反了
            if (n.prim_ > 2) {
                result.set_g(g() + n.x_ * kPentaltyTurning * kPentaltyCod);
            } else {
                result.set_g(g() + n.x_ + kPentaltyTurning);
            }
        } else {
            // 不需要转弯
            result.set_g(g() + n.x_);
        }
    } else {
        // backward driving
        if (n.prim_ != prim_) {
            // penalize change of direction
            if (n.prim_ < 3)
                result.set_g(g() + n.x_ * kPentaltyTurning * kPentaltyCod * kPentaltyTurning);
            else
                result.set_g(g() + n.x_ * kPentaltyTurning * kPentaltyReversing);
        } else
            result.set_g(g() + n.x_ * kPentaltyReversing);
    }
    return result;
}

/**
 * @brief Overloading operator == for Node class
 * @param n another Node
 * @return true if current node equals node n, else false
 */
bool HybridAStarPathPlanner::HybridNode::operator==(const HybridNode& n) const {
    // 方向角方面我们认为<= 72度都是相等的
    return (x_ == n.x_) && (y_ == n.y_) &&
           ((std::abs(theta_ - n.theta_) <= kDeltaHeading) ||
            (std::abs(theta_ - n.theta_) >= (2 * M_PI - kDeltaHeading)));
}

/**
 * @brief Overloading operator != for Node class
 * @param n another Node
 * @return true if current node equals node n, else false
 */
bool HybridAStarPathPlanner::HybridNode::operator!=(const HybridNode& n) const {
    return !operator==(n);
}

/**
 * @brief Construct a new Hybrid A* object
 * @param costmap   the environment for path planning
 * @param is_reverse whether reverse operation is allowed
 * @param max_curv   maximum curvature of model
 */
HybridAStarPathPlanner::HybridAStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, bool is_reverse, double max_curv)
    : PathPlanner(costmap_ros),
      is_reverse_(is_reverse),
      max_curv_(max_curv),
      //   dubins_gen_(std::make_unique<dubins_curve::DubinsCurve>(1.5, max_curv)),
      a_star_planner_(std::make_unique<AstarGlobalPlanner>(costmap_ros)),
      goal_(HybridNode()) {
}

}  // namespace mp::path_planner