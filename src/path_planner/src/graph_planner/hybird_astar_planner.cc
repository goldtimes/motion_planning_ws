#include "path_planner/graph_planner/hybird_astar_planner.hh"
#include <cmath>
#include <vector>
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
      dubins_gen_(std::make_unique<mp::common::geometry::DubinsCurve>(1.5, max_curv)),
      a_star_planner_(std::make_unique<AstarGlobalPlanner>(costmap_ros)),
      goal_(HybridNode()) {
}

bool HybridAStarPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand) {
    path.clear();
    expand.clear();
    HybridNode start_node(start.x(), start.y(), start.theta());
    HybridNode goal_node(goal.x(), goal.y(), goal.theta());
    updateIndex(start_node);
    updateIndex(goal_node);
    // update heuristic map
    // 混合A*的算法，会利用A*的启发函数方式，计算待障碍物但是没有运行学约束的由目标点向周围拓展的代价图
    if (goal_ != goal_node) {
        goal_.set_x(goal.x());
        goal_.set_y(goal.y());
        goal_.set_theta(goal.theta());
        double gx, gy;
        world2Map(goal.x(), goal.y(), gx, gy);
        HybridNode h_start(gx, gy, 0, 0, 0, grid2Index(gx, gy), 0);
        genHeurisiticMap(h_start);
    }
    // 是否允许方向运动
    int dir = is_reverse_ ? 6 : 3;
    // 运动
    const std::vector<HybridNode> motions = HybridNode::getMotion();
    // open_list和closed_list
    std::priority_queue<HybridNode, std::vector<HybridNode>, HybridNode::compare_cost> open_list;
    std::unordered_map<int, HybridNode> closed_list;

    open_list.push(start_node);
    while (!open_list.empty()) {
        HybridNode current = open_list.top();
        open_list.pop();

        if (closed_list.find(current.id()) != closed_list.end()) {
            continue;
        }

        closed_list.insert(std::make_pair(current.id(), current));
        expand.emplace_back(current.x(), current.y());
        // dubins曲线 goal-shot
        // hybrid算法的Analytic Expansions的思想,在搜索的过程，斜边长 < 50m
        // 才使用dubins曲线来加快搜索，如果dubins曲线命中了目标则提前退出了
        std::vector<HybridNode> path_dubins;
        if (std::hypot(current.x() - goal.x(), current.y() - goal.y()) < 50) {
            // 命中了目标
            if (dubinsShot(current, goal_node, path_dubins)) {
                // 反向遍历得到路径
                const auto& backtrace = _convertClosedListToPath(closed_list, start_node, current);
                for (auto iter = backtrace.rbegin(); iter != backtrace.rend(); ++iter) {
                    path.emplace_back(iter->x(), iter->y());
                }
                for (const auto& dubins_pt : path_dubins) {
                    path.emplace_back(dubins_pt.x(), dubins_pt.y());
                }
                return true;
            }
        }
        // 探索其他node
        for (size_t i = 0; i < dir; ++i) {
            HybridNode node_new = current + motions[i];
            updateIndex(node_new);
            if (closed_list.find(node_new.id()) != closed_list.end()) {
                continue;
            }
            // next node hit the boundary or obstacle
            // prevent planning failed when the current within inflation
            if ((_worldToIndex(node_new.x(), node_new.y()) < 0) ||
                (_worldToIndex(node_new.x(), node_new.y()) >= map_size_) ||
                (node_new.theta() / kDeltaHeading >= kHeadings) ||
                (costmap_->getCharMap()[_worldToIndex(node_new.x(), node_new.y())] >=
                     costmap_2d::LETHAL_OBSTACLE * factor_ &&
                 costmap_->getCharMap()[_worldToIndex(node_new.x(), node_new.y())] >=
                     costmap_->getCharMap()[_worldToIndex(current.x(), current.y())]))
                continue;
            // 不是障碍物，更新h代价值
            node_new.set_pid(current.id());
            updateHeuristic(node_new);
            open_list.push(node_new);
        }
    }
    // 如果bubins曲线没有命中，则调用astar规划路径
    return a_star_planner_->plan(start, goal, path, expand);
}

// 调用曲线判断命中目标
bool HybridAStarPathPlanner::dubinsShot(const HybridNode& start, const HybridNode& goal,
                                        std::vector<HybridNode>& path) {
    double sx, sy, gx, gy;
    world2Map(start.x(), start.y(), sx, sy);
    world2Map(goal.x(), goal.y(), gx, gy);
    mp::common::geometry::Points3d poses = {{sx, sy, start.theta()}, {gx, gy, goal.theta()}};
    mp::common::geometry::Points2d path_dubins;
    // 正确生成了曲线
    if (dubins_gen_->run(poses, path_dubins)) {
        path.clear();
        // 遍历所有的曲线点,如果遇见了障碍物，则曲线是失败的
        for (auto const& p : path_dubins) {
            if (costmap_->getCharMap()[grid2Index(p.x(), p.y())] >= costmap_2d::LETHAL_OBSTACLE * factor_)
                return false;
            else
                path.emplace_back(p.x(), p.y());
        }
        // 没有碰到障碍物
        return true;
    } else
        return false;
}

// 更新node的h值
void HybridAStarPathPlanner::updateHeuristic(HybridNode& node) {
    double cost_dubins = 0.0;
    double cost_2d = h_map_[_worldToIndex(node.x(), node.y())].g() * costmap_->getResolution();
    node.set_h(std::max(cost_2d, cost_dubins));
}

// 同A*的启发函数一样
// 从目标点出发，往前生成代价图
void HybridAStarPathPlanner::genHeurisiticMap(const Node& start) {
    // open list and closed list
    std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
    std::unordered_map<int, Node> open_set;
    open_list.push(start);
    open_set.emplace(start.id(), start);
    // 遍历open_list
    while (!open_list.empty()) {
        auto current = open_list.top();
        open_list.pop();
        h_map_.emplace(current.id(), current);
        // 探索周围的格子
        for (const auto& motion : motions) {
            auto node_new = current + motion;
            node_new.set_g(current.g() + motion.g());
            node_new.set_id(grid2Index(node_new.x(), node_new.y()));
            // 如果该node被访问了
            if (h_map_.find(node_new.id()) != h_map_.end()) {
                continue;
            }
            if (node_new.id() < 0 || node_new.id() >= map_size_) {
                continue;
            }
            //添加过了open_set
            if (open_set.find(node_new.id()) != open_set.end()) {
                // 更新g代价值
                if (open_set[node_new.id()].g() > node_new.g()) {
                    open_set[node_new.id()].set_g(node_new.g());
                }
            } else {
                // 否则
                open_list.push(node_new);
                open_set.emplace(node_new.id(), node_new);
            }
        }
    }
}

void HybridAStarPathPlanner::updateIndex(HybridNode& node) {
    node.set_id(static_cast<int>(node.theta() / kDeltaHeading) + _worldToIndex(node.x(), node.y()));
}

int HybridAStarPathPlanner::_worldToIndex(double wx, double wy) {
    double gx, gy;
    world2Map(wx, wy, gx, gy);
    return grid2Index(gx, gy);
}
}  // namespace mp::path_planner