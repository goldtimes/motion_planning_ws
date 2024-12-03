#include "path_planner/graph_planner/astar_planner.hh"
#include <cmath>
#include <queue>
#include <unordered_map>
#include <vector>
#include "costmap_2d/cost_values.h"
#include "path_planner/path_planner.hh"

namespace mp::path_planner {

AstarGlobalPlanner::AstarGlobalPlanner(costmap_2d::Costmap2DROS* costmap_ros, bool dijstra, bool gbfs)
    : PathPlanner(costmap_ros) {
    // 合理的,两个不同时为true
    if (!(dijstra && gbfs)) {
        is_dijstra_ = dijstra;
        is_gbgs_ = gbfs;
    } else {
        is_dijstra_ = false;
        is_gbgs_ = false;
    }
}
// global_planner_ros中调用Astar来规划
bool AstarGlobalPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand) {
    Node start_node(start.x(), start.y());
    Node goal_node(goal.x(), goal.y());
    // 栅格的索引下标
    start_node.set_id(grid2Index(start_node.x(), start_node.y()));
    goal_node.set_id(grid2Index(goal_node.x(), goal_node.y()));

    path.clear();
    expand.clear();

    // open_list
    std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
    std::unordered_map<int, Node> closed_lists;

    open_list.push(start_node);
    // 循环处理open_list
    while (!open_list.empty()) {
        auto current = open_list.top();
        open_list.pop();
        // 如果当前节点已经加入了closed list 直接跳过
        if (closed_lists.find(current.id()) != closed_lists.end()) {
            continue;
        }
        // 添加如closed list中
        closed_lists.insert({current.id(), current});
        // 探索过的点
        expand.emplace_back(current.x(), current.y());
        // 找到目标点
        if (current == goal_node) {
            // 从探索的closed list中找到Node
            const auto backtrance = _convertClosedListToPath(closed_lists, start_node, goal_node);
            // 将node 变为path
            for (auto iter = backtrance.rbegin(); iter != backtrance.rend(); ++iter) {
                path.emplace_back(iter->x(), iter->y());
            }
            return true;
        }
        // 探索 构建f(n)代价
        for (const auto& nearby : nearby_8) {
            auto node_new = current + nearby;
            // 前向探索的代价
            node_new.set_g(current.g() + nearby.g());
            // 索引
            node_new.set_id(grid2Index(node_new.x(), node_new.y()));
            // 已经被其他节点访问过了
            if (closed_lists.find(node_new.id()) != closed_lists.end()) {
                continue;
            }
            // 设置父节点
            node_new.set_pid(current.id());
            // 计算代价
            // 跳过一些异常的点
            // next node hit the boundary or obstacle
            // prevent planning failed when the current within inflation
            if (node_new.id() < 0 || node_new.id() >= map_size_ ||
                (costmap_->getCharMap()[node_new.id()] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
                 costmap_->getCharMap()[node_new.id()] >= costmap_->getCharMap()[current.id()])) {
                continue;
            }
            if (!is_dijstra_) {
                // 计算到目标点的代价
                node_new.set_h(std::hypot(node_new.x() - goal_node.x(), node_new.y() - goal_node.y()));
            }
            if (is_gbgs_) {
                node_new.set_g(0.0);
            }
            open_list.push(node_new);
        }
    }
    return false;
}
}  // namespace mp::path_planner