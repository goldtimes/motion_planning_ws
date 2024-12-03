#pragma once

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <ros/ros.h>
#include <unordered_map>
#include <vector>
#include "node.hh"
#include "point.hh"

namespace mp::path_planner {
class PathPlanner {
   public:
    using Point2d = mp::path_planner::common::Point2d;
    using Point3d = mp::path_planner::common::Point3d;
    using Points2d = mp::path_planner::common::Points2d;
    using Points3d = mp::path_planner::common::Points3d;

   public:
    PathPlanner(costmap_2d::Costmap2DROS* costmap_ros)
        : factor_(0.5), map_size_(0), costmap_ros_(costmap_ros), costmap_(costmap_ros->getCostmap()) {
        map_size_ = static_cast<int>(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    }

    virtual ~PathPlanner() = default;

    virtual bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand) = 0;

    /**
     * @brief Set or reset obstacle factor
     * @param factor obstacle factor
     */
    void setFactor(float factor);

    /**
     * @brief get the costmap
     * @return costmap costmap2d pointer
     */
    costmap_2d::Costmap2D* getCostMap() const;

    /**
     * @brief get the size of costmap
     * @return map_size the size of costmap
     */
    int getMapSize() const;

    /**
     * @brief Transform from grid map(x, y) to grid index(i)
     * @param x grid map x
     * @param y grid map y
     * @return index
     */
    int grid2Index(int x, int y);

    /**
     * @brief Transform from grid index(i) to grid map(x, y)
     * @param i grid index i
     * @param x grid map x
     * @param y grid map y
     */
    void index2Grid(int i, int& x, int& y);

    /**
     * @brief Tranform from world map(x, y) to costmap(x, y)
     * @param mx costmap x
     * @param my costmap y
     * @param wx world map x
     * @param wy world map y
     * @return true if successfull, else false
     */
    bool world2Map(double wx, double wy, double& mx, double& my);

    /**
     * @brief Tranform from costmap(x, y) to world map(x, y)
     * @param mx costmap x
     * @param my costmap y
     * @param wx world map x
     * @param wy world map y
     */
    void map2World(double mx, double my, double& wx, double& wy);

    /**
     * @brief Inflate the boundary of costmap into obstacles to prevent cross planning
     */
    void outlierMap();

   protected:
    // A*探索的所有closed_list中反向取出最优的路径，从末尾开始，找父节点
    template <typename Node>
    std::vector<Node> _convertClosedListToPath(std::unordered_map<int, Node>& closed_list, const Node& start,
                                               const Node& goal) {
        std::vector<Node> path;
        // 目标点
        auto current = closed_list.find(goal.id());
        // 没遍历到起点的情况，一直遍历
        while (current->second != start) {
            // 放到路径中
            path.emplace_back(current->second.x(), current->second.y());
            // 然后找到parent_id,迭代赋给parent
            auto parent_it = closed_list.find(current->second.pid());
            if (parent_it != closed_list.end()) {
                current = parent_it;
            } else {
                // 路径没找到
                return {};
            }
        }
        path.push_back(start);
        return path;
    }

    template <typename Node>
    std::vector<Node> _convertBiClosedListToPath(std::unordered_map<int, Node>& f_closed_list,
                                                 std::unordered_map<int, Node>& b_closed_list, const Node& start,
                                                 const Node& goal, const Node& boundary) {
    }

   protected:
    int map_size_;
    // 障碍物的系数
    float factor_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
};
}  // namespace mp::path_planner
