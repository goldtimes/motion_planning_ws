#pragma once
#include <cmath>
#include <vector>
#include "../path_planner.hh"
#include "costmap_2d/costmap_2d_ros.h"

namespace mp::path_planner {
class AstarGlobalPlanner : public PathPlanner {
   public:
    using Node = mp::path_planner::common::Node<int>;

    explicit AstarGlobalPlanner(costmap_2d::Costmap2DROS* costmap_ros, bool dijstra = false, bool gbfs = false);

    /**
     * @brief A* implementation
     * @param start          start node
     * @param goal           goal node
     * @param path           optimal path consists of Node
     * @param expand         containing the node been search during the process
     * @return true if path found, else false
     */
    bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand);

   private:
    // dijstra算法
    bool is_dijstra_;
    // 贪心算法
    bool is_gbgs_;
    // 周围的8个格子和访问它们的代价
    const std::vector<Node> nearby_8 = {{0, 1, 1.0},           {1, 0, 1.0},           {0, -1, 1.0},
                                        {-1, 0, 1.0},          {1, 1, std::sqrt(2)},  {1, -1, std::sqrt(2)},
                                        {-1, 1, std::sqrt(2)}, {-1, -1, std::sqrt(2)}};
};
}  // namespace mp::path_planner