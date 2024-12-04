#pragma once
#include <cmath>
#include <vector>
#include "../path_planner.hh"
#include "costmap_2d/costmap_2d_ros.h"

namespace mp::path_planner {
class AstarGlobalPlanner : public PathPlanner {
   public:
    /**
     * @brief Construct a new AStar object
     * @param costmap   the environment for path planning
     * @param dijkstra   using diksktra implementation
     * @param gbfs       using gbfs implementation
     */
    AstarGlobalPlanner(costmap_2d::Costmap2DROS* costmap_ros, bool dijkstra = false, bool gbfs = false);

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
    bool is_dijkstra_;  // using diksktra
    bool is_gbfs_;      // using greedy best first search(GBFS)

    using Node = mp::path_planner::common::Node<int>;
    const std::vector<Node> motions = {
        {0, 1, 1.0},          {1, 0, 1.0},           {0, -1, 1.0},          {-1, 0, 1.0},
        {1, 1, std::sqrt(2)}, {1, -1, std::sqrt(2)}, {-1, 1, std::sqrt(2)}, {-1, -1, std::sqrt(2)},
    };
};
}  // namespace mp::path_planner