#include <memory>
#include <unordered_map>
#include "dubins_curve/dubins_curve.hh"
#include "nav_common/geometry/curve.hh"
#include "path_planner/graph_planner/astar_planner.hh"
#include "path_planner/path_planner.hh"
// #include "nav"
namespace mp::path_planner {
class HybridAStarPathPlanner : public PathPlanner {
   public:
    using Node = mp::common::structure::Node<double>;
    class HybridNode : public Node {
       public:
        /**
         * @brief Constructor for Hybrid Node class
         * @param x   x value
         * @param y   y value
         * @param t
         * @param g   g value, cost to get to this node
         * @param h   h value, heuritic cost of this node
         * @param id  node's id
         * @param pid node's parent's id
         */
        HybridNode(double x = 0, double y = 0, double theta = 0, double g = 0.0, double h = 0.0, int id = 0,
                   int pid = 0, int prim = 0);
        double theta() const;
        void set_theta(double theta);
        /**
         * @brief Overloading operator + for Node class
         * @param n another Node
         * @return Node with current node's and input node n's values added
         */
        HybridNode operator+(const HybridNode& n) const;

        /**
         * @brief Overloading operator == for Node class
         * @param n another Node
         * @return true if current node equals node n, else false
         */
        bool operator==(const HybridNode& n) const;

        /**
         * @brief Overloading operator != for Node class
         * @param n another Node
         * @return true if current node equals node n, else false
         */
        bool operator!=(const HybridNode& n) const;

        /**
         * @brief Get permissible motion
         * @return  Node vector of permissible motions
         */
        static std::vector<HybridNode> getMotion();

       private:
        double theta_;
        int prim_;
    };

    /**
     * @brief Construct a new Hybrid A* object
     * @param costmap   the environment for path planning
     * @param is_reverse whether reverse operation is allowed
     * @param max_curv   maximum curvature of model
     */
    HybridAStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, bool is_reverse, double max_curv);

    /**
     * @brief Destory the Hybrid A* object
     */
    ~HybridAStarPathPlanner() = default;

    /**
     * @brief Hybrid A* implementation
     * @param start          start node
     * @param goal           goal node
     * @param path           optimal path consists of Node
     * @param expand         containing the node been search during the process
     * @return true if path found, else false
     */
    bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand);

    /**
     * @brief Try using Dubins curves to connect the start and goal
     * @param start          start node
     * @param goal           goal node
     * @param path           dubins path between start and goal
     * @return true if shot successfully, else false
     */
    bool dubinsShot(const HybridNode& start, const HybridNode& goal, std::vector<HybridNode>& path);

    /**
     * @brief update index of hybrid node
     * @param node hybrid node to update
     */
    void updateIndex(HybridNode& node);

    /**
     * @brief update the h-value of hybrid node
     * @param node hybrid node to update
     */
    void updateHeuristic(HybridNode& node);

    /**
     * @brief generate heurisitic map using A* algorithm, each matric of map is the distance between it and start.
     * @param start start node
     */
    void genHeurisiticMap(const Node& start);

   private:
    HybridNode goal_;
    // h cost
    std::unordered_map<int, Node> h_map_;
    // 允许后退
    bool is_reverse_;
    // 曲率
    double max_curv_;
    // A* planner
    std::unique_ptr<AstarGlobalPlanner> a_star_planner_;
    // dubins曲线生成
    std::unique_ptr<dubins_curve::DubinsCurve> dubins_gen_;
};
}  // namespace mp::path_planner