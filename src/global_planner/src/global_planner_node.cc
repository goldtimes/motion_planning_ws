#include "global_planner/global_planner_node.hh"
#include <ros/node_handle.h>
#include <visualization_msgs/MarkerArray.h>
#include "global_planner/global_planner.hh"
#include "global_planner/graph_planner/astar_planner.hh"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(mp::global_planner::GlobalPlannerNode, nav_core::BaseGlobalPlanner);

namespace mp::global_planner {
GlobalPlannerNode::GlobalPlannerNode() : initialized_(false), g_planner_(nullptr) {
}

GlobalPlannerNode::GlobalPlannerNode(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : GlobalPlannerNode() {
    initialize(name, costmap_ros);
}

void GlobalPlannerNode::initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos) {
    costmap_ros_ = costmapRos;
    initialize(name);
}

void GlobalPlannerNode::initialize(std::string name) {
    // 初始化ros pub service
    // 初始化全局规划器
    if (!initialized_) {
        initialized_ = true;

        ros::NodeHandle priv_nh("~/" + name);
        frame_id_ = costmap_ros_->getGlobalFrameID();
        // 加载参数
        priv_nh.param("default_tolerance", tolerance_, 0.0);
        priv_nh.param("outlier_map", is_outlier_, false);
        priv_nh.param("obstacle_factor", factor_, 0.5);
        priv_nh.param("expand_zone", is_expand_, false);
        priv_nh.param("planner_name", planner_name_, static_cast<std::string>("a_star"));
        if (planner_name_ == "a_star") {
            g_planner_ = std::make_shared<AstarGlobalPlanner>(costmap_ros_);
            planner_type_ = PLANNER_TYPE::GRAPH_PLANNER;
        } else if (planner_name_ == "dijstra") {
            g_planner_ = std::make_shared<AstarGlobalPlanner>(costmap_ros_, true);
            planner_type_ = PLANNER_TYPE::GRAPH_PLANNER;
        } else if (planner_name_ == "gbfs") {
            g_planner_ = std::make_shared<AstarGlobalPlanner>(costmap_ros_, false, true);
            planner_type_ = PLANNER_TYPE::GRAPH_PLANNER;
        }

        ROS_INFO("Using global path planner:%s", planner_name_.c_str());

        plan_pub_ = priv_nh.advertise<nav_msgs::Path>("plan", 1);
        tree_pub_ = priv_nh.advertise<visualization_msgs::MarkerArray>("random_tree", 1);
        particles_pub_ = priv_nh.advertise<visualization_msgs::MarkerArray>("particles", 1);
        // register explorer visualization publisher
        // 可视化搜索的区域
        expand_pub_ = priv_nh.advertise<nav_msgs::OccupancyGrid>("expand", 1);
        // register planning service
        make_plan_srv_ = priv_nh.advertiseService("make_plan", &GlobalPlannerNode::makePlanService, this);
    } else {
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
    }
}

bool GlobalPlannerNode::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
    makePlan(req.start, req.goal, resp.plan.poses);
    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;

    return true;
}

bool GlobalPlannerNode::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                 std::vector<geometry_msgs::PoseStamped>& plan) {
    return makePlan(start, goal, tolerance_, plan);
}

// 真正的规划全局路径的地方
bool GlobalPlannerNode::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                 double tolerance, std::vector<geometry_msgs::PoseStamped>& plan) {
}

}  // namespace mp::global_planner