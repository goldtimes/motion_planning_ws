#include "path_planner/path_planner_node.hh"
#include <ros/ros.h>
#include <tf2/utils.h>
#include <visualization_msgs/MarkerArray.h>
#include <mutex>
#include "costmap_2d/costmap_2d.h"
#include "geometry_msgs/PoseStamped.h"
#include "path_planner/graph_planner/astar_planner.hh"
#include "path_planner/visualizer.hh"
#include "pluginlib/class_list_macros.hpp"
#include "ros/time.h"

PLUGINLIB_EXPORT_CLASS(mp::path_planner::PathPlannerNode, nav_core::BaseGlobalPlanner);

namespace mp::path_planner {
PathPlannerNode::PathPlannerNode() : initialized_(false), g_planner_(nullptr) {
}

PathPlannerNode::PathPlannerNode(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : PathPlannerNode() {
    initialize(name, costmap_ros);
}

void PathPlannerNode::initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos) {
    costmap_ros_ = costmapRos;
    initialize(name);
}

void PathPlannerNode::initialize(std::string name) {
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
        make_plan_srv_ = priv_nh.advertiseService("make_plan", &PathPlannerNode::makePlanService, this);
    } else {
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
    }
}

bool PathPlannerNode::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
    makePlan(req.start, req.goal, resp.plan.poses);
    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;

    return true;
}

bool PathPlannerNode::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                               std::vector<geometry_msgs::PoseStamped>& plan) {
    return makePlan(start, goal, tolerance_, plan);
}

// 真正的规划全局路径的地方
bool PathPlannerNode::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                               double tolerance, std::vector<geometry_msgs::PoseStamped>& plan) {
    // 加锁
    std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*g_planner_->getCostMap()->getMutex());
    if (!initialized_) {
        ROS_ERROR(
            "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }
    // clear existing plan
    plan.clear();
    // 判断frame_id是否相同
    if (goal.header.frame_id != frame_id_) {
        ROS_ERROR("The goal pose passed to this planner must be in the %s frame. It is instead in the %s frame.",
                  frame_id_.c_str(), goal.header.frame_id.c_str());
        return false;
    }

    if (start.header.frame_id != frame_id_) {
        ROS_ERROR("The start pose passed to this planner must be in the %s frame. It is instead in the %s frame.",
                  frame_id_.c_str(), start.header.frame_id.c_str());
        return false;
    }
    double wx = start.pose.position.x, wy = start.pose.position.y;
    double g_start_x, g_start_y, g_goal_x, g_goal_y;
    // 起始位姿不再地图当中
    if (!g_planner_->world2Map(wx, wy, g_start_x, g_start_y)) {
        ROS_WARN(
            "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot "
            "has "
            "been properly localized?");
        return false;
    }
    // 终点位姿不在地图中
    wx = goal.pose.position.x, wy = goal.pose.position.y;
    if (!g_planner_->world2Map(wx, wy, g_goal_x, g_goal_y)) {
        ROS_WARN_THROTTLE(1.0,
                          "The goal sent to the global planner is off the global costmap. Planning will always fail to "
                          "this goal.");
        return false;
    }
    if (is_outlier_) {
        g_planner_->outlierMap();
    }
    // 规划
    PathPlanner::Points3d origin_path;
    PathPlanner::Points3d expand;
    bool path_found = false;
    path_found = g_planner_->plan({g_start_x, g_start_y, tf2::getYaw(start.pose.orientation)},
                                  {g_goal_x, g_goal_y, tf2::getYaw(goal.pose.orientation)}, origin_path, expand);
    if (path_found) {
        // 将规划得到的路径转换为ros path
        if (_getPlanFromPath(origin_path, plan)) {
            geometry_msgs::PoseStamped goalCopy = goal;
            goalCopy.header.stamp = ros::Time::now();
            plan.push_back(goalCopy);
            PathPlanner::Points3d origin_plan, prune_plan;
            // 遍历所有的找到的点
            for (const auto& pt : plan) {
                origin_plan.emplace_back(pt.pose.position.x, pt.pose.position.y);
            }
            // 可视化
            const auto& visualizer = mp::path_planner::common::VisualizerPtr::Instance();
            if (is_expand_) {
                if (planner_type_ == PLANNER_TYPE::GRAPH_PLANNER) {
                    // 发布expand地图
                    visualizer->publishExpandZone(expand, costmap_ros_->getCostmap(), expand_pub_, frame_id_);
                }
            }
            // 可视化路径
            visualizer->publishPath(origin_plan, plan_pub_, frame_id_);
        } else {
            ROS_ERROR("Failed to get a plan from path when a legal path was found. This shouldn't happen.");
        }

    } else {
        ROS_ERROR("Failed to get a path.");
    }
    return !plan.empty();
}

bool PathPlannerNode::_getPlanFromPath(PathPlanner::Points3d& path, std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR(
            "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }
    plan.clear();
    for (const auto& pt : path) {
        double wx, wy;
        g_planner_->map2World(pt.x(), pt.y(), wx, wy);

        // coding as message type
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = frame_id_;
        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
    }

    return !plan.empty();
}

}  // namespace mp::path_planner