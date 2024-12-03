/**
    继承ros的base_global_planner接口
 */
#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <ros/ros.h>
#include <memory>
#include <string>
#include "costmap_2d/costmap_2d_ros.h"
#include "path_planner.hh"
#include "ros/publisher.h"
#include "ros/service_server.h"

namespace mp::path_planner {

class PathPlannerNode : public nav_core::BaseGlobalPlanner {
   public:
    // 构造函数
    PathPlannerNode();
    PathPlannerNode(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    ~PathPlannerNode() = default;
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
    void initialize(std::string name);
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance,
                  std::vector<geometry_msgs::PoseStamped>& plan);
    /**
     * @brief Regeister planning service
     * @param req  request from client
     * @param resp response from server
     */
    bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

   protected:
    bool _getPlanFromPath(PathPlanner::Points3d& path, std::vector<geometry_msgs::PoseStamped>& plan);

   protected:
    // 三种路径搜索的枚举
    enum class PLANNER_TYPE {
        GRAPH_PLANNER = 0,
        SAMPLE_PLANNER = 1,
        EVOLUTION_PLANNER = 2,
    };

   protected:
    // 全局规划器是否被初始化
    bool initialized_;
    // 代价地图
    costmap_2d::Costmap2DROS* costmap_ros_;
    // 全局地图的frame_id
    std::string frame_id_;
    // 全局规划器的名字
    std::string planner_name_;
    // 全局规划器的实现
    std::shared_ptr<PathPlanner> g_planner_;
    // 全局路径发布
    ros::Publisher plan_pub_;
    // 探索的路径发布
    ros::Publisher expand_pub_;
    // 采样的路径发布
    ros::Publisher tree_pub_;
    // 粒子发布
    ros::Publisher particles_pub_;
    // 接受规划的请求
    ros::ServiceServer make_plan_srv_;
    // 三种路径搜索的枚举
    PLANNER_TYPE planner_type_;

   private:
    // 超出地图边界
    bool is_outlier_;
    // 发布expand地图
    bool is_expand_;
    // 距离目标的容忍度
    double tolerance_;
    // 障碍物的膨胀系数
    double factor_;
};
}  // namespace mp::path_planner