#pragma once

#include <angles/angles.h>
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dynamic_reconfigure/server.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/buffer.h>
#include <memory>
#include <vector>
#include "base_local_planner/local_planner_util.h"
#include "dwa.hh"
#include "dwa_controller/DWAControllerConfig.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "ros/publisher.h"

namespace mp::controller {
class DWAController : public nav_core::BaseLocalPlanner {
   public:
    DWAController();
    ~DWAController();

    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* cosmap_ros);

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    bool dwaComputeVelocityCommands(geometry_msgs::PoseStamped& global_pose, geometry_msgs::Twist& cmd_vel);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    bool isGoalReached();
    bool isInitialized() {
        return initialized_;
    }

   private:
    void reconfigureCB(dwa_controller::DWAControllerConfig& config, uint32_t level);

    void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

    void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

   private:
    //    坐标变换工具
    tf2_ros::Buffer* tf_;
    // 全局规划路径
    ros::Publisher g_plan_pub_;
    // 局部规划路径
    ros::Publisher l_plan_pub_;
    base_local_planner::LocalPlannerUtil planner_util_;
    // 地图信息
    costmap_2d::Costmap2DROS* costmap_ros_;
    // 动态配置
    dynamic_reconfigure::Server<dwa_controller::DWAControllerConfig>* dsrv_;
    dwa_controller::DWAControllerConfig default_config_;
    std::shared_ptr<DWA> dp_;
    bool setup_;
    geometry_msgs::PoseStamped current_pose_;
    base_local_planner::LatchedStopRotateController latchedStopRotationController_;
    bool initialized_;
    base_local_planner::OdometryHelperRos odom_helper_;
    std::string odom_topic_;
};
}  // namespace mp::controller
