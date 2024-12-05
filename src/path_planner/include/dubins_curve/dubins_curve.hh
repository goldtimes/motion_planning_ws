#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include "dubins_tools.hh"
#include "ros/node_handle.h"
#include "ros/subscriber.h"

namespace dubins_curve {
class DubinsCurve {
   public:
    DubinsCurve(const ros::NodeHandle& nh) : nh_(nh) {
        init_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("init_pose_marker", 1);
        goal_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("goal_pose_marker", 1);
        // 监听
        init_pose_sub_ = nh_.subscribe("/initialpose", 1, &DubinsCurve::InitPoseCallback, this);
        goal_pose_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &DubinsCurve::GoalPoseCallback, this);
    }
    ~DubinsCurve() = default;

    void DubinsCurveProcess();
    /**
     * @brief 求解Dubins曲线的主要函数
     *
     * @param index 最优路径的index，0-3分别为LSL，LSR，RSR，RSL
     * @return std::array<curve_type, 4> 四条路径的所有路径点
     */
    std::array<curve_type, 4> DubinsCurveSolver(int& index);

    visualization_msgs::Marker DrawDubinsCurve(const std::array<double, 3>& color, const curve_type& path_pub,
                                               const std::array<std::string, 3>& dir, double alpha = 0.15);

   private:
    void InitPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    /**
     * @brief 获取目标位置点的回调函数，由rviz人为设置，并绘制在RVIZ上
     *
     * @param msg 目标点的信息
     */
    void GoalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    curve_type LSLCurve();
    curve_type LSRCurve();
    curve_type RSRCurve();
    curve_type RSLCurve();

    /**
     * @brief Get the Dubins Curve Length object
     *
     * @param center1 第一个圆心座标点
     * @param center2 第二个圆心座标点
     * @param tangent_pos1 第一个圆上的切点
     * @param tangent_pos2 第二个圆上的切点
     * @return double 整条路径的长度
     */
    double GetDubinsCurveLength(const point_type& center1, const point_type& center2, const point_type& tangent_pos1,
                                const point_type& tangent_pos2, const std::array<std::string, 3>& dir);

   private:
    ros::NodeHandle nh_;
    // 转弯半径，差速轮的转弯半径为0也可以认为差速轮的转弯半径支持任意值
    double radius_ = 1.0;
    point_type left_center1, left_center2;
    point_type right_center1, right_center2;
    point_type start_pose_, goal_pose_;

    // pub
    ros::Publisher init_marker_pub_, goal_marker_pub_;
    visualization_msgs::Marker init_pose_marker_, goal_pose_marker_;

    // sub
    ros::Subscriber init_pose_sub_, goal_pose_sub_;
    DubinsTool* dubins_tool_;
    bool get_goal_target_;
    bool get_init_pose_;
};
}  // namespace dubins_curve