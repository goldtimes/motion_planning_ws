#include "dubins_curve/dubins_curve.hh"
#include <array>
#include <cmath>
#include <utility>
#include "dubins_curve/dubins_tools.hh"
#include "ros/init.h"
#include "ros/rate.h"
#include "tf/transform_datatypes.h"

namespace dubins_curve {
void DubinsCurve::InitPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    std::cout << "InitPoseCallback" << std::endl;
    start_pose_.first.first = msg->pose.pose.position.x;
    start_pose_.first.second = msg->pose.pose.position.y;
    start_pose_.second = tf::getYaw(msg->pose.pose.orientation);
    // marker
    init_pose_marker_.header.frame_id = "odom";
    init_pose_marker_.header.stamp = ros::Time::now();
    init_pose_marker_.ns = "init_pose_marker";
    init_pose_marker_.id = 0;
    init_pose_marker_.type = visualization_msgs::Marker::ARROW;
    init_pose_marker_.action = visualization_msgs::Marker::ADD;
    init_pose_marker_.pose.position = msg->pose.pose.position;
    init_pose_marker_.pose.orientation = msg->pose.pose.orientation;
    init_pose_marker_.scale.x = 1.0;
    init_pose_marker_.scale.y = 0.05;
    init_pose_marker_.scale.z = 0.05;
    init_pose_marker_.color.a = 1.0;
    init_pose_marker_.color.r = 0.0;
    init_pose_marker_.color.g = 1.0;
    init_pose_marker_.color.b = 0.0;

    ROS_INFO_STREAM("the init pose:" << start_pose_.first.first << ", " << start_pose_.first.second << ", "
                                     << start_pose_.second);
    if (!get_init_pose_) {
        get_init_pose_ = true;
    }
}

void DubinsCurve::GoalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    goal_pose_.first.first = msg->pose.position.x;
    goal_pose_.first.second = msg->pose.position.y;
    goal_pose_.second = tf::getYaw(msg->pose.orientation);
    // marker
    goal_pose_marker_.header.frame_id = "odom";
    goal_pose_marker_.header.stamp = ros::Time::now();
    goal_pose_marker_.ns = "goal_pose_marker";
    goal_pose_marker_.id = 0;
    goal_pose_marker_.type = visualization_msgs::Marker::ARROW;
    goal_pose_marker_.action = visualization_msgs::Marker::ADD;
    goal_pose_marker_.pose.position = msg->pose.position;
    goal_pose_marker_.pose.orientation = msg->pose.orientation;
    goal_pose_marker_.scale.x = 1.0;
    goal_pose_marker_.scale.y = 0.05;
    goal_pose_marker_.scale.z = 0.05;
    goal_pose_marker_.color.a = 1.0;
    goal_pose_marker_.color.r = 0.0;
    goal_pose_marker_.color.g = 1.0;
    goal_pose_marker_.color.b = 0.0;

    ROS_INFO_STREAM("the goal pose:" << goal_pose_.first.first << ", " << goal_pose_.first.second << ", "
                                     << goal_pose_.second);
    if (!get_goal_target_) {
        get_goal_target_ = true;
    }
}

void DubinsCurve::DubinsCurveProcess() {
    // ROS_INFO("DubinsCurveProcess");
    // std::cout << "DubinsCurveProcess" << std::endl;
    // 用于可视化的一些信息
    std::pair<std::array<std::string, 3>, std::pair<std::string, std::array<double, 3>>> marker_info[4] = {
        std::make_pair(std::array<std::string, 3>{"left", "straight", "left"},
                       std::make_pair("lsl_marker", std::array<double, 3>{1, 0, 0})),
        std::make_pair(std::array<std::string, 3>{"left", "straight", "right"},
                       std::make_pair("lsr_marker", std::array<double, 3>{1, 0, 1})),
        std::make_pair(std::array<std::string, 3>{"right", "straight", "right"},
                       std::make_pair("rsr_marker", std::array<double, 3>{0, 0, 0})),
        std::make_pair(std::array<std::string, 3>{"right", "straight", "left"},
                       std::make_pair("rsl_marker", std::array<double, 3>{0, 1, 1})),
    };

    // 用于发布四条路径
    ros::Publisher marker_pub[4]{};
    for (int i = 0; i < 4; i++) {
        marker_pub[i] = nh_.advertise<visualization_msgs::Marker>(marker_info[i].second.first, 1);
    }
    while (ros::ok()) {
        ros::spinOnce();

        // 得到了目标点
        if (get_goal_target_ && get_init_pose_) {
            // 重置标志位
            get_goal_target_ = false;
            get_init_pose_ = false;
            // 开始计算dubins曲线
            ROS_INFO("Start draw the dubins curve");
            // 发布起始点和终点
            init_marker_pub_.publish(init_pose_marker_);
            goal_marker_pub_.publish(goal_pose_marker_);
            // best_index存放最短路径的dubins曲线
            int best_index = 0;
            auto solution_array = DubinsCurveSolver(best_index);
            ROS_INFO_STREAM("the best path is : " << marker_info[best_index].first.at(0) << ", "
                                                  << marker_info[best_index].first.at(1) << ", "
                                                  << marker_info[best_index].first.at(2) << ", ");
            // 绘制出所有的曲线，最优曲线最为明显！
            for (int i = 0; i < 4; i++) {
                if (i == best_index) {
                    auto path_pub =
                        DrawDubinsCurve(marker_info[i].second.second, solution_array[i], marker_info[i].first, 1.0);
                    marker_pub[i].publish(path_pub);
                } else {
                    auto path_pub =
                        DrawDubinsCurve(marker_info[i].second.second, solution_array[i], marker_info[i].first);
                    marker_pub[i].publish(path_pub);
                }
            }
        }
        // rate.sleep();
    }
}
// 计算4条dubins 曲线
std::array<curve_type, 4> DubinsCurve::DubinsCurveSolver(int& index) {
    // 根据当前位置计算圆心
    // 求取四个圆心，第一次左右转的圆心，以及第二次左右转的圆心
    dubins_tool_->CoordTransform(start_pose_, start_pose_.second, 0, radius_, "left", left_center1);
    dubins_tool_->CoordTransform(start_pose_, start_pose_.second, 0, radius_, "right", right_center1);
    dubins_tool_->CoordTransform(goal_pose_, goal_pose_.second, 0, radius_, "left", left_center2);
    dubins_tool_->CoordTransform(goal_pose_, goal_pose_.second, 0, radius_, "right", right_center2);
    // 计算dubins曲线
    std::array<curve_type, 4> solution_array = {LSLCurve(), LSRCurve(), RSRCurve(), RSLCurve()};
    std::array<double, 4> length_array = {solution_array[0].second, solution_array[1].second, solution_array[2].second,
                                          solution_array[3].second};

    auto min_length = std::min_element(length_array.begin(), length_array.end());
    index = std::distance(length_array.begin(), min_length);

    ROS_INFO_STREAM("The min distance:" << *min_length);

    return solution_array;
}

curve_type DubinsCurve::LSLCurve() {
    // 两个圆心的向量
    // 圆心2->圆心1的向量
    auto center_line = dubins_tool_->GetDiffVec(left_center2, left_center1);
    // 向量的中心
    double R = dubins_tool_->GetDistance(left_center2, left_center1) / 2;
    // - acos(0) 这个具体要作图算出 https://zhuanlan.zhihu.com/p/673466180
    double theta = std::atan2(center_line[1], center_line[0]) - acos(0);
    // 计算两个切点
    point_type tangent_pos1, tangent_pos2;
    dubins_tool_->CoordTransform(left_center1, theta, radius_, 0, "left", tangent_pos1);
    dubins_tool_->CoordTransform(left_center2, theta, radius_, 0, "left", tangent_pos2);

    std::array<point_type, 4> lsl_path = {left_center1, tangent_pos1, tangent_pos2, left_center2};
    return std::make_pair(lsl_path, GetDubinsCurveLength(left_center1, left_center2, tangent_pos1, tangent_pos2,
                                                         std::array<std::string, 3>{"left", "stright", "left"}));
}
curve_type DubinsCurve::LSRCurve() {
    // 两个圆心的差值向量
    auto center_line = dubins_tool_->GetDiffVec(right_center2, left_center1);
    double R = dubins_tool_->GetDistance(right_center2, left_center1) / 2;
    if (R < radius_) {
        ROS_INFO("LSR is not valid!!!");
        std::array<point_type, 4> lsr_path = {start_pose_, start_pose_, start_pose_, start_pose_};
        return std::make_pair(lsr_path, INFINITY);
    }
    // 角度计算，后面的那项要考虑到，否则不正确！！！可以自行推导或是参考我的博客
    double theta = atan2(center_line[1], center_line[0]) - acos(radius_ / R);

    // 计算两个切点
    point_type tangent_pos1, tangent_pos2;
    dubins_tool_->CoordTransform(left_center1, theta, radius_, 0, "left", tangent_pos1);
    dubins_tool_->CoordTransform(right_center2, theta + M_PI, radius_, 0, "right", tangent_pos2);

    // 记录两个圆心，两个切点，即所有需要的路径点已记录完毕
    std::array<point_type, 4> lsr_path = {left_center1, tangent_pos1, tangent_pos2, right_center2};

    return std::make_pair(lsr_path, GetDubinsCurveLength(left_center1, right_center2, tangent_pos1, tangent_pos2,
                                                         std::array<std::string, 3>{"left", "stright", "right"}));
}
curve_type DubinsCurve::RSRCurve() {
    // 两个圆心的差值向量
    auto center_line = dubins_tool_->GetDiffVec(right_center2, right_center1);
    double R = dubins_tool_->GetDistance(right_center2, right_center1) / 2;
    // 角度计算，后面的那项要考虑到，否则不正确！！！可以自行推导或是参考我的博客
    double theta = atan2(center_line[1], center_line[0]) + acos(0);

    // 计算两个切点
    point_type tangent_pos1, tangent_pos2;
    dubins_tool_->CoordTransform(right_center1, theta, radius_, 0, "right", tangent_pos1);
    dubins_tool_->CoordTransform(right_center2, theta, radius_, 0, "right", tangent_pos2);

    // 记录两个圆心，两个切点，即所有需要的路径点已记录完毕
    std::array<point_type, 4> rsr_path = {right_center1, tangent_pos1, tangent_pos2, right_center2};

    return std::make_pair(rsr_path, GetDubinsCurveLength(right_center1, right_center2, tangent_pos1, tangent_pos2,
                                                         std::array<std::string, 3>{"right", "stright", "right"}));
}
curve_type DubinsCurve::RSLCurve() {
    // 两个圆心的差值向量
    auto center_line = dubins_tool_->GetDiffVec(left_center2, right_center1);
    double R = dubins_tool_->GetDistance(left_center2, right_center1) / 2;
    if (R < radius_) {
        ROS_INFO("RSL is not valid!!!");
        std::array<point_type, 4> rsl_path = {start_pose_, start_pose_, start_pose_, start_pose_};
        return std::make_pair(rsl_path, INFINITY);
    }
    // 角度计算，后面的那项要考虑到，否则不正确！！！可以自行推导或是参考我的博客
    double theta = atan2(center_line[1], center_line[0]) + acos(radius_ / R);

    // 计算两个切点
    point_type tangent_pos1, tangent_pos2;
    dubins_tool_->CoordTransform(right_center1, theta, radius_, 0, "right", tangent_pos1);
    dubins_tool_->CoordTransform(left_center2, theta + M_PI, radius_, 0, "left", tangent_pos2);

    // 记录两个圆心，两个切点，即所有需要的路径点已记录完毕
    std::array<point_type, 4> rsl_path = {right_center1, tangent_pos1, tangent_pos2, left_center2};

    return std::make_pair(rsl_path, GetDubinsCurveLength(right_center1, left_center2, tangent_pos1, tangent_pos2,
                                                         std::array<std::string, 3>{"right", "stright", "left"}));
}

double DubinsCurve::GetDubinsCurveLength(const point_type& center1, const point_type& center2,
                                         const point_type& tangent_pos1, const point_type& tangent_pos2,
                                         const std::array<std::string, 3>& dir) {
    // 两段圆弧的长和圆心的向量长度
    // 第一段圆弧的长
    double arc_length1 = radius_ * dubins_tool_->GetDelta(dubins_tool_->GetDiffVec(start_pose_, center1),
                                                          dubins_tool_->GetDiffVec(tangent_pos1, center1), dir[0]);
    double arc_length2 = radius_ * dubins_tool_->GetDelta(dubins_tool_->GetDiffVec(tangent_pos2, center2),
                                                          dubins_tool_->GetDiffVec(goal_pose_, center2), dir[2]);
    double tangent_length = dubins_tool_->GetDistance(tangent_pos1, tangent_pos2);
    return arc_length1 + tangent_length + arc_length2;
}

visualization_msgs::Marker DubinsCurve::DrawDubinsCurve(const std::array<double, 3>& color, const curve_type& path_pub,
                                                        const std::array<std::string, 3>& dir, double alpha) {
    visualization_msgs::Marker marker_path;
    marker_path.type = visualization_msgs::Marker::LINE_STRIP;
    marker_path.header.frame_id = "odom";
    marker_path.header.stamp = ros::Time::now();
    marker_path.ns = "odom";
    marker_path.id = 0;
    marker_path.action = visualization_msgs::Marker::ADD;
    marker_path.lifetime = ros::Duration();
    marker_path.color.r = color[0];
    marker_path.color.g = color[1];
    marker_path.color.b = color[2];
    marker_path.color.a = alpha;
    marker_path.scale.x = 0.02;
    marker_path.pose.orientation.w = 1.0;
    // 第一段圆弧
    double start_angle = atan2(start_pose_.first.second - path_pub.first.at(0).first.second,
                               start_pose_.first.first - path_pub.first.at(0).first.first);
    double end_angle = atan2(path_pub.first.at(1).first.second - path_pub.first.at(0).first.second,
                             path_pub.first.at(1).first.first - path_pub.first.at(0).first.first);
    // 0.01弧度生成圆弧上的点
    double angle_iter = dir[0] == "left" ? 0.01 : -0.01;
    geometry_msgs::Point marker_point;
    for (double angle = start_angle;; angle += angle_iter) {
        marker_point.x = path_pub.first.at(0).first.first + radius_ * cos(angle);
        marker_point.y = path_pub.first.at(0).first.second + radius_ * sin(angle);
        marker_point.z = 0;
        marker_path.points.push_back(marker_point);
        if (angle <= -M_PI) {
            angle += 2 * M_PI;
        } else if (angle >= M_PI) {
            angle -= 2 * M_PI;
        }
        if (abs(angle - end_angle) <= 0.01) {
            break;
        }
    }
    // 直线
    marker_point.x = path_pub.first.at(2).first.first;
    marker_point.y = path_pub.first.at(2).first.second;
    marker_path.points.push_back(marker_point);
    // 二段圆弧
    start_angle = atan2(path_pub.first.at(2).first.second - path_pub.first.at(3).first.second,
                        path_pub.first.at(2).first.first - path_pub.first.at(3).first.first);
    end_angle = atan2(goal_pose_.first.second - path_pub.first.at(3).first.second,
                      goal_pose_.first.first - path_pub.first.at(3).first.first);

    angle_iter = dir[2] == "left" ? 0.01 : -0.01;
    for (double angle = start_angle;; angle += angle_iter) {
        marker_point.x = path_pub.first.at(3).first.first + radius_ * cos(angle);
        marker_point.y = path_pub.first.at(3).first.second + radius_ * sin(angle);
        marker_point.z = 0;
        marker_path.points.push_back(marker_point);
        if (angle <= -M_PI) {
            angle += 2 * M_PI;
        } else if (angle >= M_PI) {
            angle -= 2 * M_PI;
        }
        if (abs(angle - end_angle) <= 0.01) {
            break;
        }
    }

    return marker_path;
}

}  // namespace dubins_curve