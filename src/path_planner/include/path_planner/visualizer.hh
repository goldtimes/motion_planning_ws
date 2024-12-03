#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>
#include "costmap_2d/costmap_2d.h"
#include "path_planner/point.hh"
#include "path_planner/singleton.hh"
#include "ros/publisher.h"

namespace mp::path_planner::common {
class Visualizer {
   public:
    using Point2d = mp::path_planner::common::Point2d;
    using Points2d = mp::path_planner::common::Points2d;
    using Point3d = mp::path_planner::common::Point3d;
    using Points3d = mp::path_planner::common::Points3d;
    using Line2d = std::pair<Point2d, Point2d>;
    using Lines2d = std::vector<Line2d>;

   public:
    //    发布路径
    template <typename Points>
    static void publishPath(const Points& plan, const ros::Publisher& publisher, const std::string& frame_id) {
        // create visulized path plan
        nav_msgs::Path gui_plan;
        gui_plan.poses.resize(plan.size());
        gui_plan.header.frame_id = frame_id;
        gui_plan.header.stamp = ros::Time::now();
        for (unsigned int i = 0; i < plan.size(); i++) {
            gui_plan.poses[i].header.stamp = ros::Time::now();
            gui_plan.poses[i].header.frame_id = frame_id;
            gui_plan.poses[i].pose.position.x = plan[i].x();
            gui_plan.poses[i].pose.position.y = plan[i].y();
            gui_plan.poses[i].pose.position.z = 0.0;
            gui_plan.poses[i].pose.orientation.x = 0.0;
            gui_plan.poses[i].pose.orientation.y = 0.0;
            gui_plan.poses[i].pose.orientation.z = 0.0;
            gui_plan.poses[i].pose.orientation.w = 1.0;
        }

        publisher.publish(gui_plan);
    }

    template <typename Points>
    static void publishExpandZone(const Points& expand, const costmap_2d::Costmap2D* costmap,
                                  const ros::Publisher& publisher, const std::string& frame_id) {
        // 创建占据栅格
        nav_msgs::OccupancyGrid grid;
        float resolution = costmap->getResolution();

        // build expand
        grid.header.frame_id = frame_id;
        grid.header.stamp = ros::Time::now();
        grid.info.resolution = resolution;
        grid.info.width = costmap->getSizeInCellsX();
        grid.info.height = costmap->getSizeInCellsY();

        double wx, wy;
        costmap->mapToWorld(0, 0, wx, wy);
        // 地图原点
        grid.info.origin.position.x = wx - resolution / 2;
        grid.info.origin.position.y = wy - resolution / 2;
        grid.info.origin.position.z = 0.0;
        grid.info.origin.orientation.w = 1.0;
        // 地图大小
        grid.data.resize(grid.info.width * grid.info.height);
        // 地图原始值
        for (unsigned int i = 0; i < grid.data.size(); i++) {
            grid.data[i] = 0;
        }
        // lambda函数
        auto grid2index = [&](const Point3d& pt) {
            return static_cast<int>(pt.x()) + static_cast<int>(grid.info.width * pt.y());
        };
        // 将探索过的栅格值修改为50
        for (const auto& pt : expand) {
            grid.data[grid2index(pt)] = 50;
        }

        publisher.publish(grid);
    }

   private:
    static std_msgs::ColorRGBA _colorInit(double r, double g, double b, double a);

   public:
    static std_msgs::ColorRGBA RED;
    static std_msgs::ColorRGBA DARK_GREEN;
    static std_msgs::ColorRGBA PURPLE;

    enum class MARKER_TYPE {
        CUBE = 0,
        SPHERE = 1,
    };
};
using VisualizerPtr = mp::path_planner::common::Singleton<Visualizer>;

}  // namespace mp::path_planner::common