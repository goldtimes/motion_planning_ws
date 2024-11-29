#include "global_planner/visualizer.hh"

namespace mp::global_planner::common {
// static 声明
std_msgs::ColorRGBA Visualizer::RED = Visualizer::_colorInit(1.0, 0.0, 0.0, 1.0);
std_msgs::ColorRGBA Visualizer::DARK_GREEN = Visualizer::_colorInit(0.43, 0.54, 0.24, 0.5);
std_msgs::ColorRGBA Visualizer::PURPLE = Visualizer::_colorInit(1.0, 0.0, 1.0, 1.0);

std_msgs::ColorRGBA Visualizer::_colorInit(double r, double g, double b, double a) {
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}
}  // namespace mp::global_planner::common