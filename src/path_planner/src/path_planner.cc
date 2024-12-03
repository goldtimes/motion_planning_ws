#include "path_planner/path_planner.hh"
#include <costmap_2d/cost_values.h>
namespace mp::path_planner {
void PathPlanner::setFactor(float factor) {
}

costmap_2d::Costmap2D* PathPlanner::getCostMap() const {
    return costmap_;
}

int PathPlanner::getMapSize() const {
    return map_size_;
}

// grid的x,y转换为下标
int PathPlanner::grid2Index(int x, int y) {
    // x,y为行列数
    return x + static_cast<int>(costmap_->getSizeInCellsX() * y);
}

void PathPlanner::index2Grid(int i, int& x, int& y) {
    x = static_cast<int>(i % costmap_->getSizeInCellsX());
    y = static_cast<int>(i / costmap_->getSizeInCellsX());
}

// world to map
bool PathPlanner::world2Map(double wx, double wy, double& mx, double& my) {
    // 超过了地图的范围
    if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY()) {
        return false;
    }
    mx = (wx - costmap_->getOriginX()) / costmap_->getResolution();
    my = (wy - costmap_->getOriginY()) / costmap_->getResolution();
    if (mx < costmap_->getSizeInCellsX() || my < costmap_->getSizeInCellsY()) {
        return true;
    }
    return false;
}

void PathPlanner::map2World(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + (mx + 0.5) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my + 0.5) * costmap_->getResolution();
}
/*
    0 0 0 0
    0 0 0 0
    0 0 0 0 costmap
*/

void PathPlanner::outlierMap() {
    // 获取costmap的大小
    auto nx = costmap_->getSizeInCellsX();
    auto ny = costmap_->getSizeInCellsY();
    // 获取代价地图的字符映射（char map），这是一个指向地图数据的指针
    auto pc = costmap_->getCharMap();
    // 遍历上边界,遍历每一列
    for (int i = 0; i < nx; ++i) {
        // 上边界加上障碍物的值
        *pc++ = costmap_2d::LETHAL_OBSTACLE;
    }
    // 指向最后一行
    pc = costmap_->getCharMap() + (ny - 1) * nx;
    for (int i = 0; i < nx; ++i) {
        *pc++ = costmap_2d::LETHAL_OBSTACLE;
    }
    // 指回最开始的地方
    pc = costmap_->getCharMap();
    // 遍历左边界，按行
    for (int i = 0; i < ny; ++i, pc += nx) {
        *pc = costmap_2d::LETHAL_OBSTACLE;
    }
    // 又边界
    pc = costmap_->getCharMap() + nx - 1;
    for (int i = 0; i < ny; ++i, pc += nx) {
        *pc = costmap_2d::LETHAL_OBSTACLE;
    }
}

}  // namespace mp::path_planner