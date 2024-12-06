#include "dubins_curve/dubins_curve.hh"
#include "ros/init.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "dubins_curve");
    ros::NodeHandle n;
    auto node = new dubins_curve::DubinsCurve(n);
    ros::Rate loop_rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        node->DubinsCurveProcess();
        // std::cout << "test" << std::endl;
        loop_rate.sleep();
    }

    return 0;
}