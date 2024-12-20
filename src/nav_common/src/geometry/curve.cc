#include "nav_common/geometry/curve.hh"

namespace mp::common::geometry {
Curve::Curve(double step) : step_(step) {
}
double Curve::len(Points2d path) {
    double length = 0.0;
    for (size_t i = 1; i < path.size(); ++i)
        length += std::hypot(path[i - 1].x() - path[i].x(), path[i - 1].y() - path[i].y());
    return length;
}

void Curve::setStep(double step) {
    // CHECK_GT(step, 0.0);
    step_ = step;
}

}  // namespace mp::common::geometry