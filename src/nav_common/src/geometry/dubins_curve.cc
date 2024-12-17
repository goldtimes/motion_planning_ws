#include <Eigen/Dense>
#include <cassert>
#include <climits>
#include <iostream>

#include "nav_common/geometry/dubins_curve.hh"
#include "nav_common/math/math_utils.hh"

namespace mp::common::geometry {
namespace {
#define UNPACK_DUBINS_INPUT(alpha, beta) \
    double sin_a = sin(alpha);           \
    double sin_b = sin(beta);            \
    double cos_a = cos(alpha);           \
    double cos_b = cos(beta);            \
    double cos_a_b = cos(alpha - beta);
}  // namespace
enum {
    DUBINS_NONE = -1,  // 无解，计算的dubins距离为负
    DUBINS_L = 0,
    DUBINS_S = 1,
    DUBINS_R = 2,
};

DubinsCurve::DubinsCurve(double step, double max_curv) : Curve(step), max_curv_(max_curv) {
}
DubinsCurve::DubinsCurve() : Curve(0.1), max_curv_(0.25) {
}

DubinsCurve::~DubinsCurve() {
}

void DubinsCurve::setMaxCurv(double max_curv)
{
  assert(max_curv > 0);
  max_curv_ = max_curv;
}

}  // namespace mp::common::geometry