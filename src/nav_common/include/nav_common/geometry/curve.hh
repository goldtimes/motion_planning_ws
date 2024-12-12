#pragma once

namespace mp::common::geometry {
class Curve {
   public:
    Curve(double step);
    virtual ~Curve() = default;
    // virtual bool run(const Points2d points, Points2d& path) = 0;
    // virtual bool run(const Points3d points, Points2d& path) = 0;

   protected:
    double step_;
};
}  // namespace mp::common::geometry