#pragma once
#include "nav_common/geometry/point.hh"
#include "nav_common/math/math_utils.hh"

namespace mp::common::geometry {
class Curve {
   public:
    Curve(double step);
    virtual ~Curve() = default;
    virtual bool run(const Points2d points, Points2d& path) = 0;
    virtual bool run(const Points3d points, Points2d& path) = 0;
    double len(Points2d path);

    /**
     * @brief Configure the simulation step.
     * @param step    Simulation or interpolation size
     */
    void setStep(double step);

   protected:
    // Simulation or interpolation size
    double step_;
};
}  // namespace mp::common::geometry