#pragma once

#include <cmath>
#include <limits>
#include <vector>

namespace mp::path_planner::math {
/**
 * @brief Perform modulus operation on 2π.
 * @param theta    the angle to modulu
 * @return theta_m the angle after modulus operator
 */
inline double mod2pi(double theta) {
    return theta - 2 * M_PI * std::floor(theta / (2 * M_PI));
}
}  // namespace mp::path_planner::math