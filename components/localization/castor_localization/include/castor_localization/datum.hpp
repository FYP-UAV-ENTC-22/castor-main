// Where a geodetic point lies in the team's world frame, free of ROS so it can be unit-tested.
//
// The world frame is east-north-up with its origin at the datum, one geodetic point the whole team agrees on.
// PX4 keeps each vehicle's position in its own local frame, whose origin is wherever its estimator started, and
// reports where that origin is on the globe. PX4 gets its local coordinates from GNSS fixes by an azimuthal
// equidistant projection about that origin on a sphere (its MapProjection); the same projection about the datum
// puts the origin in the world frame consistently with PX4's own numbers.

#pragma once

#include <array>
#include <cmath>

namespace castor_localization {

constexpr double kEarthRadius = 6371000.0;  // m, PX4's CONSTANTS_RADIUS_OF_EARTH

struct Geodetic {
  double latitude{0.0};   // deg
  double longitude{0.0};  // deg
  double altitude{0.0};   // m AMSL
};

// East, north, up of `point` from `datum` [m].
inline std::array<double, 3> enu_from_datum(const Geodetic &datum, const Geodetic &point) {
  const double rad = M_PI / 180.0;
  const double lat0 = datum.latitude * rad, lon0 = datum.longitude * rad;
  const double lat = point.latitude * rad, lon = point.longitude * rad;
  const double cos_d_lon = std::cos(lon - lon0);
  const double arg = std::fmax(-1.0, std::fmin(1.0, std::sin(lat0) * std::sin(lat) +
                                                     std::cos(lat0) * std::cos(lat) * cos_d_lon));
  const double c = std::acos(arg);
  const double k = std::fabs(c) > 0.0 ? c / std::sin(c) : 1.0;
  const double north = k * (std::cos(lat0) * std::sin(lat) - std::sin(lat0) * std::cos(lat) * cos_d_lon) * kEarthRadius;
  const double east = k * std::cos(lat) * std::sin(lon - lon0) * kEarthRadius;
  return {east, north, point.altitude - datum.altitude};
}

}  // namespace castor_localization
