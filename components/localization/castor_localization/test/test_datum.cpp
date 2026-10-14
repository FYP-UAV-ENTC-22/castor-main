#include <gtest/gtest.h>

#include "castor_localization/datum.hpp"

using castor_localization::enu_from_datum;
using castor_localization::Geodetic;
using castor_localization::kEarthRadius;

TEST(Datum, ThePointItselfIsTheOrigin) {
  const Geodetic d{38.736832, -9.137977, 90.0};
  const auto p = enu_from_datum(d, d);
  EXPECT_DOUBLE_EQ(p[0], 0.0);
  EXPECT_DOUBLE_EQ(p[1], 0.0);
  EXPECT_DOUBLE_EQ(p[2], 0.0);
}

TEST(Datum, NorthEastAndUpInMetres) {
  const Geodetic d{38.736832, -9.137977, 90.0};
  const double rad = M_PI / 180.0;
  // 1.5 m north: latitude grows by 1.5 / R
  auto p = enu_from_datum(d, {d.latitude + 1.5 / kEarthRadius / rad, d.longitude, 90.0});
  EXPECT_NEAR(p[0], 0.0, 1e-6);
  EXPECT_NEAR(p[1], 1.5, 1e-6);
  // 2 m east: longitude grows by 2 / (R cos lat); 0.2 m up
  p = enu_from_datum(d, {d.latitude, d.longitude + 2.0 / (kEarthRadius * std::cos(d.latitude * rad)) / rad, 90.2});
  EXPECT_NEAR(p[0], 2.0, 1e-6);
  EXPECT_NEAR(p[1], 0.0, 1e-6);
  EXPECT_NEAR(p[2], 0.2, 1e-9);
}

TEST(Datum, SouthWestIsNegative) {
  const Geodetic d{-33.0, 151.0, 10.0};
  const auto p = enu_from_datum(d, {d.latitude - 1e-5, d.longitude - 1e-5, 4.0});
  EXPECT_LT(p[0], 0.0);
  EXPECT_LT(p[1], 0.0);
  EXPECT_NEAR(p[1], -1e-5 * M_PI / 180.0 * kEarthRadius, 1e-4);
  EXPECT_NEAR(p[2], -6.0, 1e-12);
}
