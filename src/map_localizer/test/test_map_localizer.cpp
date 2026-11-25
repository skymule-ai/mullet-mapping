// Unit tests for the pure math in map_localizer.hpp: no ROS, no files, no small_gicp calls.
// Write each test before its function (it fails against the throwing stub), then make it pass.
#include <gtest/gtest.h>

#include "map_localizer/map_localizer.hpp"

using map_localizer::Params;

// TODO(step 5): with a known T_map_odom and any T_odom_body, compose_correction(predict(T_map_odom,
//   T_odom_body), T_odom_body) gives back T_map_odom (isApprox).
TEST(Correction, RoundTripsThroughPredict) { GTEST_SKIP() << "TODO(step 5)"; }

// TODO(step 5): identity odometry -> the correction equals the matched map -> body pose.
TEST(Correction, IdentityOdometry) { GTEST_SKIP() << "TODO(step 5)"; }

// TODO(step 6): 0.2 m and 2 deg apart -> step_between reports those numbers.
TEST(Step, MetersAndDegrees) { GTEST_SKIP() << "TODO(step 6)"; }

// TODO(step 6): a match that meets every threshold passes; each single violation
//   (overlap, rmse, step m, step deg, min_eig) fails on its own.
TEST(Gate, EachCheckCanFailAlone) { GTEST_SKIP() << "TODO(step 6)"; }

// TODO(step 7): TRACKING + fail -> HOLDING; HOLDING + pass -> TRACKING;
//   no pass for lost_after_s -> LOST; LOST + pass outside INIT stays LOST.
TEST(States, Transitions) { GTEST_SKIP() << "TODO(step 7)"; }
