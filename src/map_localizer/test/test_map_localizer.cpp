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

using map_localizer::Scan;
using map_localizer::stack_points;

// Expected values worked out by hand, not with the same formula as the code.
TEST(StackPoints, IntoNewestBodyFrame)
{
  Scan older;   // body 1 m along odom x, not rotated
  older.T_odom_body = Eigen::Isometry3d(Eigen::Translation3d(1, 0, 0));
  older.points = {Eigen::Vector4d(1, 0, 0, 1)};  // at odom (2, 0, 0)

  Scan newest;  // body at odom (0, 2, 0), yawed +90°: its x axis is odom +y, its y axis odom -x
  newest.T_odom_body =
    Eigen::Translation3d(0, 2, 0) * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
  newest.points = {Eigen::Vector4d(1, 0, 0, 1)};

  const std::vector<Eigen::Vector4d> points = stack_points({older, newest});

  ASSERT_EQ(points.size(), 2u);
  // odom (2, 0) seen from (0, 2) facing +y: 2 m behind, 2 m to the right
  EXPECT_TRUE(points[0].isApprox(Eigen::Vector4d(-2, -2, 0, 1))) << points[0].transpose();
  // the newest scan is the reference: its points do not move
  EXPECT_TRUE(points[1].isApprox(Eigen::Vector4d(1, 0, 0, 1))) << points[1].transpose();
}

TEST(StackPoints, NoScansNoPoints) { EXPECT_TRUE(stack_points({}).empty()); }
