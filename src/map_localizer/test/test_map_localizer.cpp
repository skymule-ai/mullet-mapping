// Unit tests for the pure math in map_localizer.hpp: no ROS, no files, no small_gicp calls.
// Write each test before its function (it fails against the throwing stub), then make it pass.
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "map_localizer/map_localizer.hpp"

using map_localizer::Params;

using map_localizer::compose_correction;
using map_localizer::predict;

// Rotations about tilted axes do not commute, so a product in the wrong order cannot round-trip.
TEST(Correction, RoundTripsThroughPredict)
{
  const Eigen::Isometry3d T_map_odom = Eigen::Translation3d(1, 2, 3) *
    Eigen::AngleAxisd(0.5, Eigen::Vector3d(1, 1, 0).normalized());
  const Eigen::Isometry3d T_odom_body = Eigen::Translation3d(-4, 0.5, 2) *
    Eigen::AngleAxisd(1.2, Eigen::Vector3d(0, 1, 1).normalized());

  const Eigen::Isometry3d back = compose_correction(predict(T_map_odom, T_odom_body), T_odom_body);

  EXPECT_TRUE(back.isApprox(T_map_odom)) << back.matrix();
}

TEST(Correction, IdentityOdometry)
{
  const Eigen::Isometry3d T_map_body = Eigen::Translation3d(5, -1, 0.5) *
    Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ());

  EXPECT_TRUE(compose_correction(T_map_body, Eigen::Isometry3d::Identity()).isApprox(T_map_body));
}

// The round trip passes for predict = T_odom_body · T_map_odom too; this one does not.
TEST(Correction, PredictByHand)
{
  // camera_init at map (10, 0, 0), yawed +90°: odom +x is map +y
  const Eigen::Isometry3d T_map_odom =
    Eigen::Translation3d(10, 0, 0) * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
  // body 1 m along odom x
  const Eigen::Isometry3d T_odom_body(Eigen::Translation3d(1, 0, 0));

  const Eigen::Isometry3d T_map_body = predict(T_map_odom, T_odom_body);

  // 1 m along map +y from (10, 0, 0); the wrong order gives (11, 0, 0)
  EXPECT_TRUE(T_map_body.translation().isApprox(Eigen::Vector3d(10, 1, 0)))
    << T_map_body.translation().transpose();
}

using map_localizer::MatchQuality;
using map_localizer::passes_gate;
using map_localizer::Step;
using map_localizer::step_between;

// previous is rotated and away from the origin, so candidate · previous⁻¹ (the wrong order) gives
// another translation; radians instead of degrees gives 0.035.
TEST(Step, MetersAndDegrees)
{
  const Eigen::Isometry3d previous = Eigen::Translation3d(3, -2, 1) *
    Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized());
  // 0.2 m (3-4-5 triangle) and 2 deg, about a tilted axis, in previous's frame
  const Eigen::Isometry3d candidate = previous * Eigen::Translation3d(0.12, -0.16, 0) *
    Eigen::AngleAxisd(2.0 * M_PI / 180.0, Eigen::Vector3d(0, 1, 1).normalized());

  const Step step = step_between(previous, candidate);

  EXPECT_NEAR(step.meters, 0.2, 1e-9);
  EXPECT_NEAR(step.degrees, 2.0, 1e-9);
}

TEST(Step, SameCorrectionIsZero)
{
  const Eigen::Isometry3d T = Eigen::Translation3d(3, -2, 1) *
    Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ());
  const Step step = step_between(T, T);
  EXPECT_NEAR(step.meters, 0.0, 1e-9);
  EXPECT_NEAR(step.degrees, 0.0, 1e-6);
}

namespace
{
Params gate_params()
{
  Params p{};
  p.min_overlap = 0.5;
  p.max_rmse = 0.2;
  p.max_step_m = 0.3;
  p.max_step_deg = 3.0;
  p.min_hessian_eig = 100.0;
  return p;
}
const MatchQuality kGood{0.8, 0.05, 500.0, 10, true};  // overlap, rmse, min_eig, iterations, conv.
const Step kSmall{0.05, 0.5};                            // m, deg
}  // namespace

TEST(Gate, EachCheckCanFailAlone)
{
  const Params p = gate_params();
  ASSERT_TRUE(passes_gate(kGood, kSmall, p));

  // Each case breaks exactly one check of the baseline.
  auto fails = [&](auto && change) {
      MatchQuality q = kGood;
      Step s = kSmall;
      change(q, s);
      return !passes_gate(q, s, p);
    };
  EXPECT_TRUE(fails([](MatchQuality & q, Step &) {q.overlap = 0.4;})) << "overlap";
  EXPECT_TRUE(fails([](MatchQuality & q, Step &) {q.rmse = 0.3;})) << "rmse";
  EXPECT_TRUE(fails([](MatchQuality & q, Step &) {q.converged = false;})) << "converged";
  EXPECT_TRUE(fails([](MatchQuality & q, Step &) {q.min_eig = 50.0;})) << "min_eig";
  EXPECT_TRUE(fails([](MatchQuality &, Step & s) {s.meters = 0.4;})) << "step m";
  EXPECT_TRUE(fails([](MatchQuality &, Step & s) {s.degrees = 4.0;})) << "step deg";
}

// Thresholds are inclusive: exactly on every limit still passes.
TEST(Gate, OnThresholdPasses)
{
  const Params p = gate_params();
  const MatchQuality q{p.min_overlap, p.max_rmse, p.min_hessian_eig, 10, true};
  EXPECT_TRUE(passes_gate(q, Step{p.max_step_m, p.max_step_deg}, p));
}

// A diverged GICP can give NaN. Every comparison with NaN is false, so the gate must fail; written
// as !(x > limit) it would pass.
TEST(Gate, NaNFails)
{
  const Params p = gate_params();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  MatchQuality q = kGood;
  q.rmse = nan;
  EXPECT_FALSE(passes_gate(q, kSmall, p)) << "rmse";
  q = kGood;
  q.overlap = nan;
  EXPECT_FALSE(passes_gate(q, kSmall, p)) << "overlap";
  q = kGood;
  q.min_eig = nan;
  EXPECT_FALSE(passes_gate(q, kSmall, p)) << "min_eig";
  EXPECT_FALSE(passes_gate(kGood, Step{nan, 0.5}, p)) << "step m";
  EXPECT_FALSE(passes_gate(kGood, Step{0.05, nan}, p)) << "step deg";
}

using map_localizer::next_state;
using map_localizer::State;

namespace
{
Params state_params()
{
  Params p{};
  p.lost_after_s = 5.0;
  p.max_rejected_steps = 3;
  return p;
}
}  // namespace

// Arguments: current, passed, seconds since the last pass, large steps in a row.
TEST(States, Transitions)
{
  const Params p = state_params();

  // INIT and LOST are left only by initialize(); a pass of the normal loop does not move LOST.
  EXPECT_EQ(next_state(State::INIT, false, 0.0, 0, p), State::INIT);
  EXPECT_EQ(next_state(State::INIT, true, 0.0, 0, p), State::TRACKING);
  EXPECT_EQ(next_state(State::LOST, true, 0.0, 0, p), State::LOST);
  EXPECT_EQ(next_state(State::LOST, false, 10.0, 5, p), State::LOST);

  EXPECT_EQ(next_state(State::TRACKING, true, 0.0, 0, p), State::TRACKING);
  EXPECT_EQ(next_state(State::TRACKING, false, 1.0, 0, p), State::HOLDING);
  EXPECT_EQ(next_state(State::HOLDING, true, 3.0, 2, p), State::TRACKING);
  EXPECT_EQ(next_state(State::HOLDING, false, 4.9, 2, p), State::HOLDING);
}

// A failure that already meets a LOST condition skips HOLDING.
TEST(States, LostConditions)
{
  const Params p = state_params();
  for (State s : {State::TRACKING, State::HOLDING}) {
    EXPECT_EQ(next_state(s, false, 1.0, 3, p), State::LOST) << "large steps";
    EXPECT_EQ(next_state(s, false, 5.1, 0, p), State::LOST) << "time";
  }
}

// lost_after_s is exclusive (exactly on it still holds); max_rejected_steps is inclusive.
TEST(States, Limits)
{
  const Params p = state_params();
  EXPECT_EQ(next_state(State::HOLDING, false, 5.0, 0, p), State::HOLDING);
  EXPECT_EQ(next_state(State::HOLDING, false, 1.0, 2, p), State::HOLDING);
  EXPECT_EQ(next_state(State::HOLDING, false, 1.0, 3, p), State::LOST);
}

using map_localizer::status_level;
using diagnostic_msgs::msg::DiagnosticStatus;

// Preflight arms only on OK, so only TRACKING may give it; LOST must stand out from HOLDING.
// Cast to int: gtest prints a uint8_t as a character.
TEST(Status, LevelPerState)
{
  EXPECT_EQ(static_cast<int>(status_level(State::TRACKING)), DiagnosticStatus::OK);
  EXPECT_EQ(static_cast<int>(status_level(State::INIT)), DiagnosticStatus::WARN);
  EXPECT_EQ(static_cast<int>(status_level(State::HOLDING)), DiagnosticStatus::WARN);
  EXPECT_EQ(static_cast<int>(status_level(State::LOST)), DiagnosticStatus::ERROR);
}

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
