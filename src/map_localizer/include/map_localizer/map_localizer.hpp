// map_localizer: FAST-LIO2 odometry localized in a prior map. Spec: Documentation/design.rst.
//
// Two layers, so the math is testable without ROS:
//   - free functions (pure: poses and numbers in, poses and numbers out), tested in test/;
//   - the MapLocalizer node, which owns subscriptions, timers, state, and calls them.
// TODO(step N) markers match the steps in Documentation/status.rst.
#pragma once

#include <deque>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

// TODO(step 2): small_gicp headers, once it is installed in the image:
//   <small_gicp/points/point_cloud.hpp>, <small_gicp/ann/kdtree_omp.hpp>,
//   <small_gicp/registration/registration_result.hpp>

namespace map_localizer
{

enum class State { INIT, TRACKING, HOLDING, LOST };

// One field per entry in config/map_localizer.yaml, same names and units.
struct Params
{
  std::string map_path;
  std::string map_frame;
  std::string odom_frame;
  std::string body_frame;
  double map_voxel;         // m
  double submap_window_s;   // s
  double submap_voxel;      // m
  double match_rate;        // Hz
  double tf_rate;           // Hz
  double max_corr_dist;     // m
  double min_overlap;       // fraction
  double max_rmse;          // m
  double max_step_m;        // m
  double max_step_deg;      // deg
  double min_hessian_eig;   // GICP Hessian, translation block
  Eigen::Vector4d init_pose;  // x, y, z (m), yaw (deg)
  int init_yaw_steps;
  double lost_after_s;      // s
  int max_rejected_steps;
};

// What one alignment produced, reduced to what the gate needs.
struct MatchQuality
{
  double overlap;        // fraction of submap points within max_corr_dist of the map
  double rmse;           // m, over those points
  double min_eig;        // smallest eigenvalue of the translation block of the GICP Hessian
};

// ── Pure math (step 5–7), tested in test/test_map_localizer.cpp ──────────────

// T_map_odom = T_map_body · T_odom_body⁻¹. Both body poses at the same stamp. (step 5)
Eigen::Isometry3d compose_correction(
  const Eigen::Isometry3d & T_map_body, const Eigen::Isometry3d & T_odom_body);

// T_map_body predicted from the last good correction and the current odometry. (step 5)
Eigen::Isometry3d predict(
  const Eigen::Isometry3d & T_map_odom, const Eigen::Isometry3d & T_odom_body);

// Translation (m) and rotation angle (deg) between two corrections. (step 6)
struct Step { double meters; double degrees; };
Step step_between(const Eigen::Isometry3d & previous, const Eigen::Isometry3d & candidate);

// True only if every check in Documentation/design.rst "Gate" holds. (step 6)
bool passes_gate(const MatchQuality & quality, const Step & step, const Params & params);

// Next state from the current one and this match's outcome. (step 7)
State next_state(
  State current, bool passed, double seconds_since_last_pass, int rejected_steps_in_a_row,
  const Params & params);

// ── The node ──────────────────────────────────────────────────────────────────

class MapLocalizer : public rclcpp::Node
{
public:
  explicit MapLocalizer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Declare and read every parameter into params_. (step 2)
  void declare_parameters();

  // Read map.pcd, downsample to map_voxel, build the GICP target (covariances, KD-tree). (step 3)
  void load_map();

  // Cloud callback: pair the scan with the odometry of the same stamp and keep it in scans_. (step 4)
  void on_scan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);

  // Odometry callback: keep a short history to look up T_odom_body at a scan's stamp. (step 4)
  void on_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);

  // The last submap_window_s of scans, in odom_frame, downsampled to submap_voxel. (step 4)
  // TODO(step 4): return type: a small_gicp point cloud.
  void stack_submap();

  // GICP of the submap against the map from an initial guess; fills the quality. (step 5)
  // TODO(step 5): signature with the submap and the guess; returns T_map_body + MatchQuality.
  void align();

  // INIT: try init_yaw_steps yaw hypotheses around init_pose, keep the best. (step 7)
  void initialize();

  // The 1 Hz loop: stack, predict, align, gate, update state and correction. (steps 4–7)
  void on_match_timer();

  // Re-send the last good map -> odom at tf_rate. (step 8)
  void publish_tf();

  // State, overlap, RMSE, step, age of the last pass. (step 8)
  void publish_status(const MatchQuality & quality, const Step & step);

  Params params_;
  State state_{State::INIT};
  Eigen::Isometry3d T_map_odom_{Eigen::Isometry3d::Identity()};  // last good correction
  rclcpp::Time last_pass_;
  int rejected_steps_in_a_row_{0};

  // TODO(step 3): the map (small_gicp point cloud + KD-tree).
  // TODO(step 4): scans_ (each scan with its T_odom_body and stamp) and the odometry history.
  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> odometry_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr match_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
};

}  // namespace map_localizer
