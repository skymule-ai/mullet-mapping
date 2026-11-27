// map_localizer: FAST-LIO2 odometry localized in a prior map. Spec: Documentation/design.rst.
//
// Two layers, so the math is testable without ROS:
//   - free functions (pure: poses and numbers in, poses and numbers out), tested in test/;
//   - the MapLocalizer node, which owns subscriptions, timers, state, and calls them.
#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/registration_result.hpp>

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
  uint32_t sync_queue_size; // scans or odometry held while waiting for their pair
  double max_corr_dist;     // m
  double min_overlap;       // fraction
  double max_rmse;          // m
  double max_step_m;        // m
  double max_step_deg;      // deg
  double min_hessian_eig;   // GICP Hessian, translation block
  Eigen::Vector4d init_pose;  // x, y, z (m), yaw (deg)
  int init_yaw_steps;
  std::vector<double> search_corr_dists;  // m, coarse search stages before max_corr_dist
  double lost_after_s;      // s
  int max_rejected_steps;
};

// What one alignment produced, reduced to what the gate needs.
struct MatchQuality
{
  double overlap;        // fraction of submap points within max_corr_dist of the map
  double rmse;           // m, over those points
  double min_eig;        // smallest eigenvalue of the translation block of the GICP Hessian
  size_t iterations;     // GICP iterations run
  bool converged;        // GICP converged within max_iterations
};

// One FAST-LIO2 scan with the odometry of the same stamp.
struct Scan
{
  rclcpp::Time stamp;
  Eigen::Isometry3d T_odom_body;        // camera_init -> body at stamp
  std::vector<Eigen::Vector4d> points;  // in body, w = 1
};

// The result of aligning a submap to the map, including the estimated pose and quality metrics.
struct Match
{
  Eigen::Isometry3d T_map_body;
  MatchQuality quality;
};

// ── Pure math, tested in test/test_map_localizer.cpp ──────────────────────────

// Every point of every scan in the newest scan's body frame: T_ref⁻¹ · T_odom_body · p, with
// T_ref the newest scan's T_odom_body. Empty for no scans.
std::vector<Eigen::Vector4d> stack_points(const std::deque<Scan> & scans);

// T_map_odom = T_map_body · T_odom_body⁻¹. Both body poses at the same stamp.
Eigen::Isometry3d compose_correction(
  const Eigen::Isometry3d & T_map_body, const Eigen::Isometry3d & T_odom_body);

// T_map_body predicted from the last good correction and the current odometry.
Eigen::Isometry3d predict(
  const Eigen::Isometry3d & T_map_odom, const Eigen::Isometry3d & T_odom_body);

// Translation (m) and rotation angle (deg) between two corrections.
struct Step { double meters; double degrees; };
Step step_between(const Eigen::Isometry3d & previous, const Eigen::Isometry3d & candidate);

// True only if every check in Documentation/design.rst "Gate" holds.
bool passes_gate(const MatchQuality & quality, const Step & step, const Params & params);

// Next state from the current one and this match's outcome.
State next_state(
  State current, bool passed, double seconds_since_last_pass, int rejected_steps_in_a_row,
  const Params & params);

// DiagnosticStatus level of a state: OK (TRACKING), WARN (INIT, HOLDING), ERROR (LOST).
uint8_t status_level(State state);

// ── The node ──────────────────────────────────────────────────────────────────

class MapLocalizer : public rclcpp::Node
{
public:
  explicit MapLocalizer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Declare and read every parameter into params_.
  void declare_parameters();

  // Read map.pcd, downsample to map_voxel, build the GICP target (covariances, KD-tree).
  void load_map();

  // A scan and its odometry, same stamp (the synchronizer pairs them): keep the last
  // submap_window_s in scans_. Drops scans in unexpected frames; clears on time going back.
  void on_scan(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom);

  // The last submap_window_s of scans in the newest scan's body frame, downsampled to
  // submap_voxel, with covariances: the GICP source. nullptr until a scan has arrived.
  small_gicp::PointCloud::Ptr stack_submap();

  // GICP of the submap against the map from an initial guess, pairing points within
  // max_correspondence_distance; fills the quality, always measured at max_corr_dist.
  Match align(
    const small_gicp::PointCloud & submap, const Eigen::Isometry3d & guess,
    double max_correspondence_distance) const;

  // INIT and LOST: init_yaw_steps yaw seeds about the predicted body, each aligned coarse to fine
  // (search_corr_dists, then max_corr_dist); keep the best. Marks the newest scan as matched.
  void initialize();

  // The 1 Hz loop: stack, predict, align, gate, update state and correction.
  void on_match_timer();

  // Re-send the last good map -> odom at tf_rate.
  void publish_tf();

  // Level from the state, the reason, quality, step, age of the last pass: interfaces.rst.
  void publish_status(
    const MatchQuality & quality, const Step & step, const std::string & message_reason);

  Params params_;
  State state_{State::INIT};
  Eigen::Isometry3d T_map_odom_{Eigen::Isometry3d::Identity()};  // last good correction
  rclcpp::Time last_pass_{0, 0, RCL_ROS_TIME};
  int rejected_steps_in_a_row_{0};

  small_gicp::PointCloud::Ptr map_;
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> map_kdtree_;
  std::deque<Scan> scans_;  // oldest first, the last submap_window_s

  // sync_ after the subscribers: it is destroyed first, while they still exist.
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> scan_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_sub_;
  std::unique_ptr<message_filters::TimeSynchronizer<
      sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>> sync_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr match_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::Time last_matched_stamp_{0, 0, RCL_ROS_TIME};  // ROS time, like message stamps
};

}  // namespace map_localizer
