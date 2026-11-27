// map_localizer: see include/map_localizer/map_localizer.hpp and Documentation/design.rst.
// Unwritten functions throw, so nothing silently returns a made-up pose.
#include "map_localizer/map_localizer.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <tuple>
#include <utility>

#include <Eigen/Eigenvalues>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <small_gicp/registration/registration_helper.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace map_localizer
{

namespace
{
[[noreturn]] void todo(const char * what) { throw std::logic_error(std::string("TODO: ") + what); }
}  // namespace

// ── Pure math ─────────────────────────────────────────────────────────────────

Eigen::Isometry3d compose_correction(
  const Eigen::Isometry3d & T_map_body, const Eigen::Isometry3d & T_odom_body)
{
  return T_map_body * T_odom_body.inverse();
}

Eigen::Isometry3d predict(const Eigen::Isometry3d & T_map_odom, const Eigen::Isometry3d & T_odom_body)
{
  return T_map_odom * T_odom_body;
}

Step step_between(const Eigen::Isometry3d & previous, const Eigen::Isometry3d & candidate)
{
  // TODO(step 6): Δ = previous⁻¹ · candidate; meters = |Δ.translation()|,
  //   degrees = angle of Δ.rotation() (Eigen::AngleAxisd), in degrees.
  (void)previous; (void)candidate;
  todo("step_between");
}

bool passes_gate(const MatchQuality & quality, const Step & step, const Params & params)
{
  // TODO(step 6): overlap >= min_overlap, rmse <= max_rmse, step within max_step_m and max_step_deg,
  //   min_eig >= min_hessian_eig. All must hold.
  (void)quality; (void)step; (void)params;
  todo("passes_gate");
}

State next_state(
  State current, bool passed, double seconds_since_last_pass, int rejected_steps_in_a_row,
  const Params & params)
{
  // TODO(step 7): the transitions in Documentation/design.rst "States". From LOST, only an INIT search
  //   brings it back.
  (void)current; (void)passed; (void)seconds_since_last_pass; (void)rejected_steps_in_a_row;
  (void)params;
  todo("next_state");
}

std::vector<Eigen::Vector4d> stack_points(const std::deque<Scan> & scans)
{
  if (scans.empty()) return {};

  size_t total = 0;
  for (const auto & scan : scans) { total += scan.points.size(); }
  std::vector<Eigen::Vector4d> points;
  points.reserve(total);

  // T_ref_odom = T_odom_body(newest)⁻¹; each scan goes through T_ref_odom · T_odom_body(scan).
  const Eigen::Isometry3d T_ref_odom = scans.back().T_odom_body.inverse();
  for (const auto & scan : scans) {
    const Eigen::Matrix4d T_ref_scan = (T_ref_odom * scan.T_odom_body).matrix();
    for (const auto & point : scan.points) { points.push_back(T_ref_scan * point); }
  }
  return points;
}

// ── The node ──────────────────────────────────────────────────────────────────

MapLocalizer::MapLocalizer(const rclcpp::NodeOptions & options)
: Node("map_localizer", options)
{
  declare_parameters();
  RCLCPP_INFO(get_logger(), "map_localizer started with map path: %s", params_.map_path.c_str());
  load_map();

  // Each scan with the odometry of the same stamp (FAST-LIO2 stamps both with lidar_end_time).
  // Relative names, so the /$DRONE_ID namespace applies.
  scan_sub_.subscribe(this, "fast_lio/cloud_registered_body", rmw_qos_profile_sensor_data);
  odometry_sub_.subscribe(this, "fast_lio/odometry", rmw_qos_profile_sensor_data);
  sync_ = std::make_unique<message_filters::TimeSynchronizer<
        sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>(
    scan_sub_, odometry_sub_, params_.sync_queue_size);
  sync_->registerCallback(std::bind(
      &MapLocalizer::on_scan, this, std::placeholders::_1, std::placeholders::_2));

  // The node's clock, so a bag replayed with use_sim_time matches once per second of bag time.
  match_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / params_.match_rate),
    std::bind(&MapLocalizer::on_match_timer, this));

  // TODO(step 8): status publisher "map_localizer/status", the TF broadcaster,
  //   tf_timer_ at tf_rate -> publish_tf.
}

void MapLocalizer::declare_parameters()
{
  params_.map_path = declare_parameter<std::string>("map_path", "/map/map.pcd");
  params_.map_frame = declare_parameter<std::string>("map_frame", "map");
  params_.odom_frame = declare_parameter<std::string>("odom_frame", "camera_init");
  params_.body_frame = declare_parameter<std::string>("body_frame", "body");
  params_.map_voxel = declare_parameter<double>("map_voxel", 0.2);
  params_.submap_window_s = declare_parameter<double>("submap_window_s", 1.0);
  params_.submap_voxel = declare_parameter<double>("submap_voxel", 0.15);
  const double match_rate = declare_parameter<double>("match_rate", 1.0);
  if (match_rate <= 0.0) {
    throw std::invalid_argument("Parameter 'match_rate' must be positive");
  }
  params_.match_rate = match_rate;
  const double tf_rate = declare_parameter<double>("tf_rate", 10.0);
  if (tf_rate <= 0.0) {
    throw std::invalid_argument("Parameter 'tf_rate' must be positive");
  }
  params_.tf_rate = tf_rate;
  const int64_t sync_queue_size = declare_parameter<int64_t>("sync_queue_size", 10);
  if (sync_queue_size < 1 || sync_queue_size > 100) {
    throw std::invalid_argument("Parameter 'sync_queue_size' must be between 1 and 100");
  }
  params_.sync_queue_size = static_cast<uint32_t>(sync_queue_size);
  params_.max_corr_dist = declare_parameter<double>("max_corr_dist", 0.5);
  params_.min_overlap = declare_parameter<double>("min_overlap", 0.5);
  params_.max_rmse = declare_parameter<double>("max_rmse", 0.2);
  params_.max_step_m = declare_parameter<double>("max_step_m", 0.3);
  params_.max_step_deg = declare_parameter<double>("max_step_deg", 3.0);
  params_.min_hessian_eig = declare_parameter<double>("min_hessian_eig", 0.0);

  const std::vector<double> init_pose = declare_parameter<std::vector<double>>(
    "init_pose", {0.0, 0.0, 0.0, 0.0});
  if (init_pose.size() != 4) {
    throw std::invalid_argument("Parameter 'init_pose' must contain x, y, z, and yaw");
  }
  params_.init_pose = Eigen::Vector4d(
    init_pose[0], init_pose[1], init_pose[2], init_pose[3]);
  params_.init_yaw_steps = declare_parameter<int>("init_yaw_steps", 12);
  params_.lost_after_s = declare_parameter<double>("lost_after_s", 5.0);
  params_.max_rejected_steps = declare_parameter<int>("max_rejected_steps", 3);
}

void MapLocalizer::load_map()
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(params_.map_path, cloud) < 0) {
    throw std::runtime_error("Couldn't read map file " + params_.map_path);
  }

  std::vector<Eigen::Vector4f> points;
  points.reserve(cloud.size());
  for (const auto & pt : cloud) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) { continue; }
    points.emplace_back(pt.x, pt.y, pt.z, 1.0f);
  }
  if (points.empty()) {
    throw std::runtime_error("Map is empty after filtering invalid points");
  }

  // Downsample, KD-tree, per-point covariances: the GICP target.
  std::tie(map_, map_kdtree_) = small_gicp::preprocess_points(points, params_.map_voxel);
  RCLCPP_INFO(
    get_logger(), "Loaded map with %zu points, downsampled to %zu points",
    points.size(), map_->size());
}

void MapLocalizer::on_scan(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom)
{
  // cloud_registered (already in camera_init) wired by mistake would otherwise stack silently.
  if (msg->header.frame_id != params_.body_frame ||
    odom->header.frame_id != params_.odom_frame ||
    odom->child_frame_id != params_.body_frame)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Frame mismatch: scan in '%s', odometry '%s' -> '%s'; expected '%s', '%s' -> '%s'",
      msg->header.frame_id.c_str(), odom->header.frame_id.c_str(), odom->child_frame_id.c_str(),
      params_.body_frame.c_str(), params_.odom_frame.c_str(), params_.body_frame.c_str());
    return;
  }

  Scan scan;
  scan.stamp = msg->header.stamp;
  tf2::fromMsg(odom->pose.pose, scan.T_odom_body);
  scan.points.reserve(msg->width * msg->height);
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) { continue; }
    scan.points.emplace_back(*iter_x, *iter_y, *iter_z, 1.0);
  }

  // A bag that loops or a sim time reset: the old scans belong to another trajectory.
  if (!scans_.empty() && scan.stamp < scans_.back().stamp) {
    RCLCPP_WARN(get_logger(), "Time went backwards: dropping %zu scans", scans_.size());
    scans_.clear();
  }
  scans_.push_back(std::move(scan));

  // Keep submap_window_s, measured in message stamps (never now(), so a replay behaves the same).
  const rclcpp::Time newest = scans_.back().stamp;
  while ((newest - scans_.front().stamp).seconds() > params_.submap_window_s) {
    scans_.pop_front();
  }
}

small_gicp::PointCloud::Ptr MapLocalizer::stack_submap()
{
  if (scans_.empty()) { return nullptr; }
  // Downsample, then covariances for GICP; the source does not need the KD-tree.
  return small_gicp::preprocess_points(stack_points(scans_), params_.submap_voxel).first;
}

Match MapLocalizer::align(
  const small_gicp::PointCloud & submap, const Eigen::Isometry3d & guess) const
{
  // GICP result -> T_map_body, Hessian -> min_eig. Overlap and RMSE from our own nearest-neighbour
  //   pass at the final pose: result.error is the covariance-weighted cost, not meters.
  small_gicp::RegistrationSetting s;
  s.type = small_gicp::RegistrationSetting::RegistrationType::GICP;
  s.max_correspondence_distance = params_.max_corr_dist;

  small_gicp::RegistrationResult result = small_gicp::align(*map_, submap, *map_kdtree_, guess, s);

  Match match;
  match.T_map_body = result.T_target_source;

  size_t count = 0;    // submap points within max_corr_dist of the map
  double sum_sq = 0.0;  // their squared distances

  for (const auto & point : submap.points) {
    size_t idx; double sq;
    if (
      small_gicp::traits::nearest_neighbor_search(
        *map_kdtree_, match.T_map_body * point, &idx, &sq) == 1 &&
      sq <= params_.max_corr_dist * params_.max_corr_dist)
    {
      count += 1;
      sum_sq += sq;
    }
  }
  match.quality.overlap = count / static_cast<double>(submap.size());
  match.quality.rmse = (count > 0) ? std::sqrt(sum_sq / count) : 0.0;
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(
    result.H.bottomRightCorner<3, 3>());
  match.quality.min_eig = eigensolver.eigenvalues()(0);
  match.quality.iterations = result.iterations + 1;  // small_gicp stores the last 0-based index
  match.quality.converged = result.converged;
  return match;
}

void MapLocalizer::initialize()
{
  // TODO(step 7): for k in [0, init_yaw_steps): guess = init_pose rotated by k·360/steps about z;
  //   align; keep the best that passes the gate (without the step check). Pass -> TRACKING.
}

void MapLocalizer::on_match_timer()
{
  // TODO(steps 6–7): state_ == INIT -> initialize(); else align(), step_between, passes_gate,
  //   state_ = next_state(...), publish_status(...).
  const auto start = std::chrono::steady_clock::now();
  const small_gicp::PointCloud::Ptr submap = stack_submap();
  if (!submap || submap->empty()) { return; }

  // The submap is in the newest scan's body frame: predict from that scan's odometry
  const auto T_odom_body = scans_.back().T_odom_body;
  const Eigen::Isometry3d guess = predict(T_map_odom_, T_odom_body);
  const Match match = align(*submap, guess);
  const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start;

  if (match.quality.converged) {
    // TODO(step 6): only on a pass
    T_map_odom_ = compose_correction(match.T_map_body, T_odom_body);
  }

  const Eigen::Matrix3d R = match.T_map_body.rotation();
  const Eigen::Vector3d t = match.T_map_body.translation();
  const double yaw_deg = std::atan2(R(1, 0), R(0, 0)) * 180.0 / M_PI;
  const MatchQuality & q = match.quality;
  RCLCPP_INFO(
    get_logger(),
    "Submap: %zu scans over %.2f s, %zu points -- [T_map_body] x: %.2f y: %.2f z: %.2f, "
    "yaw: %.1f deg -- overlap: %.2f, rmse: %.2f, min eig: %.2f, iterations: %zu, "
    "converged: %d, elapsed: %.3f s",
    scans_.size(), (scans_.back().stamp - scans_.front().stamp).seconds(), submap->size(),
    t.x(), t.y(), t.z(), yaw_deg, q.overlap, q.rmse,
    q.min_eig, q.iterations, static_cast<int>(q.converged), elapsed.count());
}

void MapLocalizer::publish_tf()
{
  // TODO(step 8): T_map_odom_ as map_frame -> odom_frame, stamped now. Nothing before the first pass.
}

void MapLocalizer::publish_status(const MatchQuality & quality, const Step & step)
{
  // TODO(step 8): level OK (TRACKING), WARN (HOLDING, INIT), ERROR (LOST); values as key/value pairs.
  (void)quality; (void)step;
}

}  // namespace map_localizer
