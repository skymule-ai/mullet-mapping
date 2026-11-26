// map_localizer: see include/map_localizer/map_localizer.hpp and Documentation/design.rst.
// Unwritten functions throw, so nothing silently returns a made-up pose.
#include "map_localizer/map_localizer.hpp"

#include <stdexcept>
#include <vector>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <small_gicp/registration/registration_helper.hpp>

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
  // TODO(step 5): T_map_odom = T_map_body · T_odom_body⁻¹
  (void)T_map_body; (void)T_odom_body;
  todo("compose_correction");
}

Eigen::Isometry3d predict(const Eigen::Isometry3d & T_map_odom, const Eigen::Isometry3d & T_odom_body)
{
  // TODO(step 5): T_map_body = T_map_odom · T_odom_body
  (void)T_map_odom; (void)T_odom_body;
  todo("predict");
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

// ── The node ──────────────────────────────────────────────────────────────────

MapLocalizer::MapLocalizer(const rclcpp::NodeOptions & options)
: Node("map_localizer", options)
{
  declare_parameters();
  RCLCPP_INFO(get_logger(), "map_localizer started with map path: %s", params_.map_path.c_str());
  load_map();
  // TODO(step 4): subscriptions, relative names so the namespace applies:
  //   "fast_lio/cloud_registered_body" -> on_scan, "fast_lio/odometry" -> on_odometry (SensorDataQoS).
  // TODO(step 8): status publisher "map_localizer/status", the TF broadcaster.
  // TODO(steps 4–8): match_timer_ at match_rate -> on_match_timer, tf_timer_ at tf_rate -> publish_tf.
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
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (params_.map_path, *cloud) < 0) //* load the file
  {
    throw std::runtime_error("Couldn't read map file " + params_.map_path);
  }

  std::vector<Eigen::Vector4f> points;
  points.reserve(cloud->size());

  for (size_t i = 0; i < cloud->size(); ++i)
  {
    const auto & pt = cloud->at(i);
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
    points.emplace_back(Eigen::Vector4f(pt.x, pt.y, pt.z, 1.0f));
  }

  if (points.empty())
  {
    throw std::runtime_error("Map is empty after filtering invalid points");
  }

  std::tie(map_, map_kdtree_) = small_gicp::preprocess_points(points, params_.map_voxel); // preprocess and downsampling
  RCLCPP_INFO(get_logger(), "Loaded map with %zu points, downsampled to %zu points",
              points.size(), map_->size());
}

void MapLocalizer::on_scan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  // TODO(step 4): find the odometry with the same stamp (FAST-LIO publishes both per scan), keep
  //   {stamp, T_odom_body, points}; drop scans older than submap_window_s.
  (void)msg;
}

void MapLocalizer::on_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  // TODO(step 4): append to odometry_, keep a few seconds.
  (void)msg;
}

void MapLocalizer::stack_submap()
{
  // TODO(step 4): transform each kept scan by its T_odom_body into odom_frame, merge, downsample
  //   (submap_voxel). The submap's reference pose is the newest scan's T_odom_body.
}

void MapLocalizer::align()
{
  // TODO(step 5): small_gicp GICP, source = submap (in body of the reference scan), target = map,
  //   initial guess = predict(T_map_odom_, T_odom_body). Fill MatchQuality from the result
  //   (inliers -> overlap, error -> rmse, Hessian translation block -> min_eig).
}

void MapLocalizer::initialize()
{
  // TODO(step 7): for k in [0, init_yaw_steps): guess = init_pose rotated by k·360/steps about z;
  //   align; keep the best that passes the gate (without the step check). Pass -> TRACKING.
}

void MapLocalizer::on_match_timer()
{
  // TODO(steps 4–7): state_ == INIT -> initialize(); else stack_submap(), align(), step_between,
  //   passes_gate, on pass T_map_odom_ = compose_correction(...), state_ = next_state(...),
  //   publish_status(...).
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
