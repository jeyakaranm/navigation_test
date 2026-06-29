#include <docking_controller/docking_node.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace docking_controller
{

DockingNode::DockingNode() : rclcpp::Node("docking_node")
{
  const DockingController::DockingControllerParams params = declareParameters();
  docking_controller_ = std::make_unique<DockingController>(params, get_logger(), now());

  rclcpp::QoS scan_qos = rclcpp::SensorDataQoS();
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(scan_topic_, scan_qos,
                                                               std::bind(&DockingNode::scanCallback, this, _1));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  docking_status_pub_ = create_publisher<std_msgs::msg::String>("docking/status", 10);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("docking/markers", 10);

  start_dock_srv_ =
      create_service<std_srvs::srv::Trigger>("docking/start", std::bind(&DockingNode::handleStart, this, _1, _2));
  cancel_dock_srv_ =
      create_service<std_srvs::srv::Trigger>("docking/cancel", std::bind(&DockingNode::handleCancel, this, _1, _2));

  const auto period = std::chrono::duration<double>(1.0 / control_rate_);
  control_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                     std::bind(&DockingNode::controlLoop, this));

  RCLCPP_INFO(get_logger(), "Docking node ready (scan=%s, cmd=%s).", scan_topic_.c_str(), cmd_vel_topic_.c_str());
}

DockingController::DockingControllerParams DockingNode::declareParameters()
{
  scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");

  DockingController::DockingControllerParams control_params;

  // FOV & wall classification
  control_params.forward_arc = declare_parameter<double>("forward_arc", 1.9199);
  control_params.face_normal_max = declare_parameter<double>("face_normal_max", 0.9599);
  control_params.side_normal_center = declare_parameter<double>("side_normal_center", -1.5708);
  control_params.side_normal_tol = declare_parameter<double>("side_normal_tol", 0.7854);

  // RANSAC line fit
  control_params.ransac.max_iterations = declare_parameter<int>("ransac_iterations", 120);
  control_params.ransac.inlier_threshold = declare_parameter<double>("ransac_inlier_threshold", 0.02);
  control_params.ransac.min_inliers = declare_parameter<int>("ransac_min_inliers", 12);
  control_params.ransac.max_rms_residual = declare_parameter<double>("ransac_max_rms_residual", 0.03);

  // Fit-quality & outlier gating
  control_params.good_fit_min_inliers = declare_parameter<int>("good_fit_min_inliers", 15);
  control_params.good_fit_max_residual = declare_parameter<double>("good_fit_max_residual", 0.025);
  control_params.wall_reject_band = declare_parameter<double>("wall_reject_band", 0.07);
  control_params.estimate_jump_thresh = declare_parameter<double>("estimate_jump_threshold", 0.30);

  // Dock goal - desired relative dock pose & success tolerances
  control_params.longitudinal_offset = declare_parameter<double>("longitudinal_offset", 0.45);
  control_params.lateral_offset = declare_parameter<double>("lateral_offset", 0.50);
  control_params.predock_pose_offset_longitudinal = declare_parameter<double>("predock_pose_offset_longitudinal", 1.0);
  control_params.xy_tolerance = declare_parameter<double>("xy_tolerance", 0.03);
  control_params.yaw_tolerance = declare_parameter<double>("yaw_tolerance", 0.02);

  // Search
  control_params.search_omega = declare_parameter<double>("search_omega", 0.25);
  control_params.search_sweep_cycles = declare_parameter<int>("search_sweep_cycles", 40);

  // Controller gains - Astolfi
  control_params.k_range = declare_parameter<double>("k_range", 0.6);
  control_params.k_alpha = declare_parameter<double>("k_alpha", 1.3);
  control_params.k_beta = declare_parameter<double>("k_beta", -0.45);
  control_params.k_yaw = declare_parameter<double>("k_yaw", 1.2);
  control_params.k_cross = declare_parameter<double>("k_cross", 0.8);
  control_params.linear_vel_max_align = declare_parameter<double>("linear_vel_max_align", 0.06);
  control_params.angular_vel_trim_max = declare_parameter<double>("angular_vel_trim_max", 0.25);
  control_params.linear_vel_max_approach = declare_parameter<double>("linear_vel_max_approach", 0.18);
  control_params.predock_ready_cycles = declare_parameter<int>("predock_ready_cycles", 5);

  // Fallback - if the robot fails to make progress, fall back to predock
  control_params.predock_progress_timeout = declare_parameter<double>("predock_progress_timeout", 6.0);
  control_params.relineup_lateral = declare_parameter<double>("relineup_lateral", 0.06);
  control_params.relineup_yaw = declare_parameter<double>("relineup_yaw", 0.12);

  // Final yaw alignment
  control_params.min_align_omega = declare_parameter<double>("min_align_omega", 0.15);
  control_params.align_kick_period = declare_parameter<int>("align_kick_period", 4);

  // Velocity & acceleration limits
  control_params.angular_vel_max = declare_parameter<double>("angular_vel_max", 0.8);
  max_lin_accel_ = declare_parameter<double>("max_lin_accel", 0.4);
  max_ang_accel_ = declare_parameter<double>("max_ang_accel", 1.5);
  control_params.error_ema_alpha = declare_parameter<double>("error_ema_alpha", 0.4);

  // State transitions & docking success
  control_params.valid_cycles_to_engage = declare_parameter<int>("valid_cycles_to_engage", 5);
  control_params.success_cycles = declare_parameter<int>("success_cycles", 8);
  control_params.hold_timeout = declare_parameter<double>("hold_timeout", 8.0);
  control_params.max_refit_attempts = declare_parameter<int>("max_refit_attempts", 3);

  // Obstacle stopping
  control_params.corridor_half_width = declare_parameter<double>("corridor_half_width", 0.18);
  control_params.obstacle_margin = declare_parameter<double>("obstacle_margin", 0.15);
  control_params.obstacle_stop_dist = declare_parameter<double>("obstacle_stop_distance", 0.30);

  control_rate_ = declare_parameter<double>("control_rate_hz", 20.0);
  control_params.auto_start = declare_parameter<bool>("auto_start", false);

  return control_params;
}

void DockingNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  docking_controller_->setScan(msg);
}

void DockingNode::handleStart(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  docking_controller_->dockStart();
  res->success = true;
  res->message = "Docking started.";
  RCLCPP_INFO(get_logger(), "Docking started via service.");
}

void DockingNode::handleCancel(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  docking_controller_->cancelDock();
  publishCmd(0.0, 0.0);
  res->success = true;
  res->message = "Docking cancelled.";
  RCLCPP_INFO(get_logger(), "Docking cancelled via service.");
}

void DockingNode::controlLoop()
{
  const DockingController::Command cmd_vel = docking_controller_->update(now());
  publishCmd(cmd_vel.linear_vel, cmd_vel.angular_vel);
  publishStatus(docking_controller_->lastGeometryEstimate());
  publishMarkers(docking_controller_->filteredGeometryEstimate());
}

void DockingNode::publishCmd(double linear_vel, double angular_vel)
{
  cmd_linear_vel_ = limitVelocityRate(linear_vel, cmd_linear_vel_, max_lin_accel_ / control_rate_);
  cmd_angular_vel_ = limitVelocityRate(angular_vel, cmd_angular_vel_, max_ang_accel_ / control_rate_);
  geometry_msgs::msg::Twist twist;
  twist.linear.x = cmd_linear_vel_;
  twist.angular.z = cmd_angular_vel_;
  cmd_vel_pub_->publish(twist);
}

double DockingNode::limitVelocityRate(double desired, double current, double max_delta)
{
  const double diff = desired - current;

  if (diff > max_delta)
  {
    return current + max_delta;
  }

  if (diff < -max_delta)
  {
    return current - max_delta;
  }

  return desired;
}

void DockingNode::publishStatus(const DockGeometry& geometry_estimate)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3) << "state=" << magic_enum::enum_name(docking_controller_->getDockState())
      << " face=" << geometry_estimate.face_valid << " side=" << geometry_estimate.side_valid
      << " dist_face=" << geometry_estimate.dist_face << " dist_side=" << geometry_estimate.dist_side
      << " range_err=" << geometry_estimate.range_err << " lateral_err=" << geometry_estimate.lateral_err
      << " yaw_err=" << geometry_estimate.yaw_err << " obstacle=" << geometry_estimate.obstacle_present
      << " obs_range=" << std::setprecision(2) << geometry_estimate.obstacle_range;
  std_msgs::msg::String msg;
  msg.data = oss.str();
  docking_status_pub_->publish(msg);
}

void DockingNode::publishMarkers(const DockGeometry& geometry_estimate)
{
  visualization_msgs::msg::MarkerArray arr;
  auto make_line = [&](int id, const LineModel& line, double r, double g, double b) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = base_frame_;
    m.header.stamp = now();
    m.ns = "dock";
    m.id = id;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.02;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0;
    const double fx = -line.c * line.a;  // foot of perpendicular
    const double fy = -line.c * line.b;
    const double dx = -line.b;  // line direction
    const double dy = line.a;
    geometry_msgs::msg::Point p1;
    geometry_msgs::msg::Point p2;
    p1.x = fx - 0.9 * dx;
    p1.y = fy - 0.9 * dy;
    p2.x = fx + 0.9 * dx;
    p2.y = fy + 0.9 * dy;
    m.points.push_back(p1);
    m.points.push_back(p2);
    return m;
  };

  if (geometry_estimate.face_valid)
  {
    arr.markers.push_back(make_line(0, geometry_estimate.face_line, 0.1, 0.5, 1.0));
  }

  if (geometry_estimate.side_valid)
  {
    arr.markers.push_back(make_line(1, geometry_estimate.side_line, 0.2, 0.9, 0.4));
  }

  if (geometry_estimate.corner_valid)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = base_frame_;
    m.header.stamp = now();
    m.ns = "dock";
    m.id = 2;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = geometry_estimate.corner.x;
    m.pose.position.y = geometry_estimate.corner.y;
    m.scale.x = m.scale.y = m.scale.z = 0.08;
    m.color.r = 1.0;
    m.color.a = 1.0;
    arr.markers.push_back(m);
  }

  /* Goal poses (robot frame): an ARROW pointing along the target heading marks
     where the robot is trying to go.
     The predock pose (yellow) is the pre dock stage before final approach;
     line-up point; the dock pose (magenta) is the final standoff pose.
  */

  auto make_pose_arrow = [&](int id, double pos_x, double pos_y, double yaw, double r, double g, double b) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = base_frame_;
    m.header.stamp = now();
    m.ns = "dock_goal";
    m.id = id;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = pos_x;
    m.pose.position.y = pos_y;
    m.pose.orientation.z = std::sin(yaw * 0.5);
    m.pose.orientation.w = std::cos(yaw * 0.5);
    m.scale.x = 0.25;  // shaft length
    m.scale.y = 0.04;  // shaft diameter
    m.scale.z = 0.04;  // head diameter
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0;
    return m;
  };

  auto make_pose_label = [&](int id, double pos_x, double pos_y, const std::string& text) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = base_frame_;
    m.header.stamp = now();
    m.ns = "dock_goal_label";
    m.id = id;
    m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = pos_x;
    m.pose.position.y = pos_y;
    m.pose.position.z = 0.15;
    m.scale.z = 0.12;  // text height
    m.color.r = m.color.g = m.color.b = 1.0;
    m.color.a = 1.0;
    m.text = text;
    return m;
  };

  if (geometry_estimate.face_valid)
  {
    // Final dock pose (magenta).
    arr.markers.push_back(make_pose_arrow(3, geometry_estimate.goal_x, geometry_estimate.goal_y,
                                          geometry_estimate.goal_yaw, 1.0, 0.0, 1.0));
    arr.markers.push_back(make_pose_label(5, geometry_estimate.goal_x, geometry_estimate.goal_y, "dock pose"));

    // Predock line-up pose (yellow).
    double pos_x = 0.0;
    double pos_y = 0.0;
    double yaw = 0.0;
    docking_controller_->predockGoal(geometry_estimate, pos_x, pos_y, yaw);
    arr.markers.push_back(make_pose_arrow(4, pos_x, pos_y, yaw, 1.0, 1.0, 0.0));
    arr.markers.push_back(make_pose_label(6, pos_x, pos_y, "predock pose"));
  }

  marker_pub_->publish(arr);
}

}  // namespace docking_controller

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<docking_controller::DockingNode>());
  rclcpp::shutdown();
  return 0;
}
