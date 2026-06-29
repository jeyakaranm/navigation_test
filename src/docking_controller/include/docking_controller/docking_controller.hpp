/*  LiDAR-based precision docking controller (ROS-agnostic core).
    DockingController owns the dock dock_geometry extraction, the finite state
    machine and the control laws. It is deliberately decoupled from the ROS
    node: it consumes a raw laser scan plus the current time and returns a body
    velocity command, leaving all topic/service/timer plumbing to DockingNode.
*/

#ifndef DOCKING_CONTROLLER__DOCKING_CONTROLLER_HPP
#define DOCKING_CONTROLLER__DOCKING_CONTROLLER_HPP

#include <docking_controller/line_fit.hpp>
#include <magic_enum.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <vector>

namespace docking_controller
{

enum class DockState
{
  IDLE,
  SEARCH,
  APPROACH_PREDOCK,
  BACKUP_RETRY,
  PREDOCK,
  FINAL_APPROACH,
  DOCKED,
  HOLD,
  ABORT
};

struct DockGeometry
{
  bool face_valid{false};
  bool side_valid{false};
  double dist_face{0.0};    // perpendicular distance to the dock face (m)
  double dist_side{0.0};    // perpendicular distance to the side wall (m)
  double yaw_err{0.0};      // heading offset from facing the dock face (rad)
  double range_err{0.0};    // dist_face - longitudinal_offset (m)
  double lateral_err{0.0};  // dist_side - lateral_offset (m)
  double goal_x{0.0};       // longitudinal_offset goal pose X in the robot frame (m)
  double goal_y{0.0};       // lateral_offset goal pose Y in the robot frame (m)
  double goal_yaw{0.0};     // heading to hold on arrival at the goal (rad)
  Point2D corner{};         // intersection of face and side lines
  bool corner_valid{false};
  LineModel face_line{};
  LineModel side_line{};
  double obstacle_range{1e6};  // nearest intruding obstacle in corridor (m)
  bool obstacle_present{false};
};

class DockingController
{
 public:
  struct DockingControllerParams
  {
    double forward_arc{0.0};
    double face_normal_max{0.0};
    double side_normal_center{0.0};
    double side_normal_tol{0.0};

    RansacParams ransac{};

    double longitudinal_offset{0.45};
    double lateral_offset{0.50};
    double predock_pose_offset_longitudinal{1.0};
    double xy_tolerance{0.03};
    double yaw_tolerance{0.02};

    double k_range{0.6};
    double k_yaw{1.2};
    double k_cross{0.8};
    double k_alpha{1.3};
    double k_beta{-0.45};
    double min_align_omega{0.15};
    int align_kick_period{4};
    double angular_vel_trim_max{0.25};
    double relineup_lateral{0.06};
    double relineup_yaw{0.12};
    double predock_progress_timeout{6.0};
    int predock_ready_cycles{5};
    int predock_stuck_cycles{40};  // cycles at the predock pose without converging -> back up and retry
    double backup_distance{1.0};   // distance (m) to reverse from the dock before re-approaching
    int max_backup_attempts{2};    // non-converging back-up retries before giving up
    double linear_vel_max_approach{0.18};
    double linear_vel_max_align{0.06};
    double angular_vel_max{0.8};
    double error_ema_alpha{0.4};

    double corridor_half_width{0.18};
    double obstacle_margin{0.15};
    double wall_reject_band{0.07};
    double obstacle_stop_dist{0.30};
    double estimate_jump_thresh{0.30};
    int valid_cycles_to_engage{5};
    double hold_timeout{8.0};
    int success_cycles{8};

    int good_fit_min_inliers{15};         // face fit must keep at least this many inliers to be trusted
    double good_fit_max_residual{0.025};  // face fit RMS residual above this is treated as a bad fit
    int max_refit_attempts{3};            // pre-PREDOCK aborts that trigger a re-acquire instead of giving up
    int search_sweep_cycles{40};          // control cycles per half of the left-right search sweep
    double search_omega{0.25};            // angular rate (rad/s) used while searching for the dock

    bool auto_start{true};
  };

  struct Command
  {
    double linear_vel{0.0};
    double angular_vel{0.0};
  };

  /**
   * @brief Construct the controller with its tunables, logger and start time.
   *
   * @param params the controller tunables
   * @param logger logger used for state-transition diagnostics
   * @param start_time time base for the freshness, hold and progress timers
   */
  DockingController(const DockingControllerParams& params, rclcpp::Logger logger, const rclcpp::Time& start_time);

  /**
   * @brief Store the latest laser scan and its stamp.
   *
   * @param scan the latest laser scan
   */
  void setScan(const sensor_msgs::msg::LaserScan::SharedPtr& scan);

  /**
   * @brief Begin a docking attempt.
   */
  void dockStart();

  /**
   * @brief Cancel the docking attempt and return to idle.
   */
  void cancelDock();

  /**
   * @brief Run one control tick at time now.
   *
   * @param now the current time
   * @return the desired velocity command
   */
  Command update(const rclcpp::Time& now);

  /**
   * @brief Current state of the docking finite state machine.
   */
  DockState getDockState() const
  {
    return state_;
  }

  /**
   * @brief Most recent raw (unfiltered) dock dock_geometry estimate.
   */
  const DockGeometry& lastGeometryEstimate() const
  {
    return last_geometry_estimate_;
  }

  /**
   * @brief Low-pass filtered dock dock_geometry estimate.
   */
  const DockGeometry& filteredGeometryEstimate() const
  {
    return filtered_geometry_estimate_;
  }

  /**
   * @brief Pre-dock line-up goal (robot frame) for the given dock_geometry, exposed
   * so the node can visualize the predock pose in RViz.
   */
  void predockGoal(const DockGeometry& dock_geometry, double& goal_x, double& goal_y, double& goal_yaw) const
  {
    computePredockGoal(dock_geometry, goal_x, goal_y, goal_yaw);
  }

 private:
  /**
   * @brief Extract the dock face/side dock_geometry from the latest scan.
   *
   * @param dock_geometry populated with the extracted dock_geometry on success
   * @return true if a valid dock face was found
   */
  bool extractGeometry(DockGeometry& dock_geometry);

  /**
   * @brief field-of-view scan points within a bearing limit.
   *
   * @param max_bearing_rad maximum absolute bearing to keep (rad)
   * @return the in-arc points in the robot frame
   */
  std::vector<Point2D> filterPointsInFOV(double max_bearing_rad) const;

  /**
   * @brief Pre-dock line-up goal (robot frame): same lateral/heading target as
   * the final pose but at the larger predock standoff.
   *
   * @param dock_geometry dock dock_geometry to derive the goal from
   * @param goal_x populated with the goal X (m)
   * @param goal_y populated with the goal Y (m)
   * @param goal_yaw populated with the goal heading (rad)
   */
  void computePredockGoal(const DockGeometry& dock_geometry, double& goal_x, double& goal_y, double& goal_yaw) const;

  /**
   * @brief Lane-following law: drive forward to close range_to_go while
   * steering to null cross-track (lateral) and heading errors. Always moves
   * forward, so it arcs onto the dock centerline instead of spinning in place.
   *
   * @param range_to_go remaining longitudinal distance to close (m)
   * @param lateral_err cross-track error to the dock centerline (m)
   * @param yaw_err heading error to the dock face normal (rad)
   * @param side_valid whether the side wall (lateral reference) is available
   * @param linear_vel_max forward speed cap (m/s)
   * @param angular_vel_cap steering rate cap (rad/s)
   * @param linear_vel populated with the forward velocity command (m/s)
   * @param angular_vel populated with the angular velocity command (rad/s)
   */
  void laneFollow(double range_to_go, double lateral_err, double yaw_err, bool side_valid, double linear_vel_max,
                  double angular_vel_cap, double& linear_vel, double& angular_vel) const;

  /**
   * @brief Normalize an angle to (-pi, pi].
   *
   * @param theta angle to normalize (rad)
   * @return the equivalent angle in (-pi, pi]
   */
  static double normalizeAngle(double theta);

  /**
   * @brief Clamp value to the closed range [lo, hi].
   *
   * @param value value to clamp
   * @param lower_limit lower bound
   * @param upper_limit upper bound
   * @return value constrained to [lower_limit, upper_limit]
   */
  static double clamp(double value, double lower_limit, double upper_limit);

  /**
   * @brief Orient the line normal (a,b) so it points from the sensor origin
   * toward the wall, and return the normal bearing angle.
   *
   * @param line line whose normal is flipped in place to face the sensor
   * @param normal_angle populated with the oriented normal bearing (rad)
   */
  static void orientToWall(LineModel& line, double& normal_angle);

  /**
   * @brief Polar (Astolfi-style) go-to-pose law: drive the unicycle to a goal
   * pose expressed in the robot frame. Converges in position and heading
   * without the near-goal singularity of a pure bearing controller.
   *
   * @param goal_x goal X in the robot frame (m)
   * @param goal_y goal Y in the robot frame (m)
   * @param goal_yaw goal heading to settle on (rad)
   * @param k_rho range gain
   * @param k_alpha bearing gain
   * @param k_beta heading gain
   * @param linear_vel_max forward speed cap (m/s)
   * @param angular_vel_max steering rate cap (rad/s)
   * @param linear_vel populated with the forward velocity command (m/s)
   * @param angular_vel populated with the angular velocity command (rad/s)
   */
  static void goToPose(double goal_x, double goal_y, double goal_yaw, double k_rho, double k_alpha, double k_beta,
                       double linear_vel_max, double angular_vel_max, double& linear_vel, double& angular_vel);

  rclcpp::Logger logger_;

  DockingControllerParams docking_controller_params_;

  DockState state_{DockState::IDLE};
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  rclcpp::Time last_scan_time_;

  DockGeometry filtered_geometry_estimate_{};
  DockGeometry last_geometry_estimate_{};

  bool filter_initialized_{false};
  double prev_dist_face_{0.0};
  bool have_prev_face_{false};

  int valid_count_{0};
  int success_count_{0};
  int align_kick_counter_{0};
  int predock_ready_count_{0};
  double final_best_range_{1e6};
  rclcpp::Time final_progress_time_;
  rclcpp::Time hold_start_;
  bool in_hold_{false};
  DockState pre_hold_state_{DockState::SEARCH};
  int refit_attempts_{0};
  int predock_stuck_count_{0};
  int backup_attempts_{0};
  double backup_start_range_{0.0};
  double search_dir_{1.0};
  int search_ticks_{0};
  bool search_half_{true};
};

}  // namespace docking_controller

#endif  // DOCKING_CONTROLLER__DOCKING_CONTROLLER_HPP
