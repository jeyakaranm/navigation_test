// Disturbance injector for the docking demo. It sits between the simulator
// and the docking controller and injects faults that are gated on the
// controller's docking stage (read from /docking/status) so each disturbance
// is exercised during the docking phase and nothing keeps firing after the
// robot has docked:
//   * scan dropout      : drop scan publication for a burst at a chosen stage
//                         (ages the controller scan -> freshness gate)
//   * scan corruption    : blank the forward sector to +inf for a burst at a
//                         chosen stage (simulates glass / occlusion on the dock)
//   * pose noise         : add Gaussian odom noise during a chosen stage
//   * moving obstacle    : drive a small obstacle into the approach corridor
//                         once, hold it until the robot stops, then clear it so
//                         the robot is given time to recover and resume.
//
// Each scan/obstacle disturbance fires once per docking session (re-armed only
// from IDLE, not on a refit) and is targeted at a configurable stage.

#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <limits>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class DisturbanceNode : public rclcpp::Node
{
 public:
  DisturbanceNode() : rclcpp::Node("disturbance_node"), rng_(std::random_device{}())
  {
    enable_dropout_ = declare_parameter<bool>("enable_scan_dropout", true);
    enable_corruption_ = declare_parameter<bool>("enable_scan_corruption", true);
    enable_pose_noise_ = declare_parameter<bool>("enable_pose_noise", true);
    enable_obstacle_ = declare_parameter<bool>("enable_moving_obstacle", true);

    scan_in_topic_ = declare_parameter<std::string>("scan_in", "/scan");
    scan_out_topic_ = declare_parameter<std::string>("scan_out", "/scan_dock");
    odom_in_topic_ = declare_parameter<std::string>("odom_in", "/odom");
    odom_out_topic_ = declare_parameter<std::string>("odom_out", "/odom_dock");
    status_topic_ = declare_parameter<std::string>("status_topic", "/docking/status");

    // Each disturbance is injected at a specific docking stage so they are
    // exercised across the run (controller states: SEARCH, APPROACH_PREDOCK, PREDOCK,
    // FINAL_APPROACH) instead of randomly / after docking. During APPROACH_PREDOCK the corruption
    // burst hits first (with pose noise), then the scan dropout fires after it
    // (dropout_delay), so both are exercised before the robot reaches the
    // predock standoff; the obstacle then stops the robot between PREDOCK/DOCK.
    dropout_stage_ = declare_parameter<std::string>("dropout_stage", "APPROACH_PREDOCK");
    corruption_stage_ = declare_parameter<std::string>("corruption_stage", "APPROACH_PREDOCK");
    pose_noise_stage_ = declare_parameter<std::string>("pose_noise_stage", "APPROACH_PREDOCK");
    obstacle_stage_ = declare_parameter<std::string>("obstacle_stage", "PREDOCK");

    dropout_duration_ = declare_parameter<double>("dropout_duration", 2.0);
    dropout_delay_ = declare_parameter<double>("dropout_delay", 2.0);
    corruption_duration_ = declare_parameter<double>("corruption_duration", 2.0);
    corruption_sector_width_deg_ = declare_parameter<double>("corruption_sector_width_deg", 40.0);

    pose_noise_xy_ = declare_parameter<double>("pose_noise_xy_stddev", 0.08);
    pose_noise_yaw_ = declare_parameter<double>("pose_noise_yaw_stddev", 0.05);

    // Moving obstacle: parked off the approach path, then driven into the
    // corridor once during its stage, held until the robot stops, then cleared.
    obstacle_name_ = declare_parameter<std::string>("obstacle_name", "moving_obstacle");
    obstacle_block_x_ = declare_parameter<double>("obstacle_block_x", 2.25);
    obstacle_block_y_ = declare_parameter<double>("obstacle_block_y", 0.5);
    obstacle_park_x_ = declare_parameter<double>("obstacle_park_x", 2.2);
    obstacle_park_y_ = declare_parameter<double>("obstacle_park_y", 2.5);
    obstacle_x_ = obstacle_park_x_;
    obstacle_y_ = obstacle_park_y_;
    obstacle_block_delay_ = declare_parameter<double>("obstacle_block_delay", 0.3);
    obstacle_clear_after_hold_ = declare_parameter<double>("obstacle_clear_after_hold", 4.0);
    obstacle_block_max_ = declare_parameter<double>("obstacle_block_max", 10.0);
    obstacle_repeat_ = declare_parameter<bool>("obstacle_repeat", false);
    obstacle_speed_ = declare_parameter<double>("obstacle_speed", 0.4);
    set_state_service_ = declare_parameter<std::string>("set_state_service", "/dock/set_entity_state");

    auto scan_qos = rclcpp::SensorDataQoS();
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_out_topic_, scan_qos);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_in_topic_, scan_qos, std::bind(&DisturbanceNode::scanCb, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_out_topic_, 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_in_topic_, rclcpp::QoS(10), std::bind(&DisturbanceNode::odomCb, this, std::placeholders::_1));

    status_sub_ = create_subscription<std_msgs::msg::String>(
        status_topic_, rclcpp::QoS(10), std::bind(&DisturbanceNode::statusCb, this, std::placeholders::_1));

    if (enable_obstacle_)
    {
      state_client_ = create_client<gazebo_msgs::srv::SetEntityState>(set_state_service_);
      obstacle_timer_ = create_wall_timer(100ms, std::bind(&DisturbanceNode::moveObstacle, this));
    }

    RCLCPP_INFO(get_logger(),
                "Disturbance injector (stage-gated): dropout=%d@%s corruption=%d@%s pose_noise=%d@%s obstacle=%d@%s",
                enable_dropout_, dropout_stage_.c_str(), enable_corruption_, corruption_stage_.c_str(),
                enable_pose_noise_, pose_noise_stage_.c_str(), enable_obstacle_, obstacle_stage_.c_str());
  }

 private:
  // Track the controller's docking stage from its status string ("state=XXX ..")
  // and arm one-shot disturbances when their target stage is first entered.
  void statusCb(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string& s = msg->data;
    const std::string key = "state=";
    const auto pos = s.find(key);
    std::string st = "IDLE";
    if (pos != std::string::npos)
    {
      const auto start = pos + key.size();
      const auto end = s.find(' ', start);
      st = s.substr(start, end == std::string::npos ? std::string::npos : end - start);
    }
    if (st == dock_state_)
    {
      return;
    }
    dock_state_ = st;
    RCLCPP_INFO(get_logger(), "==== Docking stage -> %s ====", dock_state_.c_str());

    // Terminal/idle states end the session: stop every active disturbance and
    // re-arm the one-shots so the next session starts clean. We deliberately
    // do NOT re-arm on a refit (which re-enters SEARCH) so the obstacle does
    // not block the same retry forever.
    if (dock_state_ == "IDLE" || dock_state_ == "DOCKED" || dock_state_ == "ABORT")
    {
      dropout_done_ = false;
      corruption_done_ = false;
      obstacle_done_ = false;
      dropout_active_ = false;
      in_corruption_ = false;
      obstacle_armed_ = false;
      obstacle_hold_seen_ = false;
      return;
    }

    const rclcpp::Time t = now();
    if (enable_dropout_ && !dropout_done_ && dock_state_ == dropout_stage_)
    {
      dropout_done_ = true;
      dropout_active_ = true;
      dropout_start_ = t + rclcpp::Duration::from_seconds(dropout_delay_);
      dropout_end_ = dropout_start_ + rclcpp::Duration::from_seconds(dropout_duration_);
      RCLCPP_WARN(get_logger(), "[%s] Scan DROPOUT burst (%.1fs) starting in %.1fs.", dock_state_.c_str(),
                  dropout_duration_, dropout_delay_);
    }
    if (enable_corruption_ && !corruption_done_ && dock_state_ == corruption_stage_)
    {
      corruption_done_ = true;
      in_corruption_ = true;
      corrupt_pending_ = true;  // sector chosen on the next scan (needs scan dock_geometry)
      corruption_end_ = t + rclcpp::Duration::from_seconds(corruption_duration_);
      RCLCPP_WARN(get_logger(), "[%s] Scan CORRUPTION burst (%.1fs).", dock_state_.c_str(), corruption_duration_);
    }
    if (enable_obstacle_ && !obstacle_done_ && dock_state_ == obstacle_stage_)
    {
      obstacle_armed_ = true;
      obstacle_hold_seen_ = false;
      obstacle_block_start_ = t + rclcpp::Duration::from_seconds(obstacle_block_delay_);
      RCLCPP_WARN(get_logger(), "[%s] Obstacle will block the corridor in %.1fs.", dock_state_.c_str(),
                  obstacle_block_delay_);
    }
  }

  void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const rclcpp::Time t = now();

    // --- Scan dropout: drop publication for the burst window (ages scan). --
    if (dropout_active_)
    {
      if (t >= dropout_end_)
      {
        dropout_active_ = false;
      } else if (t >= dropout_start_)
      {
        return;  // drop this scan -> controller freshness gate trips
      }
    }

    sensor_msgs::msg::LaserScan out = *msg;

    // --- Scan corruption: blank the forward sector (where the dock is) to
    //     +inf for the burst window so the line fit actually degrades. ------
    if (in_corruption_)
    {
      if (t < corruption_end_)
      {
        const std::size_t n = out.ranges.size();
        if (n > 0)
        {
          if (corrupt_pending_)
          {
            // Centre the blanked sector on bearing 0 (forward), where the dock
            // face is, so the corruption is a meaningful test of the fit gate.
            const double idx0 =
                (0.0 - static_cast<double>(out.angle_min)) / std::max(1e-6, static_cast<double>(out.angle_increment));
            const double width_rad = corruption_sector_width_deg_ * M_PI / 180.0;
            const long count =
                static_cast<long>(std::max(1.0, width_rad / std::max(1e-6, static_cast<double>(out.angle_increment))));
            corrupt_count_ = static_cast<std::size_t>(count);
            long start = std::lround(idx0) - count / 2;
            start = ((start % static_cast<long>(n)) + static_cast<long>(n)) % static_cast<long>(n);
            corrupt_start_idx_ = static_cast<std::size_t>(start);
            corrupt_pending_ = false;
          }
          for (std::size_t k = 0; k < corrupt_count_; ++k)
          {
            out.ranges[(corrupt_start_idx_ + k) % n] = std::numeric_limits<float>::infinity();
          }
        }
      } else
      {
        in_corruption_ = false;
      }
    }

    scan_pub_->publish(out);
  }

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    nav_msgs::msg::Odometry out = *msg;
    // Pose noise only during its configured stage, to force LiDAR reliance.
    if (enable_pose_noise_ && dock_state_ == pose_noise_stage_)
    {
      std::normal_distribution<double> nxy(0.0, pose_noise_xy_);
      std::normal_distribution<double> nyaw(0.0, pose_noise_yaw_);
      out.pose.pose.position.x += nxy(rng_);
      out.pose.pose.position.y += nxy(rng_);
      double yaw = tf2::getYaw(out.pose.pose.orientation) + nyaw(rng_);
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      out.pose.pose.orientation = tf2::toMsg(q);
    }
    odom_pub_->publish(out);
  }

  void moveObstacle()
  {
    if (!state_client_->service_is_ready())
    {
      return;
    }
    const rclcpp::Time t = now();
    double x = obstacle_park_x_;  // default: parked off the approach path
    double y = obstacle_park_y_;

    if (obstacle_armed_ && t >= obstacle_block_start_)
    {
      bool clear = false;
      // Once the robot has actually stopped (HOLD), keep blocking a short while
      // to demonstrate the safe stop, then clear so it can recover.
      if (dock_state_ == "HOLD")
      {
        if (!obstacle_hold_seen_)
        {
          obstacle_hold_seen_ = true;
          obstacle_hold_time_ = t;
        } else if ((t - obstacle_hold_time_).seconds() >= obstacle_clear_after_hold_)
        {
          clear = true;
        }
      }
      // Safety cap: clear even if the robot never came close enough to stop.
      if ((t - obstacle_block_start_).seconds() >= obstacle_block_max_)
      {
        clear = true;
      }
      if (clear)
      {
        if (obstacle_repeat_)
        {
          // Continuous mode: re-arm so the obstacle blocks again once the robot
          // has moved off, instead of clearing for the rest of the session.
          obstacle_hold_seen_ = false;
          obstacle_block_start_ = t + rclcpp::Duration::from_seconds(obstacle_block_delay_);
          RCLCPP_INFO(get_logger(), "Obstacle cleared: re-arming to block again.");
        } else
        {
          obstacle_armed_ = false;
          obstacle_done_ = true;
          RCLCPP_INFO(get_logger(), "Obstacle cleared: robot may recover and resume.");
        }
      } else
      {
        x = obstacle_block_x_;
        y = obstacle_block_y_;
      }
    }

    // Glide toward the target (block or park) at a fixed speed so the obstacle
    // moves like a real object instead of teleporting between positions.
    constexpr double dt = 0.1;  // matches the 100 ms timer period
    const double step = obstacle_speed_ * dt;
    const double dx = x - obstacle_x_;
    const double dy = y - obstacle_y_;
    const double dist = std::hypot(dx, dy);
    if (dist <= step || dist < 1e-6)
    {
      obstacle_x_ = x;
      obstacle_y_ = y;
    } else
    {
      obstacle_x_ += step * dx / dist;
      obstacle_y_ += step * dy / dist;
    }

    auto req = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    req->state.name = obstacle_name_;
    req->state.pose.position.x = obstacle_x_;
    req->state.pose.position.y = obstacle_y_;
    req->state.pose.position.z = 0.1;
    req->state.pose.orientation.w = 1.0;
    req->state.reference_frame = "world";
    state_client_->async_send_request(req);
  }

  // Parameters
  bool enable_dropout_;
  bool enable_corruption_;
  bool enable_pose_noise_;
  bool enable_obstacle_;
  std::string scan_in_topic_;
  std::string scan_out_topic_;
  std::string odom_in_topic_;
  std::string odom_out_topic_;
  std::string status_topic_;
  std::string dropout_stage_;
  std::string corruption_stage_;
  std::string pose_noise_stage_;
  std::string obstacle_stage_;
  double dropout_duration_;
  double dropout_delay_;
  double corruption_duration_;
  double corruption_sector_width_deg_;
  double pose_noise_xy_;
  double pose_noise_yaw_;
  std::string obstacle_name_;
  double obstacle_block_x_;
  double obstacle_block_y_;
  double obstacle_park_x_;
  double obstacle_park_y_;
  double obstacle_block_delay_;
  double obstacle_clear_after_hold_;
  double obstacle_block_max_;
  bool obstacle_repeat_;
  double obstacle_speed_;
  std::string set_state_service_;

  // State
  std::mt19937 rng_;
  std::string dock_state_{"IDLE"};
  bool dropout_active_{false};
  bool dropout_done_{false};
  rclcpp::Time dropout_start_;
  rclcpp::Time dropout_end_;
  bool in_corruption_{false};
  bool corruption_done_{false};
  bool corrupt_pending_{false};
  rclcpp::Time corruption_end_;
  std::size_t corrupt_start_idx_{0};
  std::size_t corrupt_count_{0};
  bool obstacle_armed_{false};
  bool obstacle_done_{false};
  bool obstacle_hold_seen_{false};
  double obstacle_x_{0.0};
  double obstacle_y_{0.0};
  rclcpp::Time obstacle_block_start_;
  rclcpp::Time obstacle_hold_time_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr state_client_;
  rclcpp::TimerBase::SharedPtr obstacle_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DisturbanceNode>());
  rclcpp::shutdown();
  return 0;
}
