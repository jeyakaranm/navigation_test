/*
  ROS 2 node wrapper around DockingController.
  DockingNode only declares parameters, subscribes
  to the laser scan , exposes start/cancel services, runs the
  control timer, and publishes the velocity command, status string and RViz
  markers. All dock_geometry extraction and control logic implemented in DockingController.
*/

#ifndef DOCKING_CONTROLLER__DOCKING_NODE_HPP
#define DOCKING_CONTROLLER__DOCKING_NODE_HPP

#include <chrono>
#include <cmath>
#include <docking_controller/docking_controller.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iomanip>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <visualization_msgs/msg/marker_array.hpp>

namespace docking_controller
{

class DockingNode : public rclcpp::Node
{
 public:
  DockingNode();

 private:
  /**
   * @brief laser scan callback, set the scan in the controller for processing.
   *
   * @param msg  laser scan data
   */
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  /**
   * @brief Periodic control tick: run the controller and publish outputs.
   */
  void controlLoop();

  /**
   * @brief Handle the start docking service request.
   *
   * @param req Trigger request for starting docking
   * @param res TriggerResponse populated on return with success and message
   */
  void handleStart(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  /**
   * @brief Handle the cancel docking service request.
   *
   * @param req Trigger request for canceling docking
   * @param res TriggerResponse populated on return with success and message
   */
  void handleCancel(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  /**
   * @brief publish cmd_vel
   *
   * @param linear_vel desired linear velocity (m/s)
   * @param angular_vel desired angular velocity (rad/s)
   */
  void publishCmd(double linear_vel, double angular_vel);

  /**
   * @brief Limit the rate of change of the velocity command to avoid jerky motion.
   *
   * @param desired the target command value
   * @param current the previously published command value
   * @param max_delta the maximum allowed change per tick
   * @return the rate-limited command value
   */
  double limitVelocityRate(double desired, double current, double max_delta);

  /**
   * @brief Publish the controller status string.
   *
   * @param geometry_estimate corner fitting dock_geometry estimate to include in the status message
   */
  void publishStatus(const DockGeometry& geometry_estimate);

  /**
   * @brief Publish the RViz dock dock_geometry markers.
   *
   * @param geometry_estimate corner fitting dock_geometry estimate to visualize
   */
  void publishMarkers(const DockGeometry& geometry_estimate);

  /**
   * @brief Declare ROS parameters: fills the node-only fields and returns the
   * controller tunables.
   *
   * @return the populated controller parameters
   */
  DockingController::DockingControllerParams declareParameters();

  std::string scan_topic_;
  std::string cmd_vel_topic_;
  std::string base_frame_;

  double max_lin_accel_;
  double max_ang_accel_;
  double control_rate_;

  std::unique_ptr<DockingController> docking_controller_;

  double cmd_linear_vel_{0.0};
  double cmd_angular_vel_{0.0};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr docking_status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_dock_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_dock_srv_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace docking_controller

#endif  // DOCKING_CONTROLLER__DOCKING_NODE_HPP
