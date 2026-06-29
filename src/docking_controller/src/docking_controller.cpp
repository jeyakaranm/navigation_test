#include <algorithm>
#include <cmath>
#include <docking_controller/docking_controller.hpp>
#include <limits>

namespace docking_controller
{

DockingController::DockingController(const DockingControllerParams& params, rclcpp::Logger logger,
                                     const rclcpp::Time& start_time)
    : logger_(logger),
      docking_controller_params_(params),
      last_scan_time_(start_time),
      final_progress_time_(start_time),
      hold_start_(start_time)
{
  if (docking_controller_params_.auto_start)
  {
    state_ = DockState::SEARCH;
    RCLCPP_INFO(logger_, "Auto-start enabled: entering SEARCH.");
  }
}

void DockingController::setScan(const sensor_msgs::msg::LaserScan::SharedPtr& scan)
{
  last_scan_ = scan;
  last_scan_time_ = last_scan_->header.stamp;
}

void DockingController::dockStart()
{
  state_ = DockState::SEARCH;
  valid_count_ = 0;
  success_count_ = 0;
  filter_initialized_ = false;
  have_prev_face_ = false;
  in_hold_ = false;
  refit_attempts_ = 0;
  predock_stuck_count_ = 0;
  backup_attempts_ = 0;
  search_dir_ = 1.0;
  search_ticks_ = 0;
  search_half_ = true;
}

void DockingController::cancelDock()
{
  state_ = DockState::IDLE;
}

std::vector<Point2D> DockingController::filterPointsInFOV(double max_bearing_rad) const
{
  std::vector<Point2D> scan_pts;
  if (!last_scan_)
  {
    return scan_pts;
  }
  const auto& scan = *last_scan_;
  const std::size_t n = scan.ranges.size();
  scan_pts.reserve(n);
  for (std::size_t i = 0; i < n; ++i)
  {
    const double r = scan.ranges[i];
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max)
    {
      continue;
    }
    const double ang = normalizeAngle(scan.angle_min + static_cast<double>(i) * scan.angle_increment);
    if (std::abs(ang) > max_bearing_rad)
    {
      continue;
    }
    scan_pts.push_back({r * std::cos(ang), r * std::sin(ang)});
  }
  return scan_pts;
}

void DockingController::orientToWall(LineModel& line, double& normal_angle)
{
  if (line.c > 0.0)
  {
    line.a = -line.a;
    line.b = -line.b;
    line.c = -line.c;
  }
  normal_angle = std::atan2(line.b, line.a);
}

bool DockingController::extractGeometry(DockGeometry& dock_geometry)
{
  if (!last_scan_)
  {
    return false;
  }

  /* Collect the forward field of view and extract the dominant straight
   * segments via sequential RANSAC, each refit by TLS and oriented so its
   * normal points toward the wall. Three passes are enough to recover the
   * dock face and the perpendicular side wall robustly.
   */

  std::vector<Point2D> remaining_pts = filterPointsInFOV(docking_controller_params_.forward_arc);

  struct Segment
  {
    LineModel line;
    double normal_angle{0.0};
  };

  std::vector<Segment> segs;

  for (int pass = 0; pass < 3; ++pass)
  {
    LineModel fitted_line = fitLineRansac(remaining_pts, docking_controller_params_.ransac);
    if (!fitted_line.valid)
    {
      break;
    }

    // Drop this segment's inliers so the next pass finds the next wall.
    std::vector<Point2D> outliers;
    outliers.reserve(remaining_pts.size());
    for (const auto& pts : remaining_pts)
    {
      if (std::abs(fitted_line.signedDistance(pts)) > docking_controller_params_.ransac.inlier_threshold)
      {
        outliers.push_back(pts);
      }
    }

    Segment seg;
    seg.line = fitted_line;
    orientToWall(seg.line, seg.normal_angle);
    segs.push_back(seg);

    remaining_pts.swap(outliers);
  }

  /* ----- L-corner: assign the two perpendicular segments to roles -----------
   * A segment viewed obliquely can satisfy both the face and the side angular
   * gates at once, so a greedy "most inliers becomes the face" pick lets the
   * closer, denser side wall masquerade as the dock face when the robot starts
   * off the approach axis (it then docks square to the wrong wall and never
   * recovers a lateral reference). Instead we anchor on the angularly
   * distinctive side wall first (normal closest to side_normal_center_), then
   * pick the dock face from the remaining segments as the one most square-on
   * (smallest |normal|). The roles are mutually exclusive, so the true dock
   * face is never overwritten and a usable lateral reference survives an
   * oblique start.
   */

  int side_idx = -1;
  double best_side_score = docking_controller_params_.side_normal_tol;

  for (int i = 0; i < static_cast<int>(segs.size()); ++i)
  {
    const double score = std::abs(normalizeAngle(segs[i].normal_angle - docking_controller_params_.side_normal_center));
    if (score <= docking_controller_params_.side_normal_tol && score < best_side_score)
    {
      best_side_score = score;
      side_idx = i;
    }
  }

  int face_idx = -1;
  double best_face_score = docking_controller_params_.face_normal_max;

  for (int i = 0; i < static_cast<int>(segs.size()); ++i)
  {
    if (i == side_idx)
    {
      continue;
    }
    const double score = std::abs(segs[i].normal_angle);
    if (score <= docking_controller_params_.face_normal_max && score < best_face_score)
    {
      best_face_score = score;
      face_idx = i;
    }
  }

  if (face_idx >= 0 && segs[face_idx].line.valid)
  {
    dock_geometry.face_valid = true;
    dock_geometry.face_line = segs[face_idx].line;
    dock_geometry.dist_face = std::abs(dock_geometry.face_line.c);
    dock_geometry.yaw_err = normalizeAngle(std::atan2(dock_geometry.face_line.b, dock_geometry.face_line.a));
  }

  if (side_idx >= 0 && segs[side_idx].line.valid)
  {
    dock_geometry.side_valid = true;
    dock_geometry.side_line = segs[side_idx].line;
    dock_geometry.dist_side = std::abs(dock_geometry.side_line.c);
  }

  if (dock_geometry.face_valid && dock_geometry.side_valid)
  {
    dock_geometry.corner_valid = intersectLines(dock_geometry.face_line, dock_geometry.side_line, dock_geometry.corner);
  }

  dock_geometry.range_err = dock_geometry.dist_face - docking_controller_params_.longitudinal_offset;
  dock_geometry.lateral_err =
      dock_geometry.side_valid ? (dock_geometry.dist_side - docking_controller_params_.lateral_offset) : 0.0;

  // Longitudinal goal in the robot frame: range_err along the face normal plus
  // lateral_err along the side-wall normal, facing the dock face.

  if (dock_geometry.face_valid)
  {
    dock_geometry.goal_x = dock_geometry.range_err * dock_geometry.face_line.a;
    dock_geometry.goal_y = dock_geometry.range_err * dock_geometry.face_line.b;
    if (dock_geometry.side_valid)
    {
      dock_geometry.goal_x += dock_geometry.lateral_err * dock_geometry.side_line.a;
      dock_geometry.goal_y += dock_geometry.lateral_err * dock_geometry.side_line.b;
    }
    dock_geometry.goal_yaw = dock_geometry.yaw_err;
  }

  /* Obstacle gating: a point inside the approach corridor that is clearly
   * closer than the dock dock_geometry is treated as an intruding obstacle. Points
   * that belong to one of the detected walls are skipped so the dock walls
   * themselves are never mistaken for intruders.
   */

  double min_obs = std::numeric_limits<double>::max();
  const auto front = filterPointsInFOV(docking_controller_params_.forward_arc);

  for (const auto& p : front)
  {
    if (p.x > 0.05 && std::abs(p.y) < docking_controller_params_.corridor_half_width)
    {
      const bool on_wall = (dock_geometry.face_valid && std::abs(dock_geometry.face_line.signedDistance(p)) <=
                                                            docking_controller_params_.wall_reject_band) ||
                           (dock_geometry.side_valid && std::abs(dock_geometry.side_line.signedDistance(p)) <=
                                                            docking_controller_params_.wall_reject_band);
      if (on_wall)
      {
        continue;
      }

      const double rng = std::hypot(p.x, p.y);
      const double face_reach =
          dock_geometry.face_valid ? (dock_geometry.dist_face - docking_controller_params_.obstacle_margin) : 1e6;
      if (rng < face_reach && rng < min_obs)
      {
        min_obs = rng;
      }
    }
  }

  if (min_obs < std::numeric_limits<double>::max())
  {
    dock_geometry.obstacle_range = min_obs;
    dock_geometry.obstacle_present = true;
  }

  return dock_geometry.face_valid;
}

DockingController::Command DockingController::update(const rclcpp::Time& now)
{
  Command cmd;

  if (state_ == DockState::IDLE)
  {
    last_geometry_estimate_ = DockGeometry{};
    return cmd;
  }

  // ---- Scan freshness gate (handles dropout / frozen-timestamp bursts) ----

  const double scan_age = (now - last_scan_time_).seconds();
  const bool has_scan_updated = last_scan_ && scan_age < 0.5;

  DockGeometry dock_geometry;
  bool is_detection_valid = has_scan_updated && extractGeometry(dock_geometry);

  //  Estimate jump rejection
  if (is_detection_valid && have_prev_face_)
  {
    if (std::abs(dock_geometry.dist_face - prev_dist_face_) > docking_controller_params_.estimate_jump_thresh)
    {
      dock_geometry.face_valid = false;
      is_detection_valid = false;
    }
  }

  /* Fit-quality gate
   * A face line can satisfy RANSAC yet still be a wrong / low-support fit (few
   * inliers or a high residual). Such fits yield no usable corner and send the
   * robot chasing a bad target, so treat them as unusable: the robot then stops
   * (via HOLD) instead of driving randomly.
   */
  const bool is_good_fit = dock_geometry.face_valid &&
                           dock_geometry.face_line.inliers >= docking_controller_params_.good_fit_min_inliers &&
                           dock_geometry.face_line.rms_residual <= docking_controller_params_.good_fit_max_residual;

  if (dock_geometry.face_valid && is_good_fit)
  {
    prev_dist_face_ = dock_geometry.dist_face;
    have_prev_face_ = true;
  }

  //  Low-pass filter the usable measurements
  if (is_detection_valid && is_good_fit)
  {
    if (!filter_initialized_)
    {
      filtered_geometry_estimate_ = dock_geometry;
      filter_initialized_ = true;
    } else
    {
      // Filter the three control errors directly.
      const double ema_alpha = docking_controller_params_.error_ema_alpha;

      filtered_geometry_estimate_.range_err =
          ema_alpha * dock_geometry.range_err + (1 - ema_alpha) * filtered_geometry_estimate_.range_err;
      filtered_geometry_estimate_.yaw_err =
          ema_alpha * dock_geometry.yaw_err + (1 - ema_alpha) * filtered_geometry_estimate_.yaw_err;

      if (dock_geometry.side_valid)
      {
        filtered_geometry_estimate_.lateral_err =
            ema_alpha * dock_geometry.lateral_err + (1 - ema_alpha) * filtered_geometry_estimate_.lateral_err;
      }

      // Filter the go-to-pose goal expressed in the (current) robot frame.
      filtered_geometry_estimate_.goal_x =
          ema_alpha * dock_geometry.goal_x + (1 - ema_alpha) * filtered_geometry_estimate_.goal_x;
      filtered_geometry_estimate_.goal_y =
          ema_alpha * dock_geometry.goal_y + (1 - ema_alpha) * filtered_geometry_estimate_.goal_y;
      filtered_geometry_estimate_.goal_yaw =
          ema_alpha * dock_geometry.goal_yaw + (1 - ema_alpha) * filtered_geometry_estimate_.goal_yaw;
    }

    filtered_geometry_estimate_.face_valid = true;
    filtered_geometry_estimate_.side_valid = dock_geometry.side_valid;
    filtered_geometry_estimate_.face_line = dock_geometry.face_line;
    filtered_geometry_estimate_.side_line = dock_geometry.side_line;
    filtered_geometry_estimate_.corner = dock_geometry.corner;
    filtered_geometry_estimate_.corner_valid = dock_geometry.corner_valid;
    filtered_geometry_estimate_.dist_face = dock_geometry.dist_face;
    filtered_geometry_estimate_.dist_side = dock_geometry.dist_side;
    filtered_geometry_estimate_.obstacle_present = dock_geometry.obstacle_present;
    filtered_geometry_estimate_.obstacle_range = dock_geometry.obstacle_range;
  }

  //  Safety: obstacle in corridor
  const bool obstacle_stop =
      dock_geometry.obstacle_present && dock_geometry.obstacle_range < docking_controller_params_.obstacle_stop_dist;

  const bool geometry_ok = is_detection_valid && is_good_fit;

  if ((!geometry_ok || obstacle_stop) && state_ != DockState::DOCKED && state_ != DockState::ABORT &&
      state_ != DockState::SEARCH)
  {
    if (!in_hold_)
    {
      in_hold_ = true;
      hold_start_ = now;
      pre_hold_state_ = state_;
      state_ = DockState::HOLD;
      std::string reason;
      if (obstacle_stop)
      {
        reason = "obstacle in the approach corridor";
      }
      if (!geometry_ok)
      {
        if (!reason.empty())
        {
          reason += " and ";
        }
        if (!has_scan_updated)
        {
          reason += "no fresh scan (dropout / frozen LiDAR)";
        } else
        {
          reason +=
              "no usable dock wall in view (weak/oblique fit -> robot may be off the dock face, or scan corruption)";
        }
      }
      RCLCPP_WARN(logger_, "Entering HOLD: %s. Stopping the robot until it recovers (geometry_ok=%d obstacle_stop=%d).",
                  reason.c_str(), geometry_ok, obstacle_stop);
    }
  }

  switch (state_)
  {
    case DockState::SEARCH: {
      if (geometry_ok)
      {
        if (++valid_count_ >= docking_controller_params_.valid_cycles_to_engage)
        {
          state_ = DockState::APPROACH_PREDOCK;
          RCLCPP_INFO(logger_, "Geometry acquired: APPROACH_PREDOCK.");
        }
        cmd = {0.0, 0.0};
      } else
      {
        valid_count_ = 0;

        /* Bounded left-right sweep: oscillate the heading around the search
         * start instead of spinning one way forever, which can wander onto an
         * unrelated corner. The first leg is half-width so the sweep stays
         * centred; later legs are full-width as the direction alternates.
         */

        const int leg_limit = search_half_ ? docking_controller_params_.search_sweep_cycles
                                           : 2 * docking_controller_params_.search_sweep_cycles;

        if (++search_ticks_ >= leg_limit)
        {
          search_ticks_ = 0;
          search_dir_ = -search_dir_;
          search_half_ = false;
        }

        cmd = {0.0, search_dir_ * docking_controller_params_.search_omega};
      }

      break;
    }

    case DockState::APPROACH_PREDOCK: {
      const DockGeometry& dock_geometry = filtered_geometry_estimate_;

      double predock_goal_x = 0.0;
      double predock_goal_y = 0.0;
      double predock_goal_yaw = 0.0;

      computePredockGoal(dock_geometry, predock_goal_x, predock_goal_y, predock_goal_yaw);

      const double rho = std::hypot(predock_goal_x, predock_goal_y);
      double linear_vel = 0.0;
      double angular_vel = 0.0;

      goToPose(predock_goal_x, predock_goal_y, predock_goal_yaw, docking_controller_params_.k_range,
               docking_controller_params_.k_alpha, docking_controller_params_.k_beta,
               docking_controller_params_.linear_vel_max_approach, docking_controller_params_.angular_vel_max,
               linear_vel, angular_vel);

      cmd = {linear_vel, angular_vel};

      /* Only commit to PREDOCK once the predock pose is reached in position AND
       * alignment. go-to-pose converges in heading too, so it lines up lateral
       * and yaw here while there is still runway, instead of dumping a large
       * offset on PREDOCK where lane-follow has no room left to arc it out.
       */

      const bool lateral_ok = !dock_geometry.side_valid ||
                              std::abs(dock_geometry.lateral_err) < docking_controller_params_.relineup_lateral;
      const bool yaw_ok = std::abs(dock_geometry.yaw_err) < docking_controller_params_.relineup_yaw;

      if (rho < 0.25 && lateral_ok && yaw_ok)
      {
        state_ = DockState::PREDOCK;
        predock_ready_count_ = 0;
        predock_stuck_count_ = 0;
        backup_attempts_ = 0;
        RCLCPP_INFO(logger_, "Near pre-dock pose: PREDOCK line-up.");
      } else if (rho < 0.25 && ++predock_stuck_count_ >= docking_controller_params_.predock_stuck_cycles)
      {
        // Reached position but never lined up (lateral/yaw). Out of forward
        // runway, so back straight off the dock to buy room, then re-approach.

        predock_stuck_count_ = 0;

        if (backup_attempts_ < docking_controller_params_.max_backup_attempts)
        {
          ++backup_attempts_;
          backup_start_range_ = dock_geometry.range_err;
          state_ = DockState::BACKUP_RETRY;
          RCLCPP_WARN(logger_, "Pre-dock not converging: backing up to retry %d/%d.", backup_attempts_,
                      docking_controller_params_.max_backup_attempts);
        } else
        {
          state_ = DockState::ABORT;
          RCLCPP_ERROR(logger_, "Pre-dock not converging after %d back-ups: ABORT.", backup_attempts_);
        }
      }

      break;
    }

    case DockState::BACKUP_RETRY: {
      /*
       * Reverse straight off the dock (yaw-trim only, dock stays in the forward
       * arc) until range has grown by backup_distance, then re-approach.
       */

      const DockGeometry& dock_geometry = filtered_geometry_estimate_;

      cmd = {-docking_controller_params_.linear_vel_max_approach,
             clamp(docking_controller_params_.k_yaw * dock_geometry.yaw_err,
                   -docking_controller_params_.angular_vel_max, docking_controller_params_.angular_vel_max)};

      if (dock_geometry.range_err - backup_start_range_ >= docking_controller_params_.backup_distance)
      {
        state_ = DockState::APPROACH_PREDOCK;
        predock_stuck_count_ = 0;
        RCLCPP_INFO(logger_, "Backed off pre-dock: re-approaching.");
      }

      break;
    }

    case DockState::PREDOCK: {
      /*
       * Stage 1: lane-follow the dock centerline in to the pre-dock standoff,
       * driving the lateral (Y) and heading (yaw) errors to zero so the final
       * stage is a straight push. Lane-following always moves forward, so it
       * arcs onto the line instead of spinning the dock out of the laser FOV.
       */

      const DockGeometry& dock_geometry = filtered_geometry_estimate_;
      const double predock_range =
          dock_geometry.range_err - (docking_controller_params_.predock_pose_offset_longitudinal -
                                     docking_controller_params_.longitudinal_offset);

      double linear_vel = 0.0;
      double angular_vel = 0.0;

      laneFollow(predock_range, dock_geometry.lateral_err, dock_geometry.yaw_err, dock_geometry.side_valid,
                 docking_controller_params_.linear_vel_max_align, docking_controller_params_.angular_vel_max,
                 linear_vel, angular_vel);

      cmd = {linear_vel, angular_vel};

      const bool lateral_ok = !dock_geometry.side_valid ||
                              std::abs(dock_geometry.lateral_err) < docking_controller_params_.relineup_lateral;
      const bool yaw_ok = std::abs(dock_geometry.yaw_err) < docking_controller_params_.yaw_tolerance;

      if (std::abs(predock_range) < 0.08 && lateral_ok && yaw_ok)
      {
        if (++predock_ready_count_ >= docking_controller_params_.predock_ready_cycles)
        {
          state_ = DockState::FINAL_APPROACH;
          success_count_ = 0;
          align_kick_counter_ = 0;
          final_best_range_ = 1e6;
          final_progress_time_ = now;
          RCLCPP_INFO(logger_, "Lined up (Y, yaw): straight-in FINAL_APPROACH.");
        }
      } else
      {
        predock_ready_count_ = 0;
      }
      break;
    }

    case DockState::FINAL_APPROACH: {
      /*
       * Stage 2: straight-in push to the final standoff. Y and yaw are already
       * lined up, so we only trim heading gently while closing the X gap.
       */

      const DockGeometry& dock_geometry = filtered_geometry_estimate_;

      const bool lateral_ok =
          !dock_geometry.side_valid || std::abs(dock_geometry.lateral_err) < docking_controller_params_.xy_tolerance;

      const bool position_ok =
          std::abs(dock_geometry.range_err) < docking_controller_params_.xy_tolerance && lateral_ok;

      const bool yaw_ok = std::abs(dock_geometry.yaw_err) < docking_controller_params_.yaw_tolerance;

      // Track forward progress on the X gap for the stall watchdog.

      if (std::abs(dock_geometry.range_err) < final_best_range_ - 0.005)
      {
        final_best_range_ = std::abs(dock_geometry.range_err);
        final_progress_time_ = now;
      }

      const bool lineup_lost = (dock_geometry.side_valid &&
                                std::abs(dock_geometry.lateral_err) > docking_controller_params_.relineup_lateral) ||
                               std::abs(dock_geometry.yaw_err) > docking_controller_params_.relineup_yaw;

      const bool stalled = (now - final_progress_time_).seconds() > docking_controller_params_.predock_progress_timeout;

      // Fallback: drift or a transient block (e.g. corrupted scans) -> back off
      // to the pre-dock pose and re-line-up before trying again.

      if (!position_ok && (lineup_lost || stalled))
      {
        state_ = DockState::PREDOCK;
        predock_ready_count_ = 0;
        success_count_ = 0;
        align_kick_counter_ = 0;
        RCLCPP_WARN(logger_, "Straight-in %s: re-lining at pre-dock.", lineup_lost ? "drifted" : "stalled");
        break;
      }

      if (position_ok && !yaw_ok)
      {
        // Last mile: position is inside tolerance but the final heading is not.
        double angular_vel = docking_controller_params_.k_yaw * dock_geometry.yaw_err;

        if (std::abs(angular_vel) >= docking_controller_params_.min_align_omega)
        {
          // Error large enough that the proportional command clears the
          // diff-drive deadband: rotate in place smoothly.

          angular_vel = clamp(angular_vel, -docking_controller_params_.angular_vel_max,
                              docking_controller_params_.angular_vel_max);
          cmd = {0.0, angular_vel};
          align_kick_counter_ = 0;
        } else if (align_kick_counter_ <= 0)
        {
          /* Tiny residual: a continuous min-rate command hunts (overshoot then
           * reverse) because of the drive deadband and latency. Pulse instead -
           * one short nudge, then coast for a few cycles so the base settles and
           * we re-measure before deciding to nudge again.
           */

          angular_vel = clamp(std::copysign(docking_controller_params_.min_align_omega, dock_geometry.yaw_err),
                              -docking_controller_params_.angular_vel_max, docking_controller_params_.angular_vel_max);
          cmd = {0.0, angular_vel};
          align_kick_counter_ = docking_controller_params_.align_kick_period;

        } else
        {
          --align_kick_counter_;
          cmd = {0.0, 0.0};
        }
      } else
      {
        // Forward push with a gently capped steering trim (small corrections
        // only) so the robot drives in nearly straight.

        double linear_vel = 0.0;
        double angular_vel = 0.0;

        laneFollow(dock_geometry.range_err, dock_geometry.lateral_err, dock_geometry.yaw_err, dock_geometry.side_valid,
                   docking_controller_params_.linear_vel_max_align, docking_controller_params_.angular_vel_trim_max,
                   linear_vel, angular_vel);
        cmd = {linear_vel, angular_vel};
      }

      if (position_ok && yaw_ok)
      {
        if (++success_count_ >= docking_controller_params_.success_cycles)
        {
          state_ = DockState::DOCKED;
          cmd = {0.0, 0.0};
          RCLCPP_INFO(logger_, "DOCKED: range_err=%.3f lateral_err=%.3f yaw_err=%.3f", dock_geometry.range_err,
                      dock_geometry.lateral_err, dock_geometry.yaw_err);
        }
      } else
      {
        success_count_ = 0;
      }
      break;
    }

    case DockState::HOLD: {
      cmd = {0.0, 0.0};
      const bool cleared = geometry_ok && !obstacle_stop && has_scan_updated;

      if (cleared)
      {
        in_hold_ = false;
        state_ = pre_hold_state_;
        RCLCPP_INFO(logger_, "HOLD cleared: resuming %s.", magic_enum::enum_name(state_).data());
      } else if ((now - hold_start_).seconds() > docking_controller_params_.hold_timeout)
      {
        /* Refit fallback: if the dock was lost before committing to the pre-dock
         * line-up (still in APPROACH_PREDOCK), re-acquire from scratch a bounded number
         * of times so a transient bad fit does not end the attempt. Once at or
         * past PREDOCK, an abort is terminal.
         */

        if (pre_hold_state_ == DockState::APPROACH_PREDOCK &&
            refit_attempts_ < docking_controller_params_.max_refit_attempts)
        {
          ++refit_attempts_;
          in_hold_ = false;
          filter_initialized_ = false;
          valid_count_ = 0;

          /* Keep prev_dist_face_ / have_prev_face_ so the estimate-jump guard
           * rejects an unrelated corner during re-acquire: the sweep must
           * settle back on the dock at the last known range, not a neighbour.
           */

          search_dir_ = 1.0;
          search_ticks_ = 0;
          search_half_ = true;
          state_ = DockState::SEARCH;
          RCLCPP_WARN(logger_, "Lost dock before PREDOCK: refit attempt %d/%d (re-acquiring).", refit_attempts_,
                      docking_controller_params_.max_refit_attempts);
        } else
        {
          state_ = DockState::ABORT;
          RCLCPP_ERROR(logger_, "HOLD timeout exceeded: ABORT.");
        }
      }
      break;
    }

    case DockState::DOCKED:
    case DockState::ABORT:
    case DockState::IDLE:
    default:
      cmd = {0.0, 0.0};
      break;
  }

  last_geometry_estimate_ = dock_geometry;
  return cmd;
}

double DockingController::normalizeAngle(double theta)
{
  while (theta > M_PI)
  {
    theta -= 2.0 * M_PI;
  }
  while (theta < -M_PI)
  {
    theta += 2.0 * M_PI;
  }
  return theta;
}

double DockingController::clamp(double value, double lower_limit, double upper_limit)
{
  return std::max(lower_limit, std::min(upper_limit, value));
}

void DockingController::goToPose(double goal_x, double goal_y, double goal_yaw, double k_rho, double k_alpha,
                                 double k_beta, double linear_vel_max, double angular_vel_max, double& linear_vel,
                                 double& angular_vel)
{
  const double rho = std::hypot(goal_x, goal_y);
  const double alpha = std::atan2(goal_y, goal_x);  // bearing to the goal point
  const double beta = normalizeAngle(goal_yaw - alpha);

  /* cos(alpha) shapes the forward speed so the robot leads with a turn toward
   * the goal. A floor on that factor keeps a little forward creep even when the
   * goal is abeam (alpha ~ 90 deg): the robot then arcs onto the goal instead
   * of spinning in place, which would otherwise swing the dock out of the
   * laser's forward arc. A small reverse term keeps it controllable on a slight
   * standoff overshoot.
   */

  const double fwd_shape = std::max(0.30, std::cos(alpha));

  linear_vel = clamp(k_rho * rho * fwd_shape, -0.02, linear_vel_max);
  angular_vel = clamp(k_alpha * alpha + k_beta * beta, -angular_vel_max, angular_vel_max);
}

void DockingController::computePredockGoal(const DockGeometry& dock_geometry, double& goal_x, double& goal_y,
                                           double& goal_yaw) const
{
  /* Same lateral/heading target as the final pose, but at the larger predock
   * standoff so stage 1 lines up Y and yaw from a distance.
   */

  const double predock_range_err =
      dock_geometry.range_err -
      (docking_controller_params_.predock_pose_offset_longitudinal - docking_controller_params_.longitudinal_offset);

  goal_x = predock_range_err * dock_geometry.face_line.a;
  goal_y = predock_range_err * dock_geometry.face_line.b;

  if (dock_geometry.side_valid)
  {
    goal_x += dock_geometry.lateral_err * dock_geometry.side_line.a;
    goal_y += dock_geometry.lateral_err * dock_geometry.side_line.b;
  }
  goal_yaw = dock_geometry.goal_yaw;
}

void DockingController::laneFollow(double range_to_go, double lateral_err, double yaw_err, bool side_valid,
                                   double linear_vel_max, double angular_vel_cap, double& linear_vel,
                                   double& angular_vel) const
{
  // Forward speed closes the remaining range, throttled when badly misaligned
  // so the robot turns onto the lane before driving hard.

  const double align = std::max(0.25, std::cos(yaw_err));
  linear_vel = clamp(docking_controller_params_.k_range * range_to_go * align, -0.02, linear_vel_max);

  /* Steer to null heading-to-normal (yaw_err) and cross-track (lateral) error.
   * A positive lateral_err means the robot is too far from the side wall, so it
   * needs a right turn (negative w) to slide back toward the centerline.
   */

  double steer = docking_controller_params_.k_yaw * yaw_err;
  if (side_valid)
  {
    steer -= docking_controller_params_.k_cross * lateral_err;
  }
  angular_vel = clamp(steer, -angular_vel_cap, angular_vel_cap);
}

}  // namespace docking_controller
