/*
    2D line fitting utilities for LiDAR-based docking dock_geometry extraction.
    A line is represented in normal form:  a*x + b*y + c = 0  with a^2 + b^2 = 1.
    The signed distance of a point p to the line is simply a*p.x + b*p.y + c.
*/

#ifndef DOCKING_CONTROLLER__LINE_FIT_HPP
#define DOCKING_CONTROLLER__LINE_FIT_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>
namespace docking_controller
{

/**
 * @brief A 2D point in the robot frame.
 */
struct Point2D
{
  double x{0.0};
  double y{0.0};
};

/**
 * @brief A 2D line in normal form a*x + b*y + c = 0 with a^2 + b^2 = 1.
 */
struct LineModel
{
  double a{0.0};  // normal x component (a^2 + b^2 = 1)
  double b{0.0};  // normal y component
  double c{0.0};  // offset
  bool valid{false};
  int inliers{0};
  double rms_residual{0.0};

  /**
   * @brief Signed perpendicular distance from a point to the line.
   */
  double signedDistance(const Point2D& p) const
  {
    return a * p.x + b * p.y + c;
  }

  /**
   * @brief Perpendicular distance from the sensor origin (0,0) to the line.
   */
  double distanceFromOrigin() const
  {
    return std::abs(c);
  }
};

/**
 * @brief Parameters controlling the RANSAC line fit.
 */
struct RansacParams
{
  int max_iterations{120};
  double inlier_threshold{0.02};  // inlier band half-width (m)
  int min_inliers{15};
  double max_rms_residual{0.03};  // maximum accepted RMS residual (m)
  uint32_t seed{12345};
};

/**
 * @brief Total-least-squares (PCA) line fit over all supplied points.
 *
 * @param points the points to fit
 * @return the fitted LineModel: normal-form coefficients (a, b, c), inlier
 *         count, RMS residual, and valid=true; valid=false (other fields
 *         unusable) if the fit fails, e.g. fewer than two points
 */
LineModel fitLineTLS(const std::vector<Point2D>& points);

/**
 * @brief RANSAC line fit followed by a TLS refit over the inlier set.
 *
 * @param points the points to fit
 * @param params the RANSAC parameters / thresholds
 * @return the fitted LineModel: normal-form coefficients (a, b, c), inlier
 *         count, RMS residual, and valid=true; valid=false if the inlier
 *         count or residual fail the params thresholds
 */
LineModel fitLineRansac(const std::vector<Point2D>& points, const RansacParams& params);

/**
 * @brief Intersection of two lines.
 *
 * @param line1 the first line
 * @param line2 the second line
 * @param intersected_point the intersection point on success
 * @return false if the lines are near parallel
 */
bool intersectLines(const LineModel& line1, const LineModel& line2, Point2D& intersected_point);

}  // namespace docking_controller

#endif  // DOCKING_CONTROLLER__LINE_FIT_HPP
