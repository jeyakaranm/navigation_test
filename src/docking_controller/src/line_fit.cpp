
#include <docking_controller/line_fit.hpp>

namespace docking_controller
{

LineModel fitLineTLS(const std::vector<Point2D>& points)
{
  LineModel model;
  const std::size_t num_points = points.size();
  if (num_points < 2)
  {
    return model;
  }

  double mx = 0.0;
  double my = 0.0;

  for (const auto& p : points)
  {
    mx += p.x;
    my += p.y;
  }

  mx /= static_cast<double>(num_points);
  my /= static_cast<double>(num_points);

  // 2x2 covariance of the centred points.

  double sxx = 0.0;
  double syy = 0.0;
  double sxy = 0.0;

  for (const auto& p : points)
  {
    const double dx = p.x - mx;
    const double dy = p.y - my;
    sxx += dx * dx;
    syy += dy * dy;
    sxy += dx * dy;
  }

  // The line normal is the eigenvector of the smallest eigenvalue of the
  // covariance matrix [[sxx, sxy], [sxy, syy]].

  const double trace = sxx + syy;
  const double det = sxx * syy - sxy * sxy;
  const double disc = std::sqrt(std::max(0.0, trace * trace / 4.0 - det));
  const double lambda_min = trace / 2.0 - disc;

  double nx;
  double ny;

  // Eigenvector for lambda_min: (sxx - lambda) nx + sxy ny = 0.

  if (std::abs(sxy) > 1e-12)
  {
    nx = sxy;
    ny = lambda_min - sxx;
  } else
  {
    // Axis-aligned: pick the axis with the smaller spread as the normal.
    if (sxx <= syy)
    {
      nx = 1.0;
      ny = 0.0;
    } else
    {
      nx = 0.0;
      ny = 1.0;
    }
  }

  const double norm = std::hypot(nx, ny);

  if (norm < 1e-12)
  {
    return model;
  }

  model.a = nx / norm;
  model.b = ny / norm;
  model.c = -(model.a * mx + model.b * my);

  // Residual statistics.
  double sum_squared_residual = 0.0;

  for (const auto& p : points)
  {
    const double distance = model.signedDistance(p);
    sum_squared_residual += distance * distance;
  }

  model.rms_residual = std::sqrt(sum_squared_residual / static_cast<double>(num_points));
  model.inliers = static_cast<int>(num_points);
  model.valid = true;
  return model;
}

LineModel fitLineRansac(const std::vector<Point2D>& points, const RansacParams& params)
{
  LineModel best_candidate;
  const std::size_t num_points = points.size();

  if (static_cast<int>(num_points) < params.min_inliers)
  {
    return best_candidate;
  }

  std::mt19937 rng(params.seed);
  std::uniform_int_distribution<std::size_t> pick(0, num_points - 1);

  int best_inliers = 0;

  for (int n = 0; n < params.max_iterations; ++n)
  {
    std::size_t i = pick(rng);
    std::size_t j = pick(rng);
    if (i == j)
    {
      continue;
    }

    const Point2D& p1 = points[i];
    const Point2D& p2 = points[j];

    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    const double len = std::hypot(dx, dy);

    if (len < 1e-6)
    {
      continue;
    }

    // Normal is perpendicular to the direction (dx, dy).
    LineModel cand;
    cand.a = -dy / len;
    cand.b = dx / len;
    cand.c = -(cand.a * p1.x + cand.b * p1.y);

    int count = 0;

    for (const auto& p : points)
    {
      if (std::abs(cand.signedDistance(p)) <= params.inlier_threshold)
      {
        ++count;
      }
    }

    if (count > best_inliers)
    {
      best_inliers = count;
      best_candidate = cand;
    }
  }

  if (best_inliers < params.min_inliers)
  {
    return best_candidate;  // invalid
  }

  // Refit with TLS over the inlier set for accuracy.
  std::vector<Point2D> inlier_pts;
  inlier_pts.reserve(static_cast<std::size_t>(best_inliers));

  for (const auto& p : points)
  {
    if (std::abs(best_candidate.signedDistance(p)) <= params.inlier_threshold)
    {
      inlier_pts.push_back(p);
    }
  }

  LineModel refined = fitLineTLS(inlier_pts);

  if (!refined.valid)
  {
    return best_candidate;
  }

  refined.inliers = static_cast<int>(inlier_pts.size());
  refined.valid = (refined.inliers >= params.min_inliers) && (refined.rms_residual <= params.max_rms_residual);

  return refined;
}

bool intersectLines(const LineModel& line1, const LineModel& line2, Point2D& intersected_point)
{
  // Solve [a1 b1; a2 b2] [x; y] = [-c1; -c2].
  const double det = line1.a * line2.b - line2.a * line1.b;
  if (std::abs(det) < 1e-6)
  {
    return false;  // near parallel
  }
  intersected_point.x = (-line1.c * line2.b + line2.c * line1.b) / det;
  intersected_point.y = (-line1.a * line2.c + line2.a * line1.c) / det;
  return true;
}

}  // namespace docking_controller
