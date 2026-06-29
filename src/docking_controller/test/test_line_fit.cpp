#include <gtest/gtest.h>

#include <cmath>
#include <docking_controller/line_fit.hpp>
#include <vector>

using docking_controller::fitLineRansac;
using docking_controller::fitLineTLS;
using docking_controller::intersectLines;
using docking_controller::LineModel;
using docking_controller::Point2D;
using docking_controller::RansacParams;

TEST(LineFit, TlsRecoversVerticalLine)
{
  // Wall at x = 1.5 (vertical line), points span y.
  std::vector<Point2D> pts;
  for (int i = 0; i < 20; ++i)
  {
    pts.push_back({1.5, -1.0 + 0.1 * i});
  }
  LineModel m = fitLineTLS(pts);
  ASSERT_TRUE(m.valid);
  // Normal should be along x, distance from origin = 1.5.
  EXPECT_NEAR(std::abs(m.a), 1.0, 1e-6);
  EXPECT_NEAR(std::abs(m.b), 0.0, 1e-6);
  EXPECT_NEAR(m.distanceFromOrigin(), 1.5, 1e-6);
}

TEST(LineFit, RansacRejectsOutliers)
{
  std::vector<Point2D> pts;
  for (int i = 0; i < 40; ++i)
  {
    pts.push_back({2.0, -2.0 + 0.1 * i});
  }
  // Inject outliers.
  pts.push_back({0.5, 0.0});
  pts.push_back({3.2, 1.0});

  RansacParams p;
  p.inlier_threshold = 0.02;
  p.min_inliers = 20;
  LineModel m = fitLineRansac(pts, p);
  ASSERT_TRUE(m.valid);
  EXPECT_NEAR(m.distanceFromOrigin(), 2.0, 0.02);
  EXPECT_GE(m.inliers, 40);
}

TEST(LineFit, CornerIntersection)
{
  // Line 1: x = 3 -> 1*x + 0*y - 3 = 0
  LineModel l1;
  l1.a = 1.0;
  l1.b = 0.0;
  l1.c = -3.0;
  // Line 2: y = 0 -> 0*x + 1*y + 0 = 0
  LineModel l2;
  l2.a = 0.0;
  l2.b = 1.0;
  l2.c = 0.0;
  Point2D corner;
  ASSERT_TRUE(intersectLines(l1, l2, corner));
  EXPECT_NEAR(corner.x, 3.0, 1e-9);
  EXPECT_NEAR(corner.y, 0.0, 1e-9);
}
