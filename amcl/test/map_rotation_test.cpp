#include <cmath>

#include <gtest/gtest.h>

#include "amcl/map/map_rotation.h"

namespace
{

pf_vector_t pose(double x, double y, double a)
{
  pf_vector_t p;
  p.v[0] = x;
  p.v[1] = y;
  p.v[2] = a;
  return p;
}

}  // namespace

TEST(MapRotation, IdentityKeepsPose)
{
  amcl::MapRotation rotation;
  pf_vector_t p = rotation.toGrid(pose(1.0, 2.0, 0.5));
  EXPECT_DOUBLE_EQ(p.v[0], 1.0);
  EXPECT_DOUBLE_EQ(p.v[1], 2.0);
  EXPECT_DOUBLE_EQ(p.v[2], 0.5);
}

TEST(MapRotation, RotatesAroundOrigin)
{
  // map rotated by 90 deg ccw around its origin (10, 20)
  amcl::MapRotation rotation(M_PI / 2, 10.0, 20.0);

  // the origin itself does not move
  pf_vector_t o = rotation.toGlobal(pose(10.0, 20.0, 0.0));
  EXPECT_NEAR(o.v[0], 10.0, 1e-9);
  EXPECT_NEAR(o.v[1], 20.0, 1e-9);
  EXPECT_NEAR(o.v[2], M_PI / 2, 1e-9);

  // 1 m along the grid x axis points along the global y axis
  pf_vector_t g = rotation.toGlobal(pose(11.0, 20.0, 0.0));
  EXPECT_NEAR(g.v[0], 10.0, 1e-9);
  EXPECT_NEAR(g.v[1], 21.0, 1e-9);
}

TEST(MapRotation, RoundTrip)
{
  amcl::MapRotation rotation(0.7, -3.0, 5.0);
  pf_vector_t in = pose(4.2, -1.3, 3.0);
  pf_vector_t out = rotation.toGlobal(rotation.toGrid(in));
  EXPECT_NEAR(out.v[0], in.v[0], 1e-9);
  EXPECT_NEAR(out.v[1], in.v[1], 1e-9);
  EXPECT_NEAR(out.v[2], in.v[2], 1e-9);
}

TEST(MapRotation, NormalizesAngle)
{
  amcl::MapRotation rotation(1.0, 0.0, 0.0);
  pf_vector_t g = rotation.toGlobal(pose(0.0, 0.0, 3.0));
  EXPECT_NEAR(g.v[2], 4.0 - 2 * M_PI, 1e-9);
}

TEST(MapRotation, RotatesCovariance)
{
  amcl::MapRotation rotation(M_PI / 2, 0.0, 0.0);
  pf_matrix_t cov = pf_matrix_zero();
  cov.m[0][0] = 4.0;  // uncertain along grid x
  cov.m[1][1] = 1.0;
  cov.m[2][2] = 0.1;
  pf_matrix_t g = rotation.toGlobal(cov);
  // ... which is the global y axis
  EXPECT_NEAR(g.m[0][0], 1.0, 1e-9);
  EXPECT_NEAR(g.m[1][1], 4.0, 1e-9);
  EXPECT_NEAR(g.m[0][1], 0.0, 1e-9);
  EXPECT_NEAR(g.m[2][2], 0.1, 1e-9);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
