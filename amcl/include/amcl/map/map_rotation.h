/*
 *  Converts poses between the global frame and the axis-aligned grid frame
 *  of a rotated map.
 *
 *  An OccupancyGrid may carry a yaw in info.origin.orientation (map_server
 *  reads it from "origin: [x, y, yaw]"): the grid is rotated counter-clockwise
 *  by yaw around info.origin.position, i.e. around its lower-left corner.
 *  AMCL keeps the unrotated grid (no resampling, no extra memory) and instead
 *  runs the particle filter in the grid frame. Poses are converted only where
 *  they enter (initial pose) or leave (amcl_pose, particlecloud, tf) the filter.
 *  Motion and sensor models only use poses relative to the robot and are not
 *  affected.
 */
#ifndef AMCL_MAP_ROTATION_H
#define AMCL_MAP_ROTATION_H

#include <cmath>

#include "amcl/pf/pf_vector.h"

namespace amcl
{

class MapRotation
{
public:
  MapRotation() : yaw_(0.0), origin_x_(0.0), origin_y_(0.0) {}
  MapRotation(double yaw, double origin_x, double origin_y)
    : yaw_(yaw), origin_x_(origin_x), origin_y_(origin_y) {}

  double yaw() const { return yaw_; }
  bool isIdentity() const { return yaw_ == 0.0; }

  // global frame -> grid frame
  pf_vector_t toGrid(const pf_vector_t& pose) const { return rotatePose(pose, -yaw_); }
  pf_matrix_t toGrid(const pf_matrix_t& cov) const { return rotateCov(cov, -yaw_); }

  // grid frame -> global frame
  pf_vector_t toGlobal(const pf_vector_t& pose) const { return rotatePose(pose, yaw_); }
  pf_matrix_t toGlobal(const pf_matrix_t& cov) const { return rotateCov(cov, yaw_); }

private:
  pf_vector_t rotatePose(const pf_vector_t& pose, double angle) const
  {
    if(angle == 0.0)
      return pose;
    double c = cos(angle), s = sin(angle);
    double dx = pose.v[0] - origin_x_;
    double dy = pose.v[1] - origin_y_;
    pf_vector_t out;
    out.v[0] = origin_x_ + c * dx - s * dy;
    out.v[1] = origin_y_ + s * dx + c * dy;
    out.v[2] = atan2(sin(pose.v[2] + angle), cos(pose.v[2] + angle));
    return out;
  }

  // R * cov * R^T with R rotating x/y by angle and leaving theta unchanged
  static pf_matrix_t rotateCov(const pf_matrix_t& cov, double angle)
  {
    if(angle == 0.0)
      return cov;
    double c = cos(angle), s = sin(angle);
    double r[3][3] = {{c, -s, 0.0}, {s, c, 0.0}, {0.0, 0.0, 1.0}};
    double tmp[3][3];
    for(int i = 0; i < 3; i++)
      for(int j = 0; j < 3; j++)
      {
        tmp[i][j] = 0.0;
        for(int k = 0; k < 3; k++)
          tmp[i][j] += r[i][k] * cov.m[k][j];
      }
    pf_matrix_t out;
    for(int i = 0; i < 3; i++)
      for(int j = 0; j < 3; j++)
      {
        out.m[i][j] = 0.0;
        for(int k = 0; k < 3; k++)
          out.m[i][j] += tmp[i][k] * r[j][k];
      }
    return out;
  }

  double yaw_;
  double origin_x_;
  double origin_y_;
};

}  // namespace amcl

#endif
