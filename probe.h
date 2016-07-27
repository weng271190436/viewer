#ifndef VIEWER_PROBE_H_
#define VIEWER_PROBE_H_

#include <Eigen/Dense>

struct Probe {
  // Position.
  Eigen::Vector2d lat_lng_degree;

  // Orientation.
  Eigen::Vector3d rotation_degree;

  // Vertical fov coverage.
  double vertical_fov_degree;
};

#endif  // VIEWER_PROBE_H_
