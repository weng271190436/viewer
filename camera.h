#ifndef VIEWER_CAMERA_H_
#define VIEWER_CAMERA_H_

#include <Eigen/Dense>

struct Camera {
 public:
  void Initialize(const Eigen::Vector3d& center,
                  const double radius,
                  const Eigen::Vector3d& view_degree,
                  const double perspective_angle_degree);

  // Fixed values.
  Eigen::Vector3d center_;
  double radius_;

  // View parameters.
  Eigen::Vector3d view_degree_;

  // Perspective angle.
  double perspective_angle_degree_;
};

#endif  // VIEWER_CAMERA_H_
