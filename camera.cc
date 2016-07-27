#include "camera.h"

using Eigen::Vector3d;

void Camera::Initialize(const Vector3d& center,
                        const double radius,
                        const Vector3d& view_degree,
                        const double perspective_angle_degree) {
  center_        = center;
  radius_        = radius;
  view_degree_   = view_degree;
  perspective_angle_degree_ = perspective_angle_degree;
}
