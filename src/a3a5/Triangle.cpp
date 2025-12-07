#include "Triangle.h"
#include "Ray.h"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <cmath>

bool Triangle::intersect(const Ray &ray, const double min_t, double &t,
                         Eigen::Vector3d &n) const {
  ////////////////////////////////////////////////////////////////////////////
  // ((b-a) * x + (c-a) * y) = (q-a)
  // x + y <=1;
  // ((b-a) * x + (c-a) * y) = (o + t * d -a)
  ////////////////////////////////////////////////////////////////////////////
  double a = -ray.direction.x();
  double d = -ray.direction.y();
  double g = -ray.direction.z();
  double b = std::get<1>(this->corners).x() - std::get<0>(this->corners).x();
  double e = std::get<1>(this->corners).y() - std::get<0>(this->corners).y();
  double h = std::get<1>(this->corners).z() - std::get<0>(this->corners).z();
  double c = std::get<2>(this->corners).x() - std::get<0>(this->corners).x();
  double f = std::get<2>(this->corners).y() - std::get<0>(this->corners).y();
  double i = std::get<2>(this->corners).z() - std::get<0>(this->corners).z();
  double d1 = ray.origin.x() - std::get<0>(this->corners).x();
  double d2 = ray.origin.y() - std::get<0>(this->corners).y();
  double d3 = ray.origin.z() - std::get<0>(this->corners).z();
  double helper_A = d2 * i - d3 * f;
  double helper_B = d * i - f * g;
  double helper_C = d3 * d - g * d2;
  double helper_D = e * d3 - h * d2;
  double helper_E = d * h - e * g;
  double helper_F = e * i - h * f;
  double s = (d1 * helper_F - b * helper_A - c * helper_D) /
             (a * helper_F - b * helper_B + c * helper_E);
  double u = (a * helper_A - d1 * helper_B + c * helper_C) /
             (a * helper_F - b * helper_B + c * helper_E);
  double v = (a * helper_D - b * helper_C + d1 * helper_E) /
             (a * helper_F - b * helper_B + c * helper_E);
  if (s > min_t && u + v <= 1 && u >= 0 && v >= 0) {
    t = s;
    Eigen::Vector3d n_t =
        (std::get<1>(this->corners) - std::get<0>(this->corners))
            .cross(std::get<2>(this->corners) - std::get<0>(this->corners));
    n_t = n_t.normalized();
    if (n_t.dot(ray.direction) > 0) {
      n = -n_t;
    } else {
      n = n_t;
    }
    return true;
  }
  return false;
}
