#include "TriangleSoup.h"
#include "Ray.h"
// Hint
#include <memory>

bool TriangleSoup::intersect(const Ray &ray, const double min_t, double &t,
                             Eigen::Vector3d &n) const {
  ////////////////////////////////////////////////////////////////////////////
  t = std::numeric_limits<double>::infinity();
  double tmp_t;
  Eigen::Vector3d tmp_n;
  bool hit = false;
  for (std::shared_ptr<Object> tri: this->triangles) {
    if (tri->intersect(ray, min_t, tmp_t, tmp_n)){
      if (tmp_t < t) {
        t = tmp_t;
        n = tmp_n;
        hit = true;
      }
    }
  }
  return hit;
  ////////////////////////////////////////////////////////////////////////////
}
