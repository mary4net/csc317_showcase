#include "blinn_phong_shading.h"
// Hint:
#include "Light.h"
#include "first_hit.h"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <cmath>
#include <iostream>

Eigen::Vector3d
blinn_phong_shading(const Ray &ray, const int &hit_id, const double &t,
                    const Eigen::Vector3d &n,
                    const std::vector<std::shared_ptr<Object>> &objects,
                    const std::vector<std::shared_ptr<Light>> &lights) {
  ////////////////////////////////////////////////////////////////////////////
  // Replace with your code here:
  Eigen::Vector3d L = Eigen::Vector3d(0, 0, 0);
  const double ia_const = 0.03;
  Eigen::Vector3d Ia = objects[hit_id]->material->ka * ia_const;
  L += Ia;

  Eigen::Vector3d p = ray.origin + ray.direction * t;
  auto obj = objects[hit_id];

  for (auto l : lights) {
    // check if its in shadow
    double max_t;
    Eigen::Vector3d l_dir;
    l->direction(p, l_dir, max_t);

    int shadow_hit_id;
    double shadow_t;
    Eigen::Vector3d shadow_n;
    Ray shadow_ray{p + 1e-6 * l_dir.normalized(), l_dir.normalized()};
    if (first_hit(shadow_ray, 1e-6, objects, shadow_hit_id, shadow_t,
                  shadow_n)) {
      if (shadow_t < max_t)
        // in shadow, ignore
        continue;
    }

    // diffuse light
    Eigen::Vector3d Id = obj->material->kd.cwiseProduct(l->I) *
                         std::max(0.0, n.normalized().dot(l_dir.normalized()));

    // specular light
    Eigen::Vector3d h = l_dir.normalized() - ray.direction.normalized();
    h = h.normalized();
    Eigen::Vector3d Is =
        obj->material->ks.cwiseProduct(l->I) *
        pow(std::max(0.0, n.dot(h)), obj->material->phong_exponent);

    L += Id;
    L += Is;
  }
  return L;
  ////////////////////////////////////////////////////////////////////////////
}
