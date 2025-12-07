#include "raycolor.h"
#include "Ray.h"
#include "first_hit.h"
#include "blinn_phong_shading.h"
#include "reflect.h"
#include "viewing_ray.h"
#include <Eigen/src/Core/Matrix.h>

bool raycolor(const Ray &ray, const double min_t,
              const std::vector<std::shared_ptr<Object>> &objects,
              const std::vector<std::shared_ptr<Light>> &lights,
              const int num_recursive_calls, Eigen::Vector3d &rgb) {
  ////////////////////////////////////////////////////////////////////////////
  int hit_id;
  double t;
  Eigen::Vector3d n;
  rgb = Eigen::Vector3d(0, 0, 0);
  if (first_hit(ray, min_t, objects, hit_id, t, n)) {
    Eigen::Vector3d shade_color =
        blinn_phong_shading(ray, hit_id, t, n, objects, lights);
    rgb += shade_color;

    if (num_recursive_calls < 3) {
      Ray mirror_ray;
      mirror_ray.direction = reflect(ray.direction, n);
      mirror_ray.origin = ray.origin + t * ray.direction +
                          1e-6 * mirror_ray.direction.normalized();

      Eigen::Vector3d rgb_rec;
      if (raycolor(mirror_ray, 1e-6, objects, lights, num_recursive_calls + 1,
                   rgb_rec)) {
        rgb += objects[hit_id]->material->km.cwiseProduct(rgb_rec);
      }
    }
    return true;
  }
  return false;
  ////////////////////////////////////////////////////////////////////////////
}
