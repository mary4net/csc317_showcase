#include "viewing_ray.h"
#include <Eigen/src/Core/Matrix.h>

void viewing_ray(const Camera &camera, const int i, const int j,
                 const int width, const int height, Ray &ray) {
  ////////////////////////////////////////////////////////////////////////////
  ray.origin = camera.e;
  ray.direction = 
      (camera.width / width * (j + 0.5) - camera.width / 2) * camera.u +
      (camera.height / height * (- i - 0.5) + camera.height / 2) * camera.v +
      -camera.d * camera.w;
  ////////////////////////////////////////////////////////////////////////////
}
