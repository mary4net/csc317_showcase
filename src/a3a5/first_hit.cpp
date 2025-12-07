#include "first_hit.h"
#include "Object.h"
#include <Eigen/src/Core/util/Constants.h>
#include <limits>
#include <memory>

bool first_hit(
  const Ray & ray, 
  const double min_t,
  const std::vector< std::shared_ptr<Object> > & objects,
  int & hit_id, 
  double & t,
  Eigen::Vector3d & n)
{
  ////////////////////////////////////////////////////////////////////////////
  double tmp_t;
  Eigen::Vector3d tmp_n;
  t = std::numeric_limits<double>::infinity();
  bool ret = false;
  for (unsigned long i = 0; i < objects.size(); i++) {
    auto obj = objects[i];
    bool found = obj->intersect(ray, min_t, tmp_t, tmp_n);
    if (found && tmp_t <= t) {
      t = tmp_t;
      n = tmp_n;
      hit_id = i;
      ret = true;
    }
  }
  return ret;
  ////////////////////////////////////////////////////////////////////////////
}

