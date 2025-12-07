#include "triangle_area_normal.h"
#include <Eigen/Geometry>
#include <Eigen/src/Core/Matrix.h>

Eigen::RowVector3d triangle_area_normal(
  const Eigen::RowVector3d & a, 
  const Eigen::RowVector3d & b, 
  const Eigen::RowVector3d & c)
{
  ////////////////////////////////////////////////////////////////////////////
  // Replace with your code:
  Eigen::Vector3d v1 = Eigen::Vector3d(b-a);
  Eigen::Vector3d v2 = Eigen::Vector3d(c-a);
  Eigen::Vector3d n = v1.cross(v2);
  return Eigen::RowVector3d(n)/2;
  ////////////////////////////////////////////////////////////////////////////
}
