#ifndef MESH_TYPES_H
#define MESH_TYPES_H

#include <Eigen/Core>

struct Mesh {
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
};

enum class MaterialKind { Default, Metal, Mirror };

struct MeshInstance {
  Mesh mesh;
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Vector3d base_color = Eigen::Vector3d(0.6, 0.7, 0.9);
  MaterialKind material = MaterialKind::Default;
  bool subdividable = false;
};

#endif
