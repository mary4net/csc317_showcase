#include "mesh_builders.h"
#include <vector>

static int add_vertex(std::vector<Eigen::Vector3d> &V,
                      const Eigen::Vector3d &p) {
  V.push_back(p);
  return static_cast<int>(V.size()) - 1;
}

static void add_quad(std::vector<Eigen::Vector4i> &F, int a, int b, int c,
                     int d) {
  F.emplace_back(Eigen::Vector4i(a, b, c, d));
}

static void add_box(std::vector<Eigen::Vector3d> &V,
                    std::vector<Eigen::Vector4i> &F,
                    const Eigen::Vector3d &min_corner,
                    const Eigen::Vector3d &max_corner) {
  const int b0 = add_vertex(V, {min_corner.x(), min_corner.y(), min_corner.z()});
  const int b1 = add_vertex(V, {max_corner.x(), min_corner.y(), min_corner.z()});
  const int b2 = add_vertex(V, {max_corner.x(), min_corner.y(), max_corner.z()});
  const int b3 = add_vertex(V, {min_corner.x(), min_corner.y(), max_corner.z()});
  const int b4 = add_vertex(V, {min_corner.x(), max_corner.y(), min_corner.z()});
  const int b5 = add_vertex(V, {max_corner.x(), max_corner.y(), min_corner.z()});
  const int b6 = add_vertex(V, {max_corner.x(), max_corner.y(), max_corner.z()});
  const int b7 = add_vertex(V, {min_corner.x(), max_corner.y(), max_corner.z()});
  add_quad(F, b0, b1, b2, b3); // bottom
  add_quad(F, b4, b7, b6, b5); // top
  add_quad(F, b0, b4, b5, b1); // front
  add_quad(F, b1, b5, b6, b2); // right
  add_quad(F, b2, b6, b7, b3); // back
  add_quad(F, b3, b7, b4, b0); // left
}

static Mesh finalize_mesh(const std::vector<Eigen::Vector3d> &Vvec,
                          const std::vector<Eigen::Vector4i> &Fvec) {
  Mesh mesh;
  mesh.V.resize(static_cast<int>(Vvec.size()), 3);
  mesh.F.resize(static_cast<int>(Fvec.size()), 4);
  for (int i = 0; i < static_cast<int>(Vvec.size()); ++i) {
    mesh.V.row(i) = Vvec[i];
  }
  for (int i = 0; i < static_cast<int>(Fvec.size()); ++i) {
    mesh.F.row(i) = Fvec[i];
  }
  return mesh;
}

Mesh build_room_mesh() {
  std::vector<Eigen::Vector3d> V;
  std::vector<Eigen::Vector4i> F;

  const int v0 = add_vertex(V, {-3.0, 0.0, -3.0});
  const int v1 = add_vertex(V, {3.0, 0.0, -3.0});
  const int v2 = add_vertex(V, {3.0, 0.0, 3.0});
  const int v3 = add_vertex(V, {-3.0, 0.0, 3.0});
  const int v4 = add_vertex(V, {-3.0, 3.0, -3.0});
  const int v5 = add_vertex(V, {3.0, 3.0, -3.0});
  const int v6 = add_vertex(V, {3.0, 3.0, 3.0});
  const int v7 = add_vertex(V, {-3.0, 3.0, 3.0});

  add_quad(F, v0, v3, v2, v1); // floor
  add_quad(F, v4, v6, v7, v5); // ceiling
  add_quad(F, v0, v1, v5, v4); // front
  add_quad(F, v3, v7, v6, v2); // back
  add_quad(F, v0, v4, v7, v3); // left
  add_quad(F, v1, v2, v6, v5); // right

  return finalize_mesh(V, F);
}

Mesh build_table_mesh() {
  std::vector<Eigen::Vector3d> V;
  std::vector<Eigen::Vector4i> F;

  const double top_y0 = 1.0;
  const double top_y1 = 1.15;
  add_box(V, F, Eigen::Vector3d(-0.9, top_y0, -0.6),
          Eigen::Vector3d(0.9, top_y1, 0.6));

  const double leg_w = 0.12;
  const double leg_y1 = top_y0;
  add_box(V, F, Eigen::Vector3d(-0.9, 0.0, -0.6),
          Eigen::Vector3d(-0.9 + leg_w, leg_y1, -0.6 + leg_w));
  add_box(V, F, Eigen::Vector3d(0.9 - leg_w, 0.0, -0.6),
          Eigen::Vector3d(0.9, leg_y1, -0.6 + leg_w));
  add_box(V, F, Eigen::Vector3d(-0.9, 0.0, 0.6 - leg_w),
          Eigen::Vector3d(-0.9 + leg_w, leg_y1, 0.6));
  add_box(V, F, Eigen::Vector3d(0.9 - leg_w, 0.0, 0.6 - leg_w),
          Eigen::Vector3d(0.9, leg_y1, 0.6));

  return finalize_mesh(V, F);
}

Mesh build_cube_mesh(double size) {
  std::vector<Eigen::Vector3d> V;
  std::vector<Eigen::Vector4i> F;
  const double s = size * 0.5;
  const int v0 = add_vertex(V, {-s, -s, -s});
  const int v1 = add_vertex(V, {s, -s, -s});
  const int v2 = add_vertex(V, {s, s, -s});
  const int v3 = add_vertex(V, {-s, s, -s});
  const int v4 = add_vertex(V, {-s, -s, s});
  const int v5 = add_vertex(V, {s, -s, s});
  const int v6 = add_vertex(V, {s, s, s});
  const int v7 = add_vertex(V, {-s, s, s});

  add_quad(F, v0, v1, v2, v3);
  add_quad(F, v1, v5, v6, v2);
  add_quad(F, v5, v4, v7, v6);
  add_quad(F, v4, v0, v3, v7);
  add_quad(F, v3, v2, v6, v7);
  add_quad(F, v4, v5, v1, v0);

  return finalize_mesh(V, F);
}

Mesh build_mirror_mesh(double w, double h) {
  std::vector<Eigen::Vector3d> V;
  std::vector<Eigen::Vector4i> F;
  const double hw = w * 0.5;
  const double hh = h * 0.5;
  const int v0 = add_vertex(V, {-hw, -hh, 0.0});
  const int v1 = add_vertex(V, {hw, -hh, 0.0});
  const int v2 = add_vertex(V, {hw, hh, 0.0});
  const int v3 = add_vertex(V, {-hw, hh, 0.0});
  add_quad(F, v0, v1, v2, v3);
  return finalize_mesh(V, F);
}
