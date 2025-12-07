#include "catmull_clark.h"
#include <Eigen/Core>
#include <algorithm>
#include <map>
#include <unordered_map>
#include <utility>
#include <functional>
#include <vector>

void catmull_clark(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F,
                   const int num_iters, Eigen::MatrixXd &SV,
                   Eigen::MatrixXi &SF) {
  ////////////////////////////////////////////////////////////////////////////
  // Replace with your code here:
  if (num_iters == 0) {
    SV = V;
    SF = F;
    return;
  }

  Eigen::MatrixXd mid_SV;
  Eigen::MatrixXi mid_SF;

  Eigen::MatrixXd face_points(F.rows(), 3);
  for (int i = 0; i < F.rows(); i++) {
    face_points.row(i) =
        (V.row(F(i, 0)) + V.row(F(i, 1)) + V.row(F(i, 2)) + V.row(F(i, 3))) / 4;
  }
  std::map<std::pair<int, int>, std::vector<int>> edge2face;
  for (int i = 0; i < F.rows(); i++) {
    for (int j = 0; j < 4; j++) {
      int idx_a = F(i, j);
      int idx_b = F(i, (j + 1) % 4);
      std::pair<int, int> ordered =
          std::pair<int, int>(std::min(idx_a, idx_b), std::max(idx_a, idx_b));
      edge2face[ordered].push_back(i);
    }
  }

  std::map<std::pair<int, int>, int> edge_idx;
  std::vector<Eigen::RowVector3d> edge_positions;
  for (auto kv : edge2face) {
    int a = kv.first.first;
    int b = kv.first.second;
    const auto &faces = kv.second;
    Eigen::RowVector3d e = (V.row(a) + V.row(b));
    if (faces.size() == 2)
      e = (e + face_points.row(faces[0]) + face_points.row(faces[1])) / 4.0;
    else
      e /= 2.0;
    edge_idx[kv.first] = static_cast<int>(edge_positions.size());
    edge_positions.push_back(e);
  }

  std::vector<std::vector<int>> VF(V.rows());
  std::vector<std::vector<int>> VE(V.rows());
  for (int i = 0; i < F.rows(); i++) {
    for (int j = 0; j < 4; j++) {
      int a = F(i, j);
      VF[a].push_back(i);
      std::pair<int, int> ordered =
          std::pair<int, int>(std::min(F(i, j), F(i, (j + 1) % 4)),
                              std::max(F(i, j), F(i, (j + 1) % 4)));
      VE[a].push_back(edge_idx[ordered]);
    }
  }

  std::vector<Eigen::RowVector3d> new_vertices(
      V.rows(), Eigen::RowVector3d::Zero());
  for (int v = 0; v < V.rows(); v++) {
    int n = VF[v].size();
    if (n == 0) {
      new_vertices[v] = V.row(v);
      continue;
    }
    Eigen::RowVector3d Fsum = Eigen::RowVector3d::Zero(),
                       Rsum = Eigen::RowVector3d::Zero();
    for (int f : VF[v])
      Fsum += face_points.row(f);
    for (int e : VE[v])
      Rsum += edge_positions[e];
    const double n_double = static_cast<double>(n);
    const Eigen::RowVector3d Favg = Fsum / n_double;
    const Eigen::RowVector3d Ravg = Rsum / n_double;
    new_vertices[v] =
        (Favg + 2.0 * Ravg + (n_double - 3.0) * V.row(v)) / n_double;
  }

  int n_old = V.rows();
  int n_edge = edge_positions.size();
  int n_face = F.rows();
  mid_SV.resize(n_old + n_edge + n_face, 3);
  for (int i = 0; i < n_old; i++)
    mid_SV.row(i) = new_vertices[i];
  for (int i = 0; i < n_edge; i++)
    mid_SV.row(n_old + i) = edge_positions[i];
  for (int i = 0; i < n_face; i++)
    mid_SV.row(n_old + n_edge + i) = face_points.row(i);

  // 6. 构建新面
  mid_SF.resize(F.rows() * 4, 4);
  for (int i = 0; i < F.rows(); i++) {
    int a = F(i, 0), b = F(i, 1), c = F(i, 2), d = F(i, 3);
    std::pair<int, int> ordered =
        std::pair<int, int>(std::min(a, b), std::max(a, b));
    int e_ab = n_old + edge_idx[ordered];

    ordered = std::pair<int, int>(std::min(b, c), std::max(b, c));
    int e_bc = n_old + edge_idx[ordered];
    ordered = std::pair<int, int>(std::min(c, d), std::max(c, d));
    int e_cd = n_old + edge_idx[ordered];
    ordered = std::pair<int, int>(std::min(a, d), std::max(a, d));
    int e_da = n_old + edge_idx[ordered];
    int fpt = n_old + n_edge + i;
    mid_SF.row(4 * i + 0) << a, e_ab, fpt, e_da;
    mid_SF.row(4 * i + 1) << e_ab, b, e_bc, fpt;
    mid_SF.row(4 * i + 2) << fpt, e_bc, c, e_cd;
    mid_SF.row(4 * i + 3) << e_da, fpt, e_cd, d;
  }
  catmull_clark(mid_SV, mid_SF, num_iters - 1, SV, SF);
  ////////////////////////////////////////////////////////////////////////////
}
