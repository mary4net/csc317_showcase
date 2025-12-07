#include "vertex_triangle_adjacency.h"

void vertex_triangle_adjacency(
  const Eigen::MatrixXi & F,
  const int num_vertices,
  std::vector<std::vector<int> > & VF)
{
  VF.resize(num_vertices);
  ////////////////////////////////////////////////////////////////////////////
  // Add your code here:
  for (int f = 0; f < F.rows(); f++) {
    for (int j = 0; j < 3; j++) {
      int v = F(f, j);
      VF[v].push_back(f);
    }
  }
  ////////////////////////////////////////////////////////////////////////////
}

