#include "write_obj.h"
#include <fstream>
#include <cassert>
#include <iostream>

bool write_obj(
  const std::string & filename,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  const Eigen::MatrixXd & UV,
  const Eigen::MatrixXi & UF,
  const Eigen::MatrixXd & NV,
  const Eigen::MatrixXi & NF)
{
  assert((F.size() == 0 || F.cols() == 3 || F.cols() == 4) && "F must have 3 or 4 columns");
  ////////////////////////////////////////////////////////////////////////////
  // Add your code here:
  std::ofstream file(filename);
  if (!file.is_open()) {
    return false;
  }

  // V
  for (int i=0; i<V.rows(); i++){
    file << "v";
    for (int j=0; j<V.cols(); j++) {
      file << " " << V(i,j);
    }
    file << "\n";
  }
  // UV
  for (int i=0; i<UV.rows(); i++){
    file << "vt";
    for (int j=0; j<UV.cols(); j++) {
      file << " " << UV(i,j);
    }
    file << "\n";
  }
  // NV
  for (int i=0; i<NV.rows(); i++){
    file << "vn";
    for (int j=0; j<NV.cols(); j++) {
      file << " " << NV(i,j);
    }
    file << "\n";
  }
  // F
  for (int i=0; i<F.rows(); i++){
    file << "f ";
    for (int j=0; j<F.cols(); j++) {
      file << F(i,j)+1 << "/" << UF(i,j)+1 << "/" << NF(i,j)+1 << " ";
    }
    file << "\n";
  }
  return true;
  ////////////////////////////////////////////////////////////////////////////
}
