#include "write_ppm.h"
#include <fstream>
#include <cassert>
#include <iostream>
#include <ostream>

bool write_ppm(const std::string &filename,
               const std::vector<unsigned char> &data, const int width,
               const int height, const int num_channels) {
  assert((num_channels == 3 || num_channels == 1) &&
         ".ppm only supports RGB or grayscale images");
  ////////////////////////////////////////////////////////////////////////////
  // Replace with your code here:
  std::ofstream file(filename);
  if (!file.is_open()) {
    return false;
  }
  file << "P3\n" << width << " " << height << "\n255\n";
  if (num_channels == 3) {
    for (int i = 0; i < 3 * width * height; i += 3) {
      file << (int)data[i] << " " << (int)data[i + 1] << " " << (int)data[i + 2]
           << "\n";
    }
  } else {
    for (int i = 0; i < width * height; i++) {
      unsigned char color = data[i];
      file << (int)color << " " << (int)color << " " << (int)color << "\n";
    }
  }
  return true;
  ////////////////////////////////////////////////////////////////////////////
}
