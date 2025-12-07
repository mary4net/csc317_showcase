#ifndef MESH_BUILDERS_H
#define MESH_BUILDERS_H

#include "mesh_types.h"

Mesh build_room_mesh();
Mesh build_table_mesh();
Mesh build_cube_mesh(double size);
Mesh build_mirror_mesh(double w, double h);

#endif
