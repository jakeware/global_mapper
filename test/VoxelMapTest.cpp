#include <occ_map/voxel_map.hpp>

int main(int argc, char ** argv)
{

  double world_dims[3] = { 3, 3, 1 };
  double resolution[3] = { 1, 1, 1 };
  double origin[3] = {0, 0, 0};
  int ixyz[3] = {0, 0, 0};
  int index = 0;
  double xyz[3] = {1, 1, 1};

  occ_map::FloatVoxelMap fvm(origin, world_dims, resolution, 0);
  fvm.UpdateOrigin(origin);
  fvm.WorldToGrid(xyz, ixyz);
  printf("origin: (%0.2f, %0.2f, %0.2f\n", origin[0], origin[1], origin[2]);
  printf("WorldToGrid(%0.2f, %0.2f, %0.2f) = %i, %i, %i\n", xyz[0], xyz[1], xyz[2], ixyz[0], ixyz[1], ixyz[2]);
  index = fvm.GridToIndex(ixyz);
  printf("GridToIndex(%i, %i, %i) = %i\n", ixyz[0], ixyz[1], ixyz[2], index);
  fvm.GridToWorld(ixyz, xyz);
  printf("GridToWorld(%i, %i, %i) = (%0.2f, %0.2f, %0.2f)\n", ixyz[0], ixyz[1], ixyz[2], xyz[0], xyz[1], xyz[2]);
}
