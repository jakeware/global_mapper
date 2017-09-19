#ifndef __VOXELMAP_DIRECT_INCLUDE__
#error "voxel_map.hxx should not be included directly -- Include voxel_map.hpp instead!"
#endif

template<typename T>
VoxelMap<T>::VoxelMap(const double _origin[3], const double _world_dimensions[3], const double _resolution[3], T initValue) :
    data(NULL)
{
  memcpy(world_dimensions, _world_dimensions, 3 * sizeof(double));
  memcpy(resolution, _resolution, 3 * sizeof(double));
  memcpy(origin, _origin, 3 * sizeof(double));
  
  UpdateOrigin(_origin);

  for (int i = 0; i < 3; i++) {
    grid_dimensions[i] = static_cast<int>(world_dimensions[i] / resolution[i]);
  } 

  num_cells = grid_dimensions[0] * grid_dimensions[1] * grid_dimensions[2];

  data = new T[num_cells];

  if (!data) {
    printf("(voxel_map) Error: Failed to allocate memory for map.");
    exit(1);
  }

  Reset(initValue);

  // int ixyz[3];
  // for (ixyz[2] = 0; ixyz[2] < grid_dimensions[2]; ixyz[2]++) {
  //   for (ixyz[1] = 0; ixyz[1] < grid_dimensions[1]; ixyz[1]++) {
  //     for (ixyz[0] = 0; ixyz[0] < grid_dimensions[0]; ixyz[0]++) {
  //       double xyz[3];
  //       GridToWorld(ixyz, xyz);
  //       if(xyz[0] > -2 && xyz[0] < 2 && xyz[1] > -2 && xyz[1] < 2) {
  //          WriteValue(ixyz, 1);
  //        }
  //       // if((ixyz[0]/10+ixyz[1]/10)%2==0) {
  //     }
  //   }
  // }
}

template<typename T>
template<class F>
VoxelMap<T>::VoxelMap(const VoxelMap<F> * to_copy, bool copyData, T (*transformFunc)(F))
{
  memcpy(grid_dimensions, to_copy->grid_dimensions, 3 * sizeof(int));
  memcpy(world_dimensions, to_copy->world_dimensions, 3 * sizeof(double));
  memcpy(resolution, to_copy->resolution, 3 * sizeof(double));
  memcpy(origin, to_copy->origin, 3 * sizeof(double));

  UpdateOrigin(to_copy->origin);

  num_cells = to_copy->num_cells;

  data = new T[num_cells];
  if (copyData) {
    int ixyz[3];
    for (ixyz[2] = 0; ixyz[2] < grid_dimensions[2]; ixyz[2]++) {
      for (ixyz[1] = 0; ixyz[1] < grid_dimensions[1]; ixyz[1]++) {
        for (ixyz[0] = 0; ixyz[0] < grid_dimensions[0]; ixyz[0]++) {
          if (transformFunc != NULL)
            WriteValue(ixyz, transformFunc(to_copy->ReadValue(ixyz)));
          else
            WriteValue(ixyz, to_copy->ReadValue(ixyz));
        }
      }
    }
  }
}

template<typename T>
VoxelMap<T>::~VoxelMap()
{
  if (data != NULL)
    delete[] data;
}

//get linear index into storage arrays
template<typename T>
inline int VoxelMap<T>::GridToIndex(const int ixyz[3]) const {
  int ixyz_offset[3];
  for (int i = 0; i < 3; i++) {
    ixyz_offset[i] = (ixyz[i] + origin_voxels[i]) % grid_dimensions[i];
    if(ixyz_offset[i] < 0) {
      ixyz_offset[i] += grid_dimensions[i];
    }
  }

  int index = ixyz_offset[2] * (grid_dimensions[0] * grid_dimensions[1]) + ixyz_offset[1] * grid_dimensions[0] + ixyz_offset[0];
  if(index < 0 || index >= num_cells) {
    printf("Tried to index at %i for (%i, %i, %i)\n", index, ixyz_offset[0], ixyz_offset[1], ixyz_offset[2]);
  }
  return index;
}

template<typename T>
inline int VoxelMap<T>::WorldToIndex(const double xyz[3]) const {
  int ixyz[3];
  WorldToGrid(xyz, ixyz);
  return GridToIndex(ixyz);
}

template<typename T>
inline void VoxelMap<T>::IndexToGrid(int ind, int ixyz[3]) const {
  ixyz[2] = ind / (grid_dimensions[0] * grid_dimensions[1]);
  ind -= ixyz[2] * (grid_dimensions[0] * grid_dimensions[1]);
  ixyz[1] = ind / (grid_dimensions[0]);
  ind -= ixyz[1] * grid_dimensions[0];
  ixyz[0] = ind;
}

template<typename T>
inline void VoxelMap<T>::IndexToWorld(int ind, double xyz[3]) const
{
  int ixyz[3];
  IndexToGrid(ind, ixyz);
  GridToWorld(ixyz, xyz);
}

template<typename T>
inline void VoxelMap<T>::WorldToGrid(const double xyz[3], int ixyz[3]) const
{
  for (int i = 0; i < 3; i++) {
    ixyz[i] = static_cast<int>(floor((xyz[i] - origin[i] + world_dimensions[i] * 0.5) / resolution[i]));
  }
}

template<typename T>
inline void VoxelMap<T>::GridToWorld(const int ixyz[3], double * xyz) const
{
  for (int i = 0; i < 3; i++) {
    xyz[i] = ixyz[i] * resolution[i] + (origin[i] - world_dimensions[i] * 0.5);
  }
}

template<typename T>
inline bool VoxelMap<T>::IsInMap(const int ixyz[3]) const
{
  for (int i = 0; i < 3; i++) {
    if(ixyz[i] < 0 || ixyz[i] >= grid_dimensions[i]) {
      return false;
    }
  }
  return true;
}

template<typename T>
inline bool VoxelMap<T>::IsInMap(const double xyz[3]) const
{
  int ixyz[3];
  WorldToGrid(xyz, ixyz);
  return IsInMap(ixyz);
}

template<typename T>
void VoxelMap<T>::Reset(T resetVal)
{
  if (data != NULL)
    for (int i = 0; i < num_cells; i++)
      data[i] = resetVal;
}

template<typename T>
inline void VoxelMap<T>::UpdateOrigin(const double xyz[3]) {
  int origin_voxels_diff[3] = {0, 0, 0};
  int origin_voxels_last[3] = {0, 0, 0};
  int clear_width[3] = {0, 0, 0};
  for (int i = 0; i < 3; i++) {
    origin_voxels_last[i] = static_cast<int>(origin[i]/resolution[i]);
    origin[i] = xyz[i];
    origin_voxels[i] = static_cast<int>(origin[i]/resolution[i]);
    origin_voxels_diff[i] = origin_voxels[i] - origin_voxels_last[i];
    clear_width[i] = abs(origin_voxels_diff[i]);
  }

  // clear cells
  for (int i = 0; i < 3; i++) {
    if (origin_voxels_diff[i] > 0) {
      ClearSlice(grid_dimensions[i] - 1 - clear_width[i], clear_width[i], i);
    }

    if (origin_voxels_diff[i] < 0) {
      ClearSlice(0, clear_width[i], i);
    }
  }
}

template<typename T>
inline void VoxelMap<T>::ClearSlice(const int i, const int width, const int dimension) {
  // set minimum dimensions
  int ixyz_min[3] = {0, 0, 0};
  ixyz_min[dimension] = i;

  // set max dimensions
  int ixyz_max[3] = {grid_dimensions[0] - 1, grid_dimensions[1] - 1, grid_dimensions[2] - 1};
  ixyz_max[dimension] = i + width;

  int ixyz[3];
  for (int ix = ixyz_min[0]; ix <= ixyz_max[0]; ix++) {
    for (int iy = ixyz_min[1]; iy <= ixyz_max[1]; iy++) {
      for (int iz = ixyz_min[2]; iz <= ixyz_max[2]; iz++) {
        ixyz[0] = ix;
        ixyz[1] = iy;
        ixyz[2] = iz;
        WriteValue(ixyz, 0);
      }
    }
  }
}

template<typename T>
inline T VoxelMap<T>::ReadValue(const int ixyz[3]) const
    {
  int ind = GridToIndex(ixyz);
  return data[ind];
}
template<typename T>
inline float VoxelMap<T>::ReadValue(const double xyz[3]) const
    {
  int ixyz[3];
  WorldToGrid(xyz, ixyz);
  return ReadValue(ixyz);
}
template<typename T>
inline void VoxelMap<T>::WriteValue(const int ixyz[3], T value)
{
  int ind = GridToIndex(ixyz);
  data[ind] = value;
}
template<typename T>
inline void VoxelMap<T>::WriteValue(const double xyz[3], T value)
{
  int ixyz[3];
  WorldToGrid(xyz, ixyz);
  WriteValue(ixyz, value);
}
template<typename T>
inline void VoxelMap<T>::UpdateValue(const int ixyz[3], T value, const T clamp_bounds[2])
{
  int ind = GridToIndex(ixyz);
  data[ind] += value;
  if (clamp_bounds != NULL) {
    data[ind] = clamp_value(data[ind], clamp_bounds[0], clamp_bounds[1]);
  }

}
template<typename T>
inline void VoxelMap<T>::UpdateValue(const double xyz[3], T value, const T clamp_bounds[2])
{
  int ixyz[3];
  WorldToGrid(xyz, ixyz);
  if(IsInMap(ixyz)) {
    UpdateValue(ixyz, value, clamp_bounds);
  } else {
    return;
  }
}

template<typename T>
void VoxelMap<T>::RayTrace(const int start[3], const int end[3], T miss_inc, T hit_inc, const T clamp_bounds[2])
{
  //3D Bresenham implimentation copied from:
  //http://www.cit.griffith.edu.au/~anthony/info/graphics/bresenham.procs

  int x1, y1, z1, x2, y2, z2;
  x1 = start[0];
  y1 = start[1];
  z1 = start[2];
  x2 = end[0];
  y2 = end[1];
  z2 = end[2];
  int i, dx, dy, dz, l, m, n, x_inc, y_inc, z_inc, err_1, err_2, dx2, dy2, dz2;
  int voxel[3];

  voxel[0] = x1;
  voxel[1] = y1;
  voxel[2] = z1;
  dx = x2 - x1;
  dy = y2 - y1;
  dz = z2 - z1;
  x_inc = (dx < 0) ? -1 : 1;
  l = abs(dx);
  y_inc = (dy < 0) ? -1 : 1;
  m = abs(dy);
  z_inc = (dz < 0) ? -1 : 1;
  n = abs(dz);
  dx2 = l << 1;
  dy2 = m << 1;
  dz2 = n << 1;

  if ((l >= m) && (l >= n)) {
    err_1 = dy2 - l;
    err_2 = dz2 - l;
    for (i = 0; i < l; i++) {
      UpdateValue(voxel, miss_inc, clamp_bounds);
      if (err_1 > 0) {
        voxel[1] += y_inc;
        err_1 -= dx2;
      }
      if (err_2 > 0) {
        voxel[2] += z_inc;
        err_2 -= dx2;
      }
      err_1 += dy2;
      err_2 += dz2;
      voxel[0] += x_inc;
    }
  }
  else if ((m >= l) && (m >= n)) {
    err_1 = dx2 - m;
    err_2 = dz2 - m;
    for (i = 0; i < m; i++) {
      UpdateValue(voxel, miss_inc, clamp_bounds);
      if (err_1 > 0) {
        voxel[0] += x_inc;
        err_1 -= dy2;
      }
      if (err_2 > 0) {
        voxel[2] += z_inc;
        err_2 -= dy2;
      }
      err_1 += dx2;
      err_2 += dz2;
      voxel[1] += y_inc;
    }
  }
  else {
    err_1 = dy2 - n;
    err_2 = dx2 - n;
    for (i = 0; i < n; i++) {
      UpdateValue(voxel, miss_inc, clamp_bounds);

      if (err_1 > 0) {
        voxel[1] += y_inc;
        err_1 -= dz2;
      }
      if (err_2 > 0) {
        voxel[0] += x_inc;
        err_2 -= dz2;
      }
      err_1 += dy2;
      err_2 += dx2;
      voxel[2] += z_inc;
    }
  }
}

template<typename T>
void VoxelMap<T>::RayTrace(const double start[3], const double end[3], T miss_inc, T hit_inc, const T clamp_bounds[2])
{
  int istart[3];
  int iend[3];
  WorldToGrid(start, istart);
  WorldToGrid(end, iend);
  if(IsInMap(istart) && IsInMap(iend)) {
    RayTrace(istart, iend, miss_inc, hit_inc, clamp_bounds);
  } else {
    return;
  }
}

template<typename T>
bool VoxelMap<T>::CollisionCheck(const int start[3], const int end[3], T occ_thresh, int collisionPoint[3]) const
    {
  bool collision = false;
  //3D Bresenham implimentation copied from:
  //http://www.cit.griffith.edu.au/~anthony/info/graphics/bresenham.procs
  //

  int x1, y1, z1, x2, y2, z2;
  x1 = start[0];
  y1 = start[1];
  z1 = start[2];
  x2 = end[0];
  y2 = end[1];
  z2 = end[2];
  int i, dx, dy, dz, l, m, n, x_inc, y_inc, z_inc, err_1, err_2, dx2, dy2, dz2;
  int voxel[3];

  voxel[0] = x1;
  voxel[1] = y1;
  voxel[2] = z1;
  dx = x2 - x1;
  dy = y2 - y1;
  dz = z2 - z1;
  x_inc = (dx < 0) ? -1 : 1;
  l = abs(dx);
  y_inc = (dy < 0) ? -1 : 1;
  m = abs(dy);
  z_inc = (dz < 0) ? -1 : 1;
  n = abs(dz);
  dx2 = l << 1;
  dy2 = m << 1;
  dz2 = n << 1;

  if ((l >= m) && (l >= n)) {
    err_1 = dy2 - l;
    err_2 = dz2 - l;
    for (i = 0; i < l; i++) {
      if (ReadValue(voxel) > occ_thresh) {
        collision = true;
        break;
      }
      if (err_1 > 0) {
        voxel[1] += y_inc;
        err_1 -= dx2;
      }
      if (err_2 > 0) {
        voxel[2] += z_inc;
        err_2 -= dx2;
      }
      err_1 += dy2;
      err_2 += dz2;
      voxel[0] += x_inc;
    }
  }
  else if ((m >= l) && (m >= n)) {
    err_1 = dx2 - m;
    err_2 = dz2 - m;
    for (i = 0; i < m; i++) {
      if (ReadValue(voxel) > occ_thresh) {
        collision = true;
        break;
      }
      if (err_1 > 0) {
        voxel[0] += x_inc;
        err_1 -= dy2;
      }
      if (err_2 > 0) {
        voxel[2] += z_inc;
        err_2 -= dy2;
      }
      err_1 += dx2;
      err_2 += dz2;
      voxel[1] += y_inc;
    }
  }
  else {
    err_1 = dy2 - n;
    err_2 = dx2 - n;
    for (i = 0; i < n; i++) {
      if (ReadValue(voxel) > occ_thresh) {
        collision = true;
        break;
      }

      if (err_1 > 0) {
        voxel[1] += y_inc;
        err_1 -= dz2;
      }
      if (err_2 > 0) {
        voxel[0] += x_inc;
        err_2 -= dz2;
      }
      err_1 += dy2;
      err_2 += dx2;
      voxel[2] += z_inc;
    }
  }
  if (ReadValue(voxel) > occ_thresh) {
    collision = true;
  }
  if (collisionPoint != NULL) {
    for (int i = 0; i < 3; i++) {
      collisionPoint[i] = voxel[i];
    }
  }
}

template<typename T>
bool VoxelMap<T>::CollisionCheck(const double start[3], const double end[3], T occ_thresh,
    double collisionPoint[3]) const
    {
  int istart[3];
  int iend[3];
  WorldToGrid(start, istart);
  WorldToGrid(end, iend);
  CollisionCheck(istart, iend, occ_thresh, collisionPoint);
}

template<typename T>
template<class F>
inline F VoxelMap<T>::clamp_value(F x, F min, F max) const
    {
  if (x < min)
    return min;
  if (x > max)
    return max;
  return x;
}
