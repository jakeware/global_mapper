#ifndef __VOXELMAP_DIRECT_INCLUDE__
#error "voxel_map.hxx should not be included directly -- Include voxel_map.hpp instead!"
#endif

template<typename T>
VoxelMap<T>::VoxelMap(const double _xyz0[3], const double _xyz1[3], const double _metersPerPixel[3], T initValue,
    bool allocate_data) :
    data(NULL)
{
  memcpy(xyz0, _xyz0, 3 * sizeof(double));
  memcpy(xyz1, _xyz1, 3 * sizeof(double));
  memcpy(metersPerPixel, _metersPerPixel, 3 * sizeof(double));

  num_cells = 1;
  for (int i = 0; i < 3; i++) {
    dimensions[i] = ceil(floor(100 * (1.0 / metersPerPixel[i]) * (xyz1[i] - xyz0[i])) / 100); //multiply by 100 and take floor to avoid machine precision issues
    xyz1[i] = xyz0[i] + dimensions[i] * metersPerPixel[i]; //make top right align with pixel boundaries
    num_cells *= dimensions[i];
  }
  if (allocate_data) {
    data = new T[num_cells];
    Reset(initValue);
  }
}

template<typename T>
template<class F>
VoxelMap<T>::VoxelMap(const VoxelMap<F> * to_copy, bool copyData, T (*transformFunc)(F))
{
  memcpy(xyz0, to_copy->xyz0, 3 * sizeof(double));
  memcpy(xyz1, to_copy->xyz1, 3 * sizeof(double));
  memcpy(metersPerPixel, to_copy->metersPerPixel, 3 * sizeof(double));
  memcpy(dimensions, to_copy->dimensions, 3 * sizeof(int));

  num_cells = 1;
  for (int i = 0; i < 3; i++)
    num_cells *= dimensions[i];
  data = new T[num_cells];
  if (copyData) {
    int ixyz[3];
    for (ixyz[2] = 0; ixyz[2] < dimensions[2]; ixyz[2]++) {
      for (ixyz[1] = 0; ixyz[1] < dimensions[1]; ixyz[1]++) {
        for (ixyz[0] = 0; ixyz[0] < dimensions[0]; ixyz[0]++) {
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
inline int VoxelMap<T>::GetIndex(const int ixyz[3]) const
    {
  return ixyz[2] * (dimensions[0] * dimensions[1]) + ixyz[1] * dimensions[0] + ixyz[0];
}
template<typename T>
inline int VoxelMap<T>::GetIndex(const double xyz[3]) const
    {
  int ixyz[3];
  WorldToGrid(xyz, ixyz);
  return GetIndex(ixyz);
}

template<typename T>
inline void VoxelMap<T>::IndexToGrid(int ind, int ixyz[3]) const
    {
  ixyz[2] = ind / (dimensions[0] * dimensions[1]);
  ind -= ixyz[2] * (dimensions[0] * dimensions[1]);
  ixyz[1] = ind / (dimensions[0]);
  ind -= ixyz[1] * dimensions[0];
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
inline bool VoxelMap<T>::WorldToGrid(const double xyz[3], int ixyz[3]) const
    {
  bool clamped = false;
  for (int i = 0; i < 3; i++) {
    ixyz[i] = round((xyz[i] - xyz0[i]) / metersPerPixel[i]);
    if (ixyz[i] <= 0 || ixyz[i] >= dimensions[i] - 1) {
      ixyz[i] = clamp_value(ixyz[i], 0, dimensions[i] - 1);
      clamped = true;
    }
  }
  return clamped;
}

template<typename T>
inline void VoxelMap<T>::GridToWorld(const int ixyz[3], double * xyz) const
    {
  for (int i = 0; i < 3; i++)
    xyz[i] = ((double) ixyz[i]) * metersPerPixel[i] + xyz0[i];
}

template<typename T>
inline bool VoxelMap<T>::IsInMap(const int ixyz[3]) const
    {
  for (int i = 0; i < 3; i++)
    if (ixyz[i] < 0 || ixyz[i] >= dimensions[i])
      return false;
  return true;
}
template<typename T>
inline bool VoxelMap<T>::IsInMap(const double xyz[3]) const
    {
  for (int i = 0; i < 3; i++)
    if (xyz[i] < xyz0[i] || xyz[i] > xyz1[i])
      return false;
  return true;
}

template<typename T>
void VoxelMap<T>::Reset(T resetVal)
{
  if (data != NULL)
    for (int i = 0; i < num_cells; i++)
      data[i] = resetVal;
}

template<typename T>
inline T VoxelMap<T>::ReadValue(const int ixyz[3]) const
    {
  int ind = GetIndex(ixyz);
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
  int ind = GetIndex(ixyz);
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
  int ind = GetIndex(ixyz);
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
  UpdateValue(ixyz, value, clamp_bounds);
}

template<typename T>
void VoxelMap<T>::RayTrace(const int start[3], const int end[3], T miss_inc, T hit_inc, const T clamp_bounds[2])
{
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
  RayTrace(istart, iend, miss_inc, hit_inc, clamp_bounds);
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
