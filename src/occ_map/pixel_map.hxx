#ifndef __PIXELMAP_DIRECT_INCLUDE__
#error "pixel_map.hxx should not be included directly -- Include pixel_map.hpp instead!"
#endif

template<typename T>
PixelMap<T>::PixelMap(const double _xy0[2], const double _xy1[2], double mPP, T initValue, bool allocate_data,
    bool align_to_pixels) :
    metersPerPixel(mPP),  data(NULL)
{
  if (align_to_pixels) {
    // make bottom right align with pixels
    xy0[0] = floor((1.0 / metersPerPixel) * _xy0[0]) * metersPerPixel;
    xy0[1] = floor((1.0 / metersPerPixel) * _xy0[1]) * metersPerPixel;
  }
  else {
    xy0[0] = _xy0[0];
    xy0[1] = _xy0[1];
  }

  dimensions[0] = ceil(floor(100 * (1.0 / metersPerPixel) * (_xy1[0] - xy0[0])) / 100); //multiply by 100 and take floor to avoid machine
  dimensions[1] = ceil(floor(100 * (1.0 / metersPerPixel) * (_xy1[1] - xy0[1])) / 100); //precision causing different sized maps

  //make top right align with pixels
  xy1[0] = xy0[0] + dimensions[0] * metersPerPixel;
  xy1[1] = xy0[1] + dimensions[1] * metersPerPixel;

  if (dimensions[0] <= 0 || dimensions[1] < 0) {
    printf("ERROR:dimensions[0] or dimensions[1] is less than 0\n");
    return;
  }
  num_cells = dimensions[0] * dimensions[1];
  if (allocate_data) {
    data = new T[num_cells];
    Reset(initValue);
  }
}

/*
 * Copy Constructor
 */
template<typename T>
template<class F>
PixelMap<T>::PixelMap(const PixelMap<F> * to_copy, bool copyData, T (*transformFunc)(F)) :
    metersPerPixel(to_copy->metersPerPixel), data(NULL)
{
  memcpy(xy0, to_copy->xy0, 2 * sizeof(double));
  memcpy(xy1, to_copy->xy1, 2 * sizeof(double));

  memcpy(dimensions, to_copy->dimensions, 2 * sizeof(int));
  num_cells = to_copy->num_cells;
  data = new T[num_cells];
  if (copyData) {
    int ixy[2];
    for (ixy[1] = 0; ixy[1] < dimensions[1]; ixy[1]++) {
      for (ixy[0] = 0; ixy[0] < dimensions[0]; ixy[0]++) {
        if (transformFunc != NULL)
          WriteValue(ixy, transformFunc(to_copy->ReadValue(ixy)));
        else
          WriteValue(ixy, to_copy->ReadValue(ixy));
      }
    }
  }
}

template<typename T>
PixelMap<T>::~PixelMap()
{
  if (data != NULL)
    delete[] data;
}

template<typename T>
void PixelMap<T>::Reset(T resetVal)
{
  if (data != NULL)
    for (int i = 0; i < num_cells; i++)
      data[i] = resetVal;
}

template<typename T>
inline int PixelMap<T>::GridToIndex(const int ixy[2]) const
    {
  return ixy[1] * dimensions[0] + ixy[0];
}

template<typename T>
inline int PixelMap<T>::WorldToIndex(const double xy[2]) const
    {
  int ixy[2];
  WorldToGrid(xy, ixy);
  return GridToIndex(ixy);
}

template<typename T>
inline void PixelMap<T>::IndexToGrid(int ind, int ixy[2]) const
    {
  ixy[1] = ind / (dimensions[0]);
  ind -= ixy[1] * dimensions[0];
  ixy[0] = ind;
}
template<typename T>
inline void PixelMap<T>::IndexToWorld(int ind, double xy[2]) const
    {
  int ixy[2];
  IndexToGrid(ind, ixy);
  GridToWorld(ixy, xy);
}

template<typename T>
inline void PixelMap<T>::WorldToGrid(const double xy[2], int ixy[2]) const
    {
  ixy[0] = clamp_value(round((xy[0] - xy0[0]) / metersPerPixel), 0., (double) (dimensions[0] - 1));
  ixy[1] = clamp_value(round((xy[1] - xy0[1]) / metersPerPixel), 0., (double) (dimensions[1] - 1));
}

template<typename T>
inline void PixelMap<T>::GridToWorld(const int ixy[2], double xy[2]) const
    {
  //    *xy[0] = ((double)ixy[0]+0.5) * metersPerPixel + xy0[0]; //+.5 puts it in the center of the cell
  //    *xy[1] = ((double)ixy[1]+0.5) * metersPerPixel + xy0[1];
  xy[0] = ((double) ixy[0]) * metersPerPixel + xy0[0];
  xy[1] = ((double) ixy[1]) * metersPerPixel + xy0[1];

}
template<typename T>
inline bool PixelMap<T>::IsInMap(const int ixy[2]) const
    {
  if (ixy[0] < 0 || ixy[1] < 0)
    return false;
  else if (ixy[0] >= dimensions[0] || ixy[1] >= dimensions[1])
    return false;
  else
    return true;
}

template<typename T>
inline bool PixelMap<T>::IsInMap(const double xy[2]) const
{
  int ixy[2];
  WorldToGrid(xy, ixy);
  return IsInMap(ixy);
}

template<typename T>
inline T & PixelMap<T>::ReadValue(const int ixy[2]) const
    {
  int ind = GridToIndex(ixy);
  return data[ind];

}
template<typename T>
inline T & PixelMap<T>::ReadValue(const double xy[2]) const
    {
  int ixy[2];
  WorldToGrid(xy, ixy);
  return ReadValue(ixy);
}
template<typename T>
inline void PixelMap<T>::WriteValue(const int ixy[2], T val)
{
  int ind = GridToIndex(ixy);
  data[ind] = val;
}

template<typename T>
inline void PixelMap<T>::WriteValue(const double xy[2], T val)
{
  int ixy[2];
  WorldToGrid(xy, ixy);
  WriteValue(ixy, val);
}

template<typename T>
inline void PixelMap<T>::UpdateValue(const int ixy[2], T inc, T clamp_bounds[2])
{
  int ind = GridToIndex(ixy);
  data[ind] += inc;
  if (clamp_bounds != NULL) {
    data[ind] = clamp_value(data[ind], clamp_bounds[0], clamp_bounds[1]);
  }
}

template<typename T>
inline void PixelMap<T>::UpdateValue(const double xy[2], T inc, T clamp_bounds[2])
{
  int ixy[2];
  WorldToGrid(xy, ixy);
  UpdateValue(ixy, inc, clamp_bounds);
}

template<typename T>
inline void PixelMap<T>::RayTrace(const double start[2], const double end[2], T miss_inc, T hit_inc, T clamp_bounds[2])
{
  int istart[2], iend[2];
  WorldToGrid(start, istart);
  WorldToGrid(end, iend);
  RayTrace(istart, iend, miss_inc, hit_inc, clamp_bounds);
}

/**
 * This function adapted from the Python Imaging Library
 */
template<typename T>
void PixelMap<T>::RayTrace(const int start[2], const int end[2], T miss_inc, T hit_inc, T clamp_bounds[2])
{
  int curr[2] = { start[0], start[1] };

  // normalize
  int xstep = 1;
  int ystep = 1;
  int dx = end[0] - start[0];
  if (dx < 0) {
    dx = -dx;
    xstep = -1;
  }
  int dy = end[1] - start[1];
  if (dy < 0) {
    dy = -dy;
    ystep = -1;
  }

  if (dx == 0) {
    // vertical
    for (int i = 0; i <= dy; i++) {
      int wasHit = curr[0] == end[0] && curr[1] == end[1];
      UpdateValue(curr, wasHit * hit_inc + (1 - wasHit) * miss_inc, clamp_bounds);
      curr[1] = curr[1] + ystep;
    }
  }
  else if (dy == 0) {
    // horizontal
    for (int i = 0; i <= dx; i++) {
      int wasHit = curr[0] == end[0] && curr[1] == end[1];
      UpdateValue(curr, wasHit * hit_inc + (1 - wasHit) * miss_inc, clamp_bounds);
      curr[0] += xstep;
    }
  }
  else if (dx > dy) {
    // bresenham, horizontal slope
    int n = dx;
    dy += dy;
    int e = dy - dx;
    dx += dx;

    for (int i = 0; i <= n; i++) {
      int wasHit = curr[0] == end[0] && curr[1] == end[1];
      UpdateValue(curr, wasHit * hit_inc + (1 - wasHit) * miss_inc, clamp_bounds);
      if (e >= 0) {
        curr[1] += ystep;
        e -= dx;
      }
      e += dy;
      curr[0] += xstep;
    }
  }
  else {
    // bresenham, vertical slope
    int n = dy;
    dx += dx;
    int e = dx - dy;
    dy += dy;

    for (int i = 0; i <= n; i++) {
      int wasHit = curr[0] == end[0] && curr[1] == end[1];
      UpdateValue(curr, wasHit * hit_inc + (1 - wasHit) * miss_inc, clamp_bounds);
      if (e >= 0) {
        curr[0] += xstep;
        e -= dy;
      }
      e += dx;
      curr[1] += ystep;
    }
  }
}

template<typename T>
inline bool PixelMap<T>::CollisionCheck(const double start[2], const double end[2], T occ_thresh,
    double collisionPoint[2]) const
    {
  int istart[2], iend[2], icollision[2];
  WorldToGrid(start, istart);
  WorldToGrid(end, iend);
  bool collision = CollisionCheck(istart, iend, occ_thresh, icollision);
  if (collision && collisionPoint != NULL)
    GridToWorld(icollision, collisionPoint);
  return collision;
}

template<typename T>
bool PixelMap<T>::CollisionCheck(const int start[2], const int end[2], T occ_thresh, int collisionPoint[2]) const
    {
  int curr[2] = { start[0], start[1] };
  bool collision = false;

  // normalize
  int xstep = 1;
  int ystep = 1;
  int dx = end[0] - start[0];
  if (dx < 0) {
    dx = -dx;
    xstep = -1;
  }
  int dy = end[1] - start[1];
  if (dy < 0) {
    dy = -dy;
    ystep = -1;
  }

  if (dx == 0) {
    // vertical
    for (int i = 0; i <= dy; i++) {
      if (ReadValue(curr) > occ_thresh) {
        collision = true;
        break;
      }
      curr[1] = curr[1] + ystep;
    }
  }
  else if (dy == 0) {
    // horizontal
    for (int i = 0; i <= dx; i++) {
      if (ReadValue(curr) > occ_thresh) {
        collision = true;
        break;
      }
      curr[0] += xstep;
    }
  }
  else if (dx > dy) {
    // bresenham, horizontal slope
    int n = dx;
    dy += dy;
    int e = dy - dx;
    dx += dx;

    for (int i = 0; i <= n; i++) {
      if (ReadValue(curr) > occ_thresh) {
        collision = true;
        break;
      }
      if (e >= 0) {
        curr[1] += ystep;
        e -= dx;
      }
      e += dy;
      curr[0] += xstep;
    }
  }
  else {
    // bresenham, vertical slope
    int n = dy;
    dx += dx;
    int e = dx - dy;
    dy += dy;

    for (int i = 0; i <= n; i++) {
      if (ReadValue(curr) > occ_thresh) {
        collision = true;
        break;
      }
      if (e >= 0) {
        curr[0] += xstep;
        e -= dy;
      }
      e += dx;
      curr[1] += ystep;
    }
  }

  if (collisionPoint != NULL) {
    collisionPoint[0] = curr[0];
    collisionPoint[1] = curr[1];
  }
  return collision;
}

template<typename T>
template<class F>
inline F PixelMap<T>::clamp_value(F x, F min, F max) const
    {
  if (x < min)
    return min;
  if (x > max)
    return max;
  return x;
}
