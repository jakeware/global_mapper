// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

#include <cstring>

namespace occ_map {

template<class T>
class PixelMap {
 public:
  // normal constructor
  PixelMap<T>(const double _xy0[2], const double _xy1[2], double mPP, T initValue = T(), bool allocate_data = true, bool align_to_pixels = true);

  // Copy Constructor
  template<class F>
  PixelMap<T>(const PixelMap<F> * to_copy, bool copyData=true, T(*transformFunc)(F) = NULL);
  ~PixelMap<T>();

  // set all values in the map to 0
  void Reset(T resetVal = T());

  // Map backwards from an index to a location
  inline void IndexToGrid(int ind, int ixy[2]) const;
  inline void IndexToWorld(int ind, double xy[2]) const;

  // convert from world coordinates into the map
  inline void WorldToGrid(const double xy[2], int ixy[2]) const;
  // convert from the map coordinates to world coordinates
  inline void GridToWorld(const int ixy[2], double xy[2]) const;

  // check whether a location is inside the map bounds
  inline bool IsInMap(const int ixy[2]) const;
  inline bool IsInMap(const double xy[2]) const;

  //read the value contained in this cell
  inline T & ReadValue(const int ixy[2]) const;
  inline T & ReadValue(const double xy[2]) const;

  //write the value in the cell
  inline void WriteValue(const int ixy[2], T val);
  inline void WriteValue(const double xy[2], T val);

  //add the value to the cell with optional value clamping
  inline void UpdateValue(const int ixy[2], T inc, T clamp_bounds[2] = NULL);
  inline void UpdateValue(const double xy[2], T inc, T clamp_bounds[2] = NULL);

  //step along the line segment from start to end, updating with miss_inc along the way, and update by hit_inc at end
  void RayTrace(const int start[2], const int end[2], T miss_inc, T hit_inc, T clamp_bounds[2] = NULL);
  void RayTrace(const double start[2], const double end[2], T miss_inc, T hit_inc, T clamp_bounds[2] = NULL);

  //check whether any of the cells between start and end are greater than occ_thresh
  bool CollisionCheck(const int start[2], const int end[2], T occ_thresh, int collisionPoint[2] = NULL) const;
  bool CollisionCheck(const double start[2], const double end[2], T occ_thresh, double collisionPoint[2] = NULL) const;

 private:
  template<class F>
  inline F clamp_value(F x, F min, F max) const;

  //get linear index into storage arrays
  inline int GridToIndex(const int ixy[2]) const;
  inline int WorldToIndex(const double xy[2]) const;

  //metadata
  double xy0[2], xy1[2];
  double metersPerPixel;
  int dimensions[2];
  int num_cells;
  // the actual storage array
  T* data;
};

//typedefs for ease of use
using FloatPixelMap = PixelMap<float>;
using IntPixelMap = PixelMap<int32_t>;
using Uint8PixelMap = PixelMap<uint8_t>;

//include the actual implimentations
#define __PIXELMAP_DIRECT_INCLUDE__
#include "pixel_map.hxx"

}  // namespace occ_map
