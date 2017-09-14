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
class VoxelMap {
 public:
  // Normal Constructor
  VoxelMap<T>(const double _origin[3], const double _world_dimensions[3], const double _resolution[3], T initValue = T());

  // Copy Constructor
  template<class F>
  VoxelMap<T>(const VoxelMap<F> * to_copy, bool copyData = true, T (*transformFunc)(F) = NULL);
  ~VoxelMap<T>();

  // set all values in the map to 0
  void Reset(T resetVal = T());

  // convert from world coordinates into the map
  inline void WorldToGrid(const double xyz[3], int ixyz[3]) const;

  // convert from the map coordinates to world coordinates
  inline void GridToWorld(const int ixyz[3], double * xyz) const;

  // check whether a location is inside the map bounds
  inline bool IsInMap(const int ixyz[3]) const;
  inline bool IsInMap(const double xyz[3]) const;

  //read the value contained in this cell
  inline T ReadValue(const int ixyz[3]) const;
  inline float ReadValue(const double xyz[3]) const;

  //write the value in the cell
  inline void WriteValue(const int ixyz[3], T value);
  inline void WriteValue(const double xyz[3], T value);

  //add the value to the cell with optional value clamping
  inline void UpdateValue(const int ixyz[3], T value, const T clamp_bounds[2] = NULL);
  inline void UpdateValue(const double xyz[3], T value, const T clamp_bounds[2] = NULL);

  //step along the line segment from start to end, updating with miss_inc along the way, and update by hit_inc at end
  void RayTrace(const int start[3], const int end[3], T miss_inc, T hit_inc, const T clamp_bounds[2] = NULL);
  void RayTrace(const double start[3], const double end[3], T miss_inc, T hit_inc, const T clamp_bounds[2] = NULL);

  //check whether any of the cells between start and end are greater than occ_thresh
  bool CollisionCheck(const int start[3], const int end[3], T occ_thresh, int collisionPoint[3] = NULL) const;
  bool CollisionCheck(const double start[3], const double end[3], T occ_thresh, double collisionPoint[3] = NULL) const;

 private:
  template<class F>
  inline F clamp_value(F x, F min, F max) const;

  //get linear index into storage arrays
  inline int GetIndex(const int ixyz[3]) const;
  inline int GetIndex(const double xyz[3]) const;

  // Map backwards from an index to a location
  inline void IndexToGrid(int ind, int ixyz[3]) const;
  inline void IndexToWorld(int ind, double xyz[3]) const;
  
  //metadata
  double origin[3];
  double world_dimensions[3];
  double resolution[3];
  int grid_dimensions[3]; //map bounds in grid coordinates
  int num_cells;

  //the actual storage arrays
  T * data;
};

using FloatVoxelMap = VoxelMap<float>;
using IntVoxelMap = VoxelMap<int32_t>;
using Uint8VoxelMap = VoxelMap<uint8_t>;
  
//include the actual implimentations
#define __VOXELMAP_DIRECT_INCLUDE__
#include "voxel_map.hxx"

} // namespace occ_map
