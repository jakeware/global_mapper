#include <occ_map/PixelMap.hpp>

int main(int argc, char ** argv){
  double xyz0[2] = {-20, -20};
  double xyz1[2] = {20, 20};
  double mpp = .2;
  occ_map::FloatPixelMap fvm(xyz0,xyz1,mpp,0);
  
  double xy[2];
  for (xy[1] = -5; xy[1] < 10; xy[1] += 0.2) {
      for (xy[0] = -5; xy[0] < 5; xy[0] += 0.2) {
          fvm.writeValue (xy, 0.99);
      }
  }

  double xy0[2] = {0, 0};
  double xyR[2] = {0, 5};
  for (double x = -5; x < 5; x += 0.5) {
      xyR[0] = x;
      fvm.rayTrace (xy0, xyR, 1, 0.3);
  }

#ifndef NO_LCM
  const occ_map_pixel_map_t *msg = fvm.get_pixel_map_t (0);
  lcm_t * lcm = lcm_create (NULL);
  occ_map_pixel_map_t_publish (lcm, "PIXEL_MAP", msg);
#endif
}
