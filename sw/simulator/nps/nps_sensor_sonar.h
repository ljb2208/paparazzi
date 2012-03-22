#ifndef NPS_SENSOR_SONAR_H
#define NPS_SENSOR_SONAR_H

#include "math/pprz_algebra.h"
#include "math/pprz_algebra_double.h"
#include "math/pprz_algebra_float.h"
#include "std.h"

struct NpsSensorSonar {
  double  value;
  double  next_update;
  bool_t  data_available;
};


extern void nps_sensor_sonar_init(struct NpsSensorSonar* sonar, double time);
extern void nps_sensor_sonar_run_step(struct NpsSensorSonar* sonar, double time);

#endif /* NPS_SENSOR_SONAR_H */
