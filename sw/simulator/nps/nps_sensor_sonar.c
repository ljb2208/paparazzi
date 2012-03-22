#include "nps_sensor_sonar.h"

#include "generated/airframe.h"

#include "std.h"
#include "nps_fdm.h"
#include "nps_random.h"
#include NPS_SENSORS_PARAMS

void nps_sensor_sonar_init(struct NpsSensorSonar* sonar, double time) {
	sonar->value = 0.;
	sonar->next_update = time;
	sonar->data_available = FALSE;
}


void nps_sensor_sonar_run_step(struct NpsSensorSonar* sonar, double time) {


  if (time < sonar->next_update)
    return;


    double z = (fdm.ltpprz_pos.z + get_gaussian_noise()*NPS_SONAR_NOISE_STD_DEV) * 100; // convert to cm

    double sonar_reading = -z * NPS_SONAR_SENSITIVITY;
    sonar_reading = rint(sonar_reading);
    sonar->value = sonar_reading;
    Bound(sonar->value, 20, 700);
    //}

    sonar->next_update += NPS_SONAR_DT;
    sonar->data_available = TRUE;
}

