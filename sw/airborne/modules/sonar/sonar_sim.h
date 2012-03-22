#ifndef SONAR_SIM_H
#define SONAR_SIM_H

#include "std.h"

extern void sonar_impl_init(void);
extern void sonar_feed_value(void);

extern void maxbotix_init(void);
extern void maxbotix_read(void);

#define SonarEvent(_handler) { \
  if (sonar_data_available) { \
    _handler(); \
    sonar_data_available = FALSE; \
  } \
}

#endif /* SONAR_SIM_H */
