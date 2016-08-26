#include <libgpsmm.h>
#include <string>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>

struct gps{

float x;
float y;
float z;

gps(): x(0), y(0),z(0) {};

};

class GPSDClient {
  public:
    GPSDClient();

    gps getGPS();
    void getGPS1();

  private:
    float m_latitude;
    float m_longitude;
    float m_altitude;

    gpsmm *m_gps;
    bool m_use_gps_time;
    bool m_check_fix_by_variance;
};

