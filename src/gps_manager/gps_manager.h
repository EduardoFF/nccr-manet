#ifndef GPS_MANAGER_H_INCLUDED
#define GPS_MANAGER_H_INCLUDED

#include <string>
#include <libgpsmm.h>
#include <cmath>

struct gps1 {

float x;
float y;
float z;

gps1(): x(0), y(0),z(0) {};

};

class GPSManager {
  public:

    GPSManager();

    gps1 getGPS1();
    void getGPS2();

  private:

    float m_latitude;
    float m_longitude;
    float m_altitude;

    gpsmm *m_gps;

    bool m_use_gps_time;
    bool m_check_fix_by_variance;

};

#endif // GPS_MANAGER_H_INCLUDED
