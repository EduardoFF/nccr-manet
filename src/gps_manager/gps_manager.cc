#include "gps_manager.h"
#include <sys/time.h>

GPSManager::GPSManager()
    : m_gps(NULL), m_use_gps_time(true), m_check_fix_by_variance(true) {

    std::string host = "localhost";
    int m_port = 2947;
    char m_port_s[12];
    snprintf(m_port_s, 12, "%d", m_port);

    gps_data_t *m_resp = NULL;
    m_gps = new gpsmm(host.c_str(), m_port_s);
    m_resp = m_gps->stream(WATCH_ENABLE);

    if (m_resp == NULL) {
        printf("Failed to open GPSd\n");
    }

    printf("GPSd opened\n");
}

gps1
GPSManager::getGPS1() {

    gps1 coordinates;
    if (!m_gps->waiting(1e6)){
        return coordinates;
    }

    gps_data_t *p = m_gps->read();

    /// Prevents returning values iff GPS offline or returning NULL
    if (p == NULL || !p->online ){
        return coordinates;
        }

    /// Prevents returning NaN values
    if (isnan(p->fix.epx) && m_check_fix_by_variance) {
        return coordinates;
    }

    coordinates.x = p->fix.latitude;
    coordinates.y = p->fix.longitude;
    coordinates.z = p->fix.altitude;

    return coordinates;
}

void
GPSManager::getGPS2() {

    if (!m_gps->waiting(1e6))
        return;

    gps_data_t *p = m_gps->read();

    /// Prevents returning values if GPS offline or returning NULL
    if (p == NULL || !p->online )
        return ;

    /// Prevents returning NaN values
    if (isnan(p->fix.epx) && m_check_fix_by_variance)
        return;

    m_latitude  = p->fix.latitude;
    m_longitude = p->fix.longitude;
    m_altitude  = p->fix.altitude;

    printf("INSIDE GPSD CLIENT   : %f %f %f \n",m_latitude, m_longitude, m_altitude);

    return;
}
