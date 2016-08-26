#include "gps_client.h"
#include <sys/time.h>

GPSDClient::GPSDClient()
    : m_gps(NULL), m_use_gps_time(true), m_check_fix_by_variance(true) {

    std::string m_host = "localhost";
    int m_port = 2947;
    char m_port_s[12];
    snprintf(m_port_s, 12, "%d", m_port);

    gps_data_t *m_resp = NULL;
    m_gps = new gpsmm(m_host.c_str(), m_port_s);
    m_resp = m_gps->stream(WATCH_ENABLE);

    if (m_resp == NULL)
    {
        printf("Failed to open GPSd\n");
    }
    printf("GPSd opened\n");
}

gps
GPSDClient::getGPS()
{
    gps coordinates;
    if (!m_gps->waiting(1e6)){
        //printf("ERREUR 1\n");
        return coordinates;
    }

    gps_data_t *p = m_gps->read();

    if (p == NULL || !p->online){
        //printf("ERREUR 2\n");
        return coordinates;
    }

    if (std::isnan(p->fix.epx) && m_check_fix_by_variance)
    {
        //printf("ERREUR 3\n");
        return coordinates;
    }

    /// Store gps Data in inner variables
    coordinates.x = p->fix.latitude;
    coordinates.y = p->fix.longitude;
    coordinates.z = p->fix.altitude;
    //printf( "lattitude: %f, longitude:  %f, altitude: %f\n",coordinates.x, coordinates.y, coordinates.z);

    return coordinates;
}

void
GPSDClient::getGPS1()
{
    if (!m_gps->waiting(1e6))
        return;

    gps_data_t *p = m_gps->read();

    /// Prevents returning values if GPS offline or returning NULL
    if (p == NULL || !p->online)
        return;

    /// Prevents returning NaN values
    if (std::isnan(p->fix.epx) && m_check_fix_by_variance)
    {
        return;
    }

    /// Store gps Data in inner variables
    m_latitude = p->fix.latitude;
    m_longitude = p->fix.longitude;
    m_altitude = p->fix.altitude;

    /* Latitude, longitude and altitude are printed out as spherical values */
    printf( "lattitude: %f, longitude:  %f, altitude: %f\n",m_latitude, m_longitude, m_altitude);
}

