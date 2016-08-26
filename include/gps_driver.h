#ifndef _GPS_DRIVER_H_
#define _GPS_DRIVER_H_
// -*- mode: c++; c-basic-offset: 4 -*-
/*
 * gps_driver.{cc,hh} -- interface to GPS data
 *
 * Author:		        Eduardo Feo Flushing
                    Dalle Molle Institute for Artificial Intelligence
                IDSIA - Manno - Lugano - Switzerland
                (eduardo <at> idsia.ch)

 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <stdlib.h>
#include <stdio.h>
#include <strings.h>
#include <set>
#include <string>
#include <libgpsmm.h>
#include <cmath>
#include <sstream>
#include <fstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <lcm/lcm-cpp.hpp>
#include <pthread.h>
#include "pos_gps_t.hpp"
#include "gps_client.h"

using namespace std;

struct TimestampedGPSData
{
    uint64_t timestamp;
    float lat;
    float lon;
    float alt;
    TimestampedGPSData(uint64_t t, float _lat, float _lon, float _alt):
        timestamp(t),
        lat(_lat),
        lon(_lon),
        alt(_alt)
    {}
    TimestampedGPSData():
        timestamp(0),lon(0),lat(0),alt(0)
    {}

};

class GPSDriver 
{
public:
  /// constructor for using gpsd client
    GPSDriver(bool autorun);
    GPSDriver(int id,
	      const char * url,
	      const string &channel,
	      bool handle,
	      bool with_gpsd);

    bool run();

    void handleMessage(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const pos_gps_t* msg);


    TimestampedGPSData data();
    
    void notifyPos(int nodeid, double lon, double lat, double alt, int epsg);
  
    ~GPSDriver();
private:
    static uint64_t getTime();
    static std::string getTimeStr();
    TimestampedGPSData m_latestData;

    int m_id;
    bool m_uselcm;
    const char * m_lcmURL;
    string m_lcmChannel;
    lcm::LCM m_lcm;

    bool m_gpsdOk;
    bool m_running;
    bool m_abort;
    GPSDClient *m_gpsdClient;


    pthread_mutex_t m_mutex; /** Mutex to control the access to member variables **/
    pthread_t m_thread;     /** Thread **/

    inline bool isLCMReady();
    bool isRunning(){ return m_running;}
    bool stop(){ m_abort=true;}
    inline void subscribeToChannel(const string & channel) ;
    static void * internalThreadEntryFunc(void * ptr)
    {
        (( GPSDriver *) ptr)->internalThreadEntry();
        return NULL;
    }

    void internalThreadEntry();



};

#endif
