#include "gps_driver.h"
#include <sys/time.h>
#include <lcm/lcm-cpp.hpp>
#include <glog/logging.h>
#define IT(c) __typeof((c).begin())
#define FOREACH(i,c) for(__typeof((c).begin()) i=(c).begin();i!=(c).end();++i)



std::ostream
&operator<<(std::ostream &os, const pos_gps_t &gps)
{
  os << "lat: " << gps.latitude
     << " lon: " << gps.longitude
     << " alt: " << gps.altitude;
  return os;
}
  

GPSDriver::GPSDriver(int id,
		     const char * url,
		     const string &channel,
		     bool handle,
		     bool with_gpsd):
    m_uselcm(true)
{
  m_id = id;
    /// Create a new LCM instance
    m_lcm = lcm_create(url);
    /// Set LCM URL
    m_lcmURL = url;
    /// Set LCM channel
    m_lcmChannel = channel;
    m_running = false;
    m_abort=false;
    if( handle )
      {
	/// FIXME: better outside?
	if (isLCMReady())
	  {
	    subscribeToChannel(channel);
	  }
	run();
      }
    if( with_gpsd )
      {
	m_gpsdClient = new GPSDClient();
	m_gpsdOk = true;
      }
    else
      {
	m_gpsdOk = false;
      }
    pthread_mutex_init(&m_mutex, NULL);
}

GPSDriver::GPSDriver(bool autorun):
    m_uselcm(false)
{
    pthread_mutex_init(&m_mutex, NULL);
    /*if(m_gpsdClient)
    {
      fprintf(stderr, "gpsd client could not start!\n");
      m_gpsdOk=false;
    }

  else*/
    m_gpsdOk = true;
    
    if( autorun )
        run();
}

bool
GPSDriver::run()
{
    int status = pthread_create(&m_thread, NULL, internalThreadEntryFunc, this);
    m_running=true;
    return (status == 0);
}

void
GPSDriver::handleMessage(const lcm::ReceiveBuffer* rbuf,
                         const std::string& chan,
                         const pos_gps_t* msg)
{
    uint64_t tt = getTime();
    //  uint8_t  rid = msg->robotid;
    LOG(INFO) << "handleMsg " << tt
	      << " @ chan " << chan
	      << " pos_gps: " << *msg;
    if( m_id != msg->robotid )
      {
	LOG(INFO) << "Not my GPS pose";
	return;
      }
    pthread_mutex_lock(&m_mutex);
    m_latestData.lat = msg->latitude;
    m_latestData.lon = msg->longitude;
    m_latestData.alt = msg->altitude;
    pthread_mutex_unlock(&m_mutex);
}

/// lock, copy, unlock, return
TimestampedGPSData
GPSDriver::data()
{
    pthread_mutex_lock(&m_mutex);
    TimestampedGPSData ret( m_latestData );
    pthread_mutex_unlock(&m_mutex);
    return ret;
}

/// returns time in milliseconds
uint64_t
GPSDriver::getTime()
{
    struct timeval timestamp;
    gettimeofday(&timestamp, NULL);

    uint64_t ms1 = (uint64_t) timestamp.tv_sec;
    ms1*=1000;

    uint64_t ms2 = (uint64_t) timestamp.tv_usec;
    ms2/=1000;

    return (ms1+ms2);
}

std::string
GPSDriver::getTimeStr()
{
#ifndef FOOTBOT_LQL_SIM
    char buffer [80];
    timeval curTime;
    gettimeofday(&curTime, NULL);
    int milli = curTime.tv_usec / 1000;
    strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", localtime(&curTime.tv_sec));
    char currentTime[84] = "";
    sprintf(currentTime, "%s:%d", buffer, milli);
    std::string ctime_str(currentTime);
    return ctime_str;
#else
    return "mytime";
#endif
}

inline bool
GPSDriver::isLCMReady()
{
    if (!m_lcm.good())
    {
        printf("LCM is not ready :(");
        return false;
    }
    else
    {
        printf("LCM is ready :)");
        return true;
    }
}

inline void
GPSDriver::subscribeToChannel(const string & channel)
{
    printf("Listening to channel %s\n", channel.c_str());
    m_lcm.subscribe(channel, &GPSDriver::handleMessage, this);
}

void
GPSDriver::notifyPos(int nodeid,
		     double lon,
		     double lat,
		     double alt,
		     int epsg)
{
  pos_gps_t mymsg;
  mymsg.robotid = nodeid;
  mymsg.longitude = lon;
  mymsg.latitude = lat;
  mymsg.altitude = alt;
  /// here we transform to UTM
  /// and then publish
  m_lcm.publish(m_lcmChannel.c_str(), &mymsg);
}
void
GPSDriver::internalThreadEntry()
{
  while (true)
    {
      if( m_uselcm )
        {
	  m_lcm.handle();
        }
      else{
	if( m_gpsdOk ){
	  LOG(INFO) << "Trying to get data from GPSD Client";
	  gps gpsCoordinates;
	  gpsCoordinates = m_gpsdClient->getGPS();
	  if ((gpsCoordinates.x != 0) | (gpsCoordinates.y != 0) | (gpsCoordinates.z != 0 ))
	    {
	      pthread_mutex_lock(&m_mutex);
	      m_latestData.lat = gpsCoordinates.x;
	      m_latestData.lon = gpsCoordinates.y;
	      m_latestData.alt = gpsCoordinates.z;
	      LOG(INFO) << "GPS Data updated from GPSD:"
			<< " [ " << gpsCoordinates.x
			<< " , " << gpsCoordinates.y
			<< " , " << gpsCoordinates.z
			<< " ]";
	      pthread_mutex_unlock(&m_mutex);
	    }
	}
      }
    }
}

GPSDriver::~GPSDriver()
{
  if( isRunning() )
    {
      stop();
      pthread_join(m_thread,NULL);
    }
}
