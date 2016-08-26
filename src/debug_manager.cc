#include "debug_manager.h"
#include <sys/time.h>
#include <glog/logging.h>

#define IT(c) __typeof((c).begin())
#define FOREACH(i,c) for(__typeof((c).begin()) i=(c).begin();i!=(c).end();++i)


using namespace xbee_app_data;


DebugManager::DebugManager(const char * url,
			   const string &channel,
			   bool handle)
{
  m_lcm = lcm_create(url); /// Create a new LCM instance
  m_lcmURL = url; /// Set LCM URL
  m_lcmChannel = channel; /// Set LCM channel
  pthread_mutex_init(&m_mutex, NULL);
  if( handle )
    {
      if (isLCMReady())/// FIXME: better outside?
        {
	  subscribeToChannel(channel);
        }
      run();
    }
}

void
DebugManager::checkpoint(int nodeid,
			 int n_packets_sent,
			 int n_packets_rcv,
			 EndNodeDebug &edebug)
{
  if( m_lastCP.find( nodeid ) != m_lastCP.end())
    {
      int last_packets_sent = m_lastCP[nodeid].n_packets_sent;
      int last_packets_rcv = m_lastCP[nodeid].n_packets_rcv;
      EndNodeDebug &last_edebug = m_lastCP[nodeid].edebug;
      /// compute prrs
      /// a -> b
      int n_sent_a = (n_packets_sent - last_packets_sent);
      int n_rcv_a = (edebug.n_xbee_pkts_rcv - last_edebug.n_xbee_pkts_rcv);
      double prr_a = 0;
      if( n_sent_a != 0)
	prr_a = 1.0 * n_rcv_a / n_sent_a;


      int n_sent_b = (edebug.n_xbee_pkts_sent - last_edebug.n_xbee_pkts_sent);
      int n_rcv_b = (n_packets_rcv - last_packets_rcv);      
	  
      double prr_b = 0;
      if( n_sent_b != 0)
	prr_b = 1.0 * n_rcv_b / n_sent_b;
      LOG(INFO) << "DebugCheckpoint "
		<< nodeid << " [->] " << prr_a
		<< " [<-] " << prr_b;

    }
  DebugCheckpoint cp;
  cp.n_packets_sent = n_packets_sent;
  cp.n_packets_rcv = n_packets_rcv;
  cp.edebug = edebug;
  m_lastCP[nodeid] = cp;
}

bool
DebugManager::run()
{
  int status = pthread_create(&m_thread, NULL, internalThreadEntryFunc, this);
  return (status == 0);
}


void
DebugManager::publishLCM(int nodeid, xbee_app_data::EndNodeDebug &eDebug)
{
  pthread_mutex_lock(&m_mutex);
  endnodedebug_t mymsg;
  //TODO assign timestamp
  mymsg.timestamp = getTime();
  mymsg.nodeid = (char) nodeid;
  mymsg.n_xbee_pkts_sent = eDebug.n_xbee_pkts_sent;
  mymsg.n_xbee_bytes_sent = eDebug.n_xbee_bytes_sent;
  mymsg.n_xbee_pkts_rcv= eDebug.n_xbee_pkts_rcv;
  mymsg.n_xbee_bytes_rcv= eDebug.n_xbee_bytes_rcv;
  mymsg.timestamp= eDebug.timestamp;
  mymsg.last_flow_notify_time= eDebug.last_flow_notify_time;
  mymsg.manet_alive = eDebug.manet_alive;
  
  m_lcm.publish(m_lcmChannel.c_str(), &mymsg);
  pthread_mutex_unlock(&m_mutex);
}

/* Converts LCM data to TimestampedROUTINGData */
// we assign the timestamp of lcm msg reception
void
DebugManager::handleMessage(const lcm::ReceiveBuffer* rbuf,
			    const std::string& chan,
			    const endnodedebug_t* msg)
{

}

/// returns time in milliseconds
uint64_t
DebugManager::getTime()
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
DebugManager::getTimeStr()
{
  char buffer [80];
  timeval curTime;
  gettimeofday(&curTime, NULL);
  int milli = curTime.tv_usec / 1000;
  strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", localtime(&curTime.tv_sec));
  char currentTime[84] = "";
  sprintf(currentTime, "%s:%d", buffer, milli);
  std::string ctime_str(currentTime);
  return ctime_str;
}

inline bool
DebugManager::isLCMReady()
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
DebugManager::subscribeToChannel(const string & channel)
{
  printf("Listening to channel %s\n", channel.c_str());
  m_lcm.subscribe(channel, &DebugManager::handleMessage, this);
}

void
DebugManager::internalThreadEntry()
{
  while (true)
    {
      m_lcm.handle();
    }
}

