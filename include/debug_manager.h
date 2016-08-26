#ifndef _DEBUG_MANAGER_H_
#define _DEBUG_MANAGER_H_

#include <stdlib.h>
#include <stdio.h>
#include <strings.h>
#include <set>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <map>
#include <utility>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <lcm/lcm-cpp.hpp>
#include <pthread.h>
#include "endnodedebug_t.hpp"
#include "xbee_app_data.h"

using namespace std;


struct DebugCheckpoint
{
  int n_packets_sent;
  int n_packets_rcv;
  xbee_app_data::EndNodeDebug edebug;
};

class DebugManager
{
public:
    // Constructers
    DebugManager(const char * url, const string &channel, bool autorun);

    bool run();

    void handleMessage(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const endnodedebug_t* msg);

    void publishLCM(int nodeid, xbee_app_data::EndNodeDebug &eDebug);
    void checkpoint(int nodeid, int n_packets_sent,
		    int n_packets_rcv, xbee_app_data::EndNodeDebug &edebug);
      
private:
    std::map<int, DebugCheckpoint> m_lastCP;
    static uint64_t getTime();
    static std::string getTimeStr();

    const char * m_lcmURL;
    string m_lcmChannel;
    lcm::LCM m_lcm;

    pthread_mutex_t m_mutex; /** Mutex to control the access to member variables**/
    pthread_t m_thread; /** Thread **/

    inline bool isLCMReady();
    inline void subscribeToChannel(const string & channel) ;


    static void * internalThreadEntryFunc(void * ptr)
    {
        (( DebugManager *) ptr)->internalThreadEntry();
        return NULL;
    }
    void internalThreadEntry();
};

#endif
