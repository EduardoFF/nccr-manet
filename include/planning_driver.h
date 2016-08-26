#ifndef _PLANNING_DRIVER_H_
#define _PLANNING_DRIVER_H_

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
#include "plan2_tree_t.hpp"


using namespace std;

struct EntryP
{
    float latitude;
    float longitude;
    float altitude;
    std::string action;
    std::string option;
    uint64_t timestamp;
};

typedef vector<EntryP> PlanningTable;

struct TimestampedPLANNINGData
{
    uint64_t timestamp;
    map<string, PlanningTable > plan;

    TimestampedPLANNINGData(): timestamp(0)
    {
    }
};
std::ostream &operator<<(std::ostream &, const TimestampedPLANNINGData &);

class PLANNINGDriver
{
public:
    // Constructers
    PLANNINGDriver();
    PLANNINGDriver(const char * url, const string &channel, bool autorun);

    bool run();

    void handleMessage(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const plan2_tree_t* msg);

    TimestampedPLANNINGData data();
    void clearData(uint64_t t);
    void addEntry(std::string, EntryP &);
    void publishLCM();
private:
    static uint64_t getTime();
    static std::string getTimeStr();
    TimestampedPLANNINGData m_latestPlanningData;

    const char * m_lcmURL;
    string m_lcmChannel;
    lcm::LCM m_lcm;

    pthread_mutex_t m_mutex; /** Mutex to control the access to member variables**/
    pthread_t m_thread; /** Thread **/

    inline bool isLCMReady();
    inline void subscribeToChannel(const string & channel) ;


    static void * internalThreadEntryFunc(void * ptr)
    {
        (( PLANNINGDriver *) ptr)->internalThreadEntry();
        return NULL;
    }
    void internalThreadEntry();
};

#endif
