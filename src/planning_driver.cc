#include "planning_driver.h"
#include <sys/time.h>

#define IT(c) __typeof((c).begin())
#define FOREACH(i,c) for(__typeof((c).begin()) i=(c).begin();i!=(c).end();++i)

PLANNINGDriver::PLANNINGDriver()
{
}


std::ostream &
operator<<(std::ostream &os, const TimestampedPLANNINGData &rdata)
{
    os << "t " << rdata.timestamp << std::endl;
    os << "Tables: " << rdata.plan.size() << std::endl;
    FOREACH(it, rdata.plan)
    {
        os << it->first << " (" << it->second.size() << ") ";
        FOREACH(jt, it->second )
        {
            os << "[" << (*jt).latitude << (*jt).longitude << (*jt).altitude << "] ";
            os << (*jt).action << " " << (*jt).option;
            os << (*jt).timestamp;
            os << std::endl;
        }
        os << std::endl;
    }
    return os;
}

PLANNINGDriver::PLANNINGDriver(const char * url,
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

bool
PLANNINGDriver::run()
{
    int status = pthread_create(&m_thread, NULL, internalThreadEntryFunc, this);
    return (status == 0);
}

/// lock, copy, unlock, return
TimestampedPLANNINGData
PLANNINGDriver::data()
{
    pthread_mutex_lock(&m_mutex);
    TimestampedPLANNINGData ret( m_latestPlanningData );
    pthread_mutex_unlock(&m_mutex);
    return ret;
}

void
PLANNINGDriver::clearData(uint64_t t)
{
    pthread_mutex_lock(&m_mutex);
    m_latestPlanningData.timestamp = t;
    m_latestPlanningData.plan.clear();
    pthread_mutex_unlock(&m_mutex);
}

void
PLANNINGDriver::addEntry(std::string nid,
                         EntryP &entry)
{
    pthread_mutex_lock(&m_mutex);
    m_latestPlanningData.plan[nid].push_back(entry);
    pthread_mutex_unlock(&m_mutex);
}


void
PLANNINGDriver::publishLCM()
{
    pthread_mutex_lock(&m_mutex);
    plan2_tree_t mymsg;
    //TODO assign timestamp
    mymsg.timestamp = m_latestPlanningData.timestamp;
    mymsg.n = m_latestPlanningData.plan.size();
    mymsg.rtable.resize(mymsg.n);
    int i=0;
    FOREACH(it, m_latestPlanningData.plan)
    {
        plan2_table_t rtable;
        rtable.node = it->first;
        rtable.n = it->second.size();
        rtable.entries.resize(rtable.n);
        int j=0;
        FOREACH(jt, it->second )
        {
            plan2_entry_t entry;
            entry.latitude  = (*jt).latitude;
            entry.longitude = (*jt).longitude;
            entry.altitude  = (*jt).altitude;
            entry.action    = (*jt).action;
            entry.option    = (*jt).option;
            entry.timestamp = (*jt).timestamp;
            rtable.entries[j++]=entry;
        }
        mymsg.rtable[i++]=rtable;
    }
    m_lcm.publish(m_lcmChannel.c_str(), &mymsg);
    pthread_mutex_unlock(&m_mutex);
}

/* Converts LCM data to TimestampedROUTINGData */
// we assign the timestamp of lcm msg reception
void
PLANNINGDriver::handleMessage(const lcm::ReceiveBuffer* rbuf,
                              const std::string& chan,
                              const plan2_tree_t* msg)
{
    uint64_t tt = getTime();
    TimestampedPLANNINGData planningData;

    printf("PLANNINGDriver: [%lld] got config \n", (long long)tt);
    printf("  Received message on channel \"%s\":\n", chan.c_str());
    printf("  timestamp   = %lld\n", (long long)msg->timestamp);

    //    planningData.timestamp = msg->timestamp;
    planningData.timestamp = tt;
    for(int i = 0; i < msg->n; i++)
    {
        PlanningTable table;
        string nodeID = msg->rtable[i].node; // maybe change name of nodeID
        int nbEntry = msg->rtable[i].n;

        for(int j = 0; j < nbEntry; j++)
        {
            EntryP entry;
            entry.latitude = msg->rtable[i].entries[j].latitude;
            entry.longitude = msg->rtable[i].entries[j].longitude;
            entry.altitude = msg->rtable[i].entries[j].altitude;
            entry.action = msg->rtable[i].entries[j].action;
            entry.option = msg->rtable[i].entries[j].option;
            entry.timestamp = msg->rtable[i].entries[j].timestamp;
            table.push_back(entry);
        }
        planningData.plan[nodeID] = table;
    }
    pthread_mutex_lock(&m_mutex);
    m_latestPlanningData = planningData;
    pthread_mutex_unlock(&m_mutex);
    std::cout << "READ: " << std::endl;
    std::cout << m_latestPlanningData << std::endl;
}

/// returns time in milliseconds
uint64_t
PLANNINGDriver::getTime()
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
PLANNINGDriver::getTimeStr()
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
PLANNINGDriver::isLCMReady()
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
PLANNINGDriver::subscribeToChannel(const string & channel)
{
    printf("Listening to channel %s\n", channel.c_str());
    m_lcm.subscribe(channel, &PLANNINGDriver::handleMessage, this);
}

void
PLANNINGDriver::internalThreadEntry()
{
    while (true)
    {
        m_lcm.handle();
    }
}

