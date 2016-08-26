#include "flow_notifier.h"
#include <sys/time.h>
#include <algorithm>
#ifndef DISABLE_GLOG
#include <glog/logging.h>
#else
#define LOG(X) 
#endif
#define IT(c) __typeof((c).begin())
#define FOREACH(i,c) for(__typeof((c).begin()) i=(c).begin();i!=(c).end();++i)

FlowNotifier::FlowNotifier()
{
  m_running=false;
  m_abort=false;
  m_lastUpdate = 0;
}

void
FlowNotifier::readAddressBook(const std::string &fn_addrbook)
{
  ifstream ifile(fn_addrbook.c_str());
  int node_id;
  string ip_addr, mac_addr;
  while( ifile >> node_id >> ip_addr >> mac_addr )
    {
      node_addr ip_a, mac_a;
      if( parseIP(ip_addr, ip_a) )
	{
	  m_addrBookId[ip_a] = node_id;
	  m_addrBookIp[node_id] = ip_a;
	}
      if( parseMAC(mac_addr, mac_a) )
	{
	  m_addrBookId[mac_a] = node_id;
	  m_addrBookMac[node_id] = mac_a;
	}
    }
}

int
FlowNotifier::getIdFromAddressBook(node_addr &addr)
{
  IT(m_addrBookId) it = m_addrBookId.find(addr);
  if( it == m_addrBookId.end())
    return -1;
  else
    return it->second;
}

node_addr
FlowNotifier::getIpFromAddressBook(int node_id)
{
  node_addr naddr;
  naddr.type = 0;
  IT(m_addrBookIp) it = m_addrBookIp.find(node_id);
  if( it == m_addrBookIp.end())
    return naddr;
  else
    {
      return it->second;
    }
}

bool
FlowNotifier::notifyFlows(int nid, uint64_t timestamp, FlowList &flist)
{
  flow_list_t mymsg;
  node_addr ipaddr = getIpFromAddressBook(nid);
  if( ipaddr.type != 'i' )
    return false;
  mymsg.addr = ipaddr.toString();
  mymsg.timestamp = timestamp;
  mymsg.n = flist.size();
  mymsg.flows.resize(mymsg.n);
  int ix=0;
  for(int i=0; i< flist.size(); i++)
    {
      FlowEntry &fe = flist[i];
      flow_entry_t fentry;
      /// change nid to ip
      node_addr src_a = fe.src_addr;
      node_addr dst_a = fe.dst_addr;
      if( fe.src_addr.type == 'n' )
	{
	  src_a = getIpFromAddressBook((int) fe.src_addr.value[0]);
	}
      if( fe.dst_addr.type == 'n' )
	{
	  dst_a = getIpFromAddressBook((int) fe.dst_addr.value[0]);
	}

      std::string src_addr_s = src_a.toString();
      std::string dst_addr_s = dst_a.toString();
      
	
      fentry.src_addr = src_addr_s;
      fentry.dst_addr = dst_addr_s;
      fentry.pkt_count = fe.pkt_count;
      fentry.byte_count = fe.byte_count;
      fentry.data_rate = fe.data_rate;
      fentry.last_activity = fe.last_activity;
      mymsg.flows[ix++]=fentry;
    }
  printf("notified flows to %s\n", m_lcmChannel.c_str());
  m_lcm.publish(m_lcmChannel.c_str(), &mymsg);
}

FlowList
FlowNotifier::getFlows(int dt)
{
  FlowList flist;
  uint64_t ddt =  getTime() - dt;
  FOREACH(it, m_flowBySrc)
    {
      FOREACH(jt, it->second )
	{
	  FlowEntry &fe = jt->second;
	  if( fe.last_activity >= ddt )
	    {
	      flist.push_back(fe);
	    }
	}
    }
  return flist;
}

uint64_t
FlowNotifier::lastNotificationTime()
{
  return m_lastUpdate;
}


				   
std::ostream &
operator<<(std::ostream &os, const FlowMap &rdata)
{
    return os;
}

std::ostream &
operator<<(std::ostream &os, const node_addr &addr)
{
  if( addr.type == 'i' )
    {
      os << "IP ";
      char ips[15];
      sprintf(ips, "%d.%d.%d.%d",
	      addr.value[0],
	      addr.value[1],
	      addr.value[2],
	      addr.value[3]);
      os << ips;
    }
  else if( addr.type == 'm' )
    {
      os << "MAC ";
      char macs[20];
      sprintf(macs, "%02X:%02X:%02X:%02X:%02X:%02X",
	      addr.value[0],
	      addr.value[1],
	      addr.value[2],
	      addr.value[3],
	      addr.value[4],
	      addr.value[5]);

      os << macs;
    }
  else
    {
      os << "INVALID_ADDR";
    }
  return os;
}

FlowNotifier::FlowNotifier(const char * url, 
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
FlowNotifier::run()
{
    int status = pthread_create(&m_thread, NULL, internalThreadEntryFunc, this);
    m_running=true;
    return (status == 0);
}


/* Inserts LCM data to FlowMap */
void
FlowNotifier::handleMessage(const lcm::ReceiveBuffer* rbuf,
                             const std::string& chan,
                             const flow_list_t* msg)
{
    uint64_t tt = getTime();
    m_lastUpdate = tt;

    printf("FlowNotifier: [%lld] got config \n", (long long)tt);
    printf("  Received message on channel \"%s\":\n", chan.c_str());
    printf("  timestamp   = %lld\n", (long long)msg->timestamp);

    for(int i = 0; i < msg->n; i++)
    {
      FlowEntry f_entry;
      f_entry.data_rate = msg->flows[i].data_rate;
      f_entry.byte_count = msg->flows[i].byte_count;
      f_entry.pkt_count = msg->flows[i].pkt_count;
      f_entry.last_activity = msg->flows[i].last_activity;
      LOG(INFO) << "Checking addresses "
		<< msg->flows[i].src_addr
		<< " " << msg->flows[i].dst_addr;
      
      if( isIPAddress(msg->flows[i].src_addr) &&
	  isIPAddress(msg->flows[i].dst_addr) )
	{
	  //	  printf("is IP addr\n");
	  if( !parseIP(msg->flows[i].src_addr, f_entry.src_addr) ||
	      !parseIP(msg->flows[i].dst_addr, f_entry.dst_addr) )
	    {
	      //  printf("parsing error\n");
	    }
	  else
	    {
	      //LOG(INFO) << "Parsing OK";
	      cout << f_entry.src_addr << endl;
	      m_flowBySrc[f_entry.src_addr][f_entry.dst_addr] = f_entry;
	      m_flowByDst[f_entry.dst_addr][f_entry.src_addr] = f_entry;	
	    }
	  //	  f_entry.src_addr = parseIP(msg->flows[i].dst_addr);
	}
      else
	if(isMACAddress(msg->flows[i].src_addr) &&
	   isMACAddress(msg->flows[i].dst_addr) )
	  {
	    //printf("is MAC addr\n");
	    if ( !parseMAC(msg->flows[i].src_addr, f_entry.src_addr) ||
		 !parseMAC(msg->flows[i].dst_addr, f_entry.dst_addr) )
	      {
		//	printf("parsing error for MAC address\n");
	      }
	    else
	      {
		//	LOG(INFO) << "Parsing OK";
		//cout << f_entry.src_addr << endl;
		m_flowBySrc[f_entry.src_addr][f_entry.dst_addr] = f_entry;
		m_flowByDst[f_entry.dst_addr][f_entry.src_addr] = f_entry;
		//LOG(INFO) << "Parsing OK2";
	      }
	  }
    }
}


/// WARNING: this is a simple check for an IP address of the
/// form xxx.xxx.xxx.xxx , it does not tell if address is valid
bool
FlowNotifier::isIPAddress(const std::string &s)
{
  return (std::count(s.begin(), s.end(), '.') == 3 );
}

/// WARNING: this is a simple check for a MAC address of the
/// form xx:xx:xx:xx:xx:xx , it does not tell if address is validx
bool
FlowNotifier::isMACAddress(const std::string &s)
{
  return ((std::count(s.begin(), s.end(), ':') == 5 )
	  || (std::count(s.begin(), s.end(), '-') == 5 ));
    
}
    
bool
FlowNotifier::parseIP(const std::string &ss, node_addr &addr)
{
  //  const char *
  //IPAddressArg::basic_parse(const char *s, const char *end,
  //			  unsigned char value[4], int &nbytes)
  //{
  int nbytes = 0;
  const char *s = ss.c_str();
  memset(addr.value, 0, 4);
  addr.type = 0;
  int d = 0;
  const char *end = s + ss.size();
  while (d < 4 && s != end && (d == 0 || *s == '.')) {
    const char *t = s + !!d;
    if (t == end || !isdigit((unsigned char) *t))
      break;
    int part = *t - '0';
    for (++t; t != end && isdigit((unsigned char) *t) && part <= 255; ++t)
      part = part * 10 + *t - '0';
    if (part > 255)
      break;
    s = t;
    addr.value[d] = part;
    if (++d == 4)
      break;
  }
  nbytes = d;
  if (nbytes == 4)
    {
      addr.type ='i';
      return true;
    }
  else
    return false;
     
}

bool
FlowNotifier::parseMAC(const std::string &ss, node_addr &addr)
{
  int d = 0, p = 0, sep = 0;
  const char *s = ss.c_str();
  const char *begin = ss.c_str();
  const char *end = s + ss.size();
  memset(addr.value, 0, 6);
  addr.type = 0;
  for (s = begin; s != end; ++s) {
    int digit;
    //    LOG(INFO) <<"parsing " << *s;
    if (*s >= '0' && *s <= '9')
      digit = *s - '0';
    else if (*s >= 'a' && *s <= 'f')
      digit = *s - 'a' + 10;
    else if (*s >= 'A' && *s <= 'F')
      digit = *s - 'A' + 10;
    else {
      if (sep == 0 && (*s == '-' || *s == ':'))
	sep = *s;
      if (*s == sep && (p == 1 || p == 2) && d < 5) {
	p = 0;
	++d;
	continue;
      } else
	break;
    }

    if (p == 2 || d == 6)
      break;
    //    LOG(INFO) << "add.value " << d;
   addr.value[d] = (p ? addr.value[d] << 4 : 0) + digit;
    ++p;
  }

  if (s == end && p != 0 && d == 5)
    {
      addr.type ='m';
      return true;
    }
  else
    return false;
}



/// returns time in milliseconds
uint64_t
FlowNotifier::getTime()
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
FlowNotifier::getTimeStr()
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
FlowNotifier::isLCMReady() 
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
FlowNotifier::subscribeToChannel(const string & channel)
{
    printf("Listening to channel %s\n", channel.c_str());
    m_lcm.subscribe(channel, &FlowNotifier::handleMessage, this);
}

void
FlowNotifier::internalThreadEntry()
{
    while (true)
    {
        m_lcm.handle();
    }
}

FlowNotifier::~FlowNotifier()
{
  if( isRunning() )
    {
      stop();
      pthread_join(m_thread,NULL);
    }
}


