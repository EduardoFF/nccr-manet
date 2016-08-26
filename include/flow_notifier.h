#ifndef _FLOW_NOTIFIER_H_
#define _FLOW_NOTIFIER_H_
// -*- mode: c++; c-basic-offset: 4 -*-
/*
 * flow_notifier.{cc,hh} -- interface to flow information
 *
 * Author:      Eduardo Feo Flushing
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
#include "flow_list_t.hpp"
//#include "tree.h"

using namespace std;

struct node_addr
{
  char    type;
  uint8_t value[6];
  string toString()
  {
    if( type == 'i' )
      {
	char ips[20];
	sprintf(ips, "%d.%d.%d.%d",
		value[0],
		value[1],
		value[2],
		value[3]);
	return ips;
      }
    else if( type == 'm' )
      {
	char macs[20];
	sprintf(macs, "%02x:%02x:%02x:%02x:%02x:%02x",
		value[0],
		value[1],
		value[2],
		value[3],
		value[4],
		value[5]);

	return macs;
      }
    else if( type == 'n' )
      {
	char nid[3];
	sprintf(nid,"%d", (uint8_t) value[0]);
	return nid;
      }
    else
      return "INVALID";
  }
};

struct NodeAddrComp
{
    bool operator()( node_addr const& lhs, node_addr const& rhs ) const
    {
      if( lhs.type == rhs.type )
	{
	  for(int i=0; i<6; i++)
	    {
	      if( lhs.value[i] < rhs.value[i] )
		return true;
	      else if( lhs.value[i] > rhs.value[i])
		return false;
	    }
	  return false;
	}
      else
	return (lhs.type < rhs.type);
    }
};

struct FlowEntry
{
  node_addr  src_addr;
  node_addr  dst_addr;
  int32_t    pkt_count;
  int32_t    byte_count;
  double     data_rate;
  int64_t    last_activity;
};

typedef std::vector<FlowEntry> FlowList;

typedef std::map< node_addr,
  std::map< node_addr, FlowEntry, NodeAddrComp>, NodeAddrComp > FlowMap;
typedef std::map< node_addr, int, NodeAddrComp > AddressBookByAddr;
typedef std::map< int, node_addr  > AddressBookById;


std::ostream &operator<<(std::ostream &, const FlowMap &);

class FlowNotifier
{
public:
    // Constructers
    FlowNotifier();
    FlowNotifier(const char * url, const string &channel, bool autorun);

    bool run();
    FlowList getFlows(int dt);
    bool notifyFlows(int node_id, uint64_t timestamp, FlowList &);

    void handleMessage(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const flow_list_t* msg);

    //    FlowList data();
    AddressBookById   m_addrBookIp, m_addrBookMac;
    AddressBookByAddr m_addrBookId;
    void readAddressBook(const std::string &fn_addrbook);
    int getIdFromAddressBook(node_addr &addr);
    node_addr getIpFromAddressBook(int node_id);
    bool isRunning(){ return m_running;}
    bool stop(){ m_abort=true;}
    uint64_t lastNotificationTime();
    ~FlowNotifier();
      
    
      
      
private:
    static uint64_t getTime();
    static std::string getTimeStr();
    FlowMap m_flowBySrc, m_flowByDst;

    bool isIPAddress(const std::string &);
    bool isMACAddress(const std::string &);
    
    bool parseIP(const std::string &, node_addr &);
    bool parseMAC(const std::string &, node_addr &);

    const char * m_lcmURL;
    string m_lcmChannel;
    lcm::LCM m_lcm;

    pthread_mutex_t m_mutex; /** Mutex to control the access to member variables**/
    pthread_t m_thread; /** Thread **/

    bool m_running;
    bool m_abort;
    uint64_t m_lastUpdate;

    inline bool isLCMReady();
    inline void subscribeToChannel(const string & channel) ;


    static void * internalThreadEntryFunc(void * ptr)
    {
        (( FlowNotifier *) ptr)->internalThreadEntry();
        return NULL;
    }
    void internalThreadEntry();
};

#endif
