#include <iostream>
#include <stdlib.h>
#include <GetPot.hpp>
#include "gps_driver.h"
#include "routing_driver.h"
#include "planning_driver/planning_driver.h"
#include "xbee_interface.h"
#include "xbee_app_data.h"
#include "timer.h"
#include <csignal>
#include <glog/logging.h>
#include "flow_notifier/flow_notifier.h"
#include "debug_manager/debug_manager.h"
using namespace std;

#ifndef IT
#define IT(c) __typeof((c).begin())
#endif

#ifndef FOREACH
#define FOREACH(i,c) for(__typeof((c).begin()) i=(c).begin();i!=(c).end();++i)
#endif


//#define NO_XBEE_TEST

XbeeInterface *g_xbee;

ROUTINGDriver *g_routingDriver;
Timer *g_sendRoutingDataTimer;

// to make xbee - LCM bridge
FlowNotifier *g_flowNotifier;
GPSDriver *g_gpsDriver;

PLANNINGDriver *g_planningDriver;
Timer *g_sendPlanningDataTimer;

char g_outBuf[130];
bool g_abort;

TimestampedROUTINGData g_lastRoutingDataSent;
uint16_t g_lastXbeeTableId;
vector< vector<uint8_t> > g_routingXbeeMsgs;

TimestampedPLANNINGData g_lastPlanningDataSent;
uint16_t g_lastXbeePlanId;
vector< vector<uint8_t> > g_planningXbeeMsgs;
int g_epsg;

/// Mutex used to send one packet at a time
pthread_mutex_t g_sendMutex;


DebugManager *g_debugManager;
/// Function to print Help if need be
void print_help(const string Application)
{
  printf("ctrl_node:\n");
  exit(0);
}

ofstream *g_dumpSent;
ofstream *g_dumpRcv;


int g_nPacketsSent;
std::map<uint8_t, int> g_nPacketsRcv;
std::map<uint8_t, xbee_app_data::EndNodeDebug> g_lastDebugRcv;

std::map<uint8_t, uint16_t>  g_lastSeqnRcv;

uint16_t g_seqn;
uint8_t g_nodeId;

void xbeeSend(size_t buflen);
/// let's assume that desc is either 'SINK' or an IP address
uint8_t getNodeId(std::string desc)
{
    if( desc == "SINK" )
    {
        return 0;
    }
    else
    {
        uint8_t value[4] = {0};
        size_t index = 0;
        FOREACH(ct, desc)
        {
            if (isdigit((unsigned char)*ct))
            {
                value[index] *= 10;
                value[index] += *ct - '0';
            }
            else
            {
                index++;
            }
        }
        //TODO error checking
        return value[3];
    }
}

xbee_app_data::Header
makeHeader(const char packet_type)
{
  xbee_app_data::Header hdr;
  hdr.type = packet_type;
  hdr.src = g_nodeId;
  hdr.seqn = ++g_seqn;
  return hdr;
}


/// Function that sends the node info packet for routing Tables ///
void
sendRoutingDataTimerCB(void *arg)
{
    if( g_abort )
        return;
    /// lock the mutex first
    using namespace xbee_app_data;

    /// make payload
    /// Structure the data packets for Xbee
    TimestampedROUTINGData routingData = g_routingDriver->data();

    /// first, check if the routing data has changed by comparing the
    /// timestamps
    if( routingData.timestamp > g_lastRoutingDataSent.timestamp )
    {
        /// it has changed ...
        /// we need to create new xbee messages
        IT(routingData.route) it = routingData.route.begin();

        g_routingXbeeMsgs.clear();
	LOG(INFO) << "Sending " << routingData;
	bool abortSending=false;
        while(!abortSending)
        {
            std::vector<uint8_t> dataVec;
            dataVec.reserve(XBEE_MAX_PAYLOAD_LENGTH);
            int dataIx = 0;
            //// compose node info packet
            Header hdr = makeHeader(XBEEDATA_ROUTING);
            /// make header
	    //            hdr.type = XBEEDATA_ROUTING;
            dataVec.resize(dataVec.size() + sizeof(Header) );
            memcpy(&dataVec[dataVec.size() - sizeof(Header)],
                    &hdr, sizeof(Header));
            /// --- now the routing header
            Routing routeHdr;
            dataVec.resize(dataVec.size() + sizeof(Routing) );
            memcpy(&dataVec[dataVec.size() - sizeof(Routing)],
                    &routeHdr, sizeof(Routing));

            /// let's forget for a moment about the fragment numbers
            int nbytes=0;


            while(it != routingData.route.end() )
            {
                std::string nodedesc = it->first;
                LOG(INFO) << "pushing table of " << nodedesc;
                RoutingTable table = it->second;
                LOG(INFO) << table.size() << " entries ";
                /// can we fit all entries here ?
                if( dataVec.size() + sizeof(xbee_app_data::RoutingTableHdr)
                        + sizeof(xbee_app_data::RoutingEntry)*table.size()
                        <= XBEE_MAX_PAYLOAD_LENGTH )
                {
                    RoutingTableHdr rtHdr;
                    /// we need somehow to map node identifiers to numeric ids (0-255)
                    NodeId nid = getNodeId(nodedesc);
                    rtHdr.nodeId = nid;
                    rtHdr.nEntries = static_cast<uint8_t>(table.size());
                    dataVec.resize(dataVec.size() + sizeof(RoutingTableHdr) );
                    memcpy(&dataVec[dataVec.size() - sizeof(RoutingTableHdr)],
                            &rtHdr, sizeof(RoutingTableHdr));

                    for(int j=0; j<table.size(); j++)
                    {
                        RoutingEntry rtentry;
                        rtentry.dest = getNodeId(table[j].endNode);
                        rtentry.nextHop = getNodeId(table[j].nextHop);
                        rtentry.weight = table[j].weight;
                        dataVec.resize(dataVec.size() + sizeof(RoutingEntry) );
                        memcpy(&dataVec[dataVec.size() - sizeof(RoutingEntry)],
                                &rtentry, sizeof(RoutingEntry));
                    }
                    LOG(INFO) << "byte count " << dataVec.size();
                }
                else
                {
                    ///
		  LOG(INFO) << "table too big - sorry";
		  abortSending=true;
		  break;
                }
                it++;
            }
            /// push the message
            g_routingXbeeMsgs.push_back(dataVec);
            if( it == routingData.route.end() )
            {
                /// we are done
                break;
            }
        }
        /// now we care about the fragments
        LOG(INFO) << "generated msgs "
		  << g_routingXbeeMsgs.size();
        g_lastXbeeTableId++;
        uint8_t fgn = 0;
        FOREACH(jt, g_routingXbeeMsgs)
        {
            assert((*jt).size() >= sizeof(Header) + sizeof(Routing));
            if( (*jt).size() < sizeof(Header) + sizeof(Routing) )
            {
                fprintf(stderr, "something wrong here (%lu < %lu)\n",
                        (*jt).size(), sizeof(Header) + sizeof(Routing));
                exit(1);
            }
            Routing *routeHdr = (Routing *)&(*jt)[sizeof(Header)];
            routeHdr->tabId = g_lastXbeeTableId;
            routeHdr->fragNb = fgn++;
            routeHdr->nbOfFrag = static_cast<uint8_t>(g_routingXbeeMsgs.size());
            routeHdr->nbBytes = (*jt).size() - sizeof(Header) - sizeof(Routing);
        }
        /// we're done
        g_lastRoutingDataSent = routingData;
    }
    else
    {
        /// we don't need to encode again the xbee payload since the
        /// routing hasn't changed
      LOG(INFO) << "routing unchanged ...";
    }

    if( g_routingXbeeMsgs.size() )
      LOG(INFO) << "sending " << g_routingXbeeMsgs.size() << " msgs";
    for(int i=0; i< g_routingXbeeMsgs.size(); i++)
    {
      LOG(INFO) << "sending msgs " << (i+1)
		<< " out of " << g_routingXbeeMsgs.size()
		<< "with size " << g_routingXbeeMsgs[i].size();
      LOG(INFO) << +(g_routingXbeeMsgs[i][0]);

        memcpy(g_outBuf, &g_routingXbeeMsgs[i][0], g_routingXbeeMsgs[i].size());
        size_t buflen = g_routingXbeeMsgs[i].size();
        XbeeInterface::TxInfo txInfo;
        txInfo.reqAck = true;
        txInfo.readCCA = false;

	xbeeSend(buflen);
    }


}



void xbeeSend(size_t buflen)
{
  using namespace xbee_app_data;
  pthread_mutex_lock(&g_sendMutex);
  XbeeInterface::TxInfo txInfo;
  txInfo.reqAck = true;
  txInfo.readCCA = false;
  
#ifndef NO_XBEE_TEST
  int retval = g_xbee->send(XBEE_BROADCAST_ADDR, txInfo, g_outBuf, buflen);
  if( retval == XbeeInterface::NO_ACK )
    {
      LOG(INFO) << "send failed NOACK";
    }
  else if( retval == XbeeInterface::TX_MAC_BUSY )
    {
      LOG(INFO) << "send failed MACBUSY";
    }
  else
    {
      LOG(INFO) << "send OK";
      g_nPacketsSent++;
    }
#endif
  pthread_mutex_unlock(&g_sendMutex);
}

/// Function that sends the node info packet for planning Tables ///
void
sendPlanningDataTimerCB(void *arg)
{
  if( g_abort )
    return;
  /// lock the mutex first
  
  using namespace xbee_app_data;
  TimestampedPLANNINGData planningData = g_planningDriver->data();

  /// first, check if the routing data has changed by comparing the
  /// timestamps
  if( planningData.timestamp > g_lastPlanningDataSent.timestamp )
    {
      /// it has changed ...
      /// we need to create new xbee messages
      IT(planningData.plan) it = planningData.plan.begin();

      g_planningXbeeMsgs.clear();
      for(;;)
        {
	  std::vector<uint8_t> dataVec;
	  dataVec.reserve(XBEE_MAX_PAYLOAD_LENGTH);
	  int dataIx = 0;
	  //// compose node info packet
	  Header hdr = makeHeader(XBEEDATA_PLANNING);
	  /// make header
	  //	  hdr.type = XBEEDATA_PLANNING;
	  dataVec.resize(dataVec.size() + sizeof(Header) );
	  memcpy(&dataVec[dataVec.size() - sizeof(Header)],
		 &hdr, sizeof(Header));
	  /// --- now the planning header
	  Planning planHdr;
	  dataVec.resize(dataVec.size() + sizeof(Planning) );
	  memcpy(&dataVec[dataVec.size() - sizeof(Planning)],
		 &planHdr, sizeof(Planning));

	  /// let's forget for a moment about the fragment numbers
	  int nbytes=0;

	  while(it != planningData.plan.end() )
            {
	      std::string nodedesc = it->first;
	      LOG(INFO) << "pushing table of " << nodedesc;
	      PlanningTable table = it->second;
	      LOG(INFO) << table.size() << " entries ";
	      /// can we fit all entries here ?
	      if( dataVec.size() + sizeof(xbee_app_data::RoutingTableHdr)
		  + sizeof(xbee_app_data::PlanningEntry)*table.size()
		  <= XBEE_MAX_PAYLOAD_LENGTH )
                {
		  PlanningTableHdr plHdr;
		  /// we need somehow to map node identifiers to numeric ids (0-255)
		  NodeId nid = getNodeId(nodedesc);
		  plHdr.nodeId = nid;
		  plHdr.nEntries = static_cast<uint8_t>(table.size());
		  dataVec.resize(dataVec.size() + sizeof(PlanningTableHdr) );
		  memcpy(&dataVec[dataVec.size() - sizeof(PlanningTableHdr)],
			 &plHdr, sizeof(PlanningTableHdr));

		  for(int j=0; j<table.size(); j++)
                    {
		      PlanningEntry plEntry;
		      plEntry.latitude    = table[j].latitude;
		      plEntry.longitude   = table[j].longitude;
		      plEntry.altitude    = table[j].altitude;
		      plEntry.action      = table[j].action;
		      plEntry.option      = table[j].option;
		      plEntry.timestamp   = table[j].timestamp;
		      dataVec.resize(dataVec.size() + sizeof(PlanningEntry) );
		      memcpy(&dataVec[dataVec.size() - sizeof(PlanningEntry)],
			     &plEntry, sizeof(PlanningEntry));
                    }
		  LOG(INFO) << "byte count " << dataVec.size();
                }
	      else
                {
		  ///
		  break;
                }
	      it++;
            }
	  /// push the message
	  g_planningXbeeMsgs.push_back(dataVec);
	  if( it == planningData.plan.end() )
            {
	      /// we are done
	      break;
            }
        }
      /// now we care about the fragments
      LOG(INFO) << "generated msgs " <<  g_planningXbeeMsgs.size();
      g_lastXbeePlanId++;
      uint8_t fgn = 0;
      FOREACH(jt, g_planningXbeeMsgs)
        {
	  assert((*jt).size() >= sizeof(Header) + sizeof(Planning));
	  if( (*jt).size() < sizeof(Header) + sizeof(Planning) )
            {
	      fprintf(stderr, "something wrong here (%lu < %lu)\n",
		      (*jt).size(), sizeof(Header) + sizeof(Planning));
	      exit(1);
            }
	  Planning *planHdr = (Planning *)&(*jt)[sizeof(Header)];
	  planHdr->tabId = g_lastXbeePlanId;
	  planHdr->fragNb = fgn++;
	  planHdr->nbOfFrag = static_cast<uint8_t>(g_planningXbeeMsgs.size());
	  planHdr->nbBytes = (*jt).size() - sizeof(Header) - sizeof(Planning);
        }
      /// we're done
      g_lastPlanningDataSent = planningData;
    }
  else
    {
      /// we don't need to encode again the xbee payload since the
      /// routing hasn't changed
      LOG(INFO) << "Planning unchanged ...";
    }

  if( g_planningXbeeMsgs.size() )
    LOG(INFO) << "sending " << g_planningXbeeMsgs.size() << " msgs";
  for(int i=0; i< g_planningXbeeMsgs.size(); i++)
    {
      LOG(INFO) << "sending msgs " << (i+1)
		<< " out of " << (int) g_planningXbeeMsgs.size()
		<< " with size " << g_planningXbeeMsgs[i].size();
      LOG(INFO) << +(g_planningXbeeMsgs[i][0]);

      memcpy(g_outBuf, &g_planningXbeeMsgs[i][0], g_planningXbeeMsgs[i].size());
      size_t buflen = g_planningXbeeMsgs[i].size();
      xbeeSend(buflen);
    }
}

void
signalHandler( int signum )
{
    g_abort = true;
    printf("Ending ...\n");
    LOG(INFO) << "Ending app...";
    /// stop timers
    if( g_sendRoutingDataTimer )
    {
        g_sendRoutingDataTimer->stop();
	int cnt=10;
        while( g_sendRoutingDataTimer->isRunning() && --cnt)
        {
            sleep(1);
            g_sendRoutingDataTimer->stop();
        }
    }
    if( g_sendPlanningDataTimer )
    {
        g_sendPlanningDataTimer->stop();
	int cnt=10;
        while( g_sendPlanningDataTimer->isRunning() && --cnt)
        {
            sleep(1);
            g_sendPlanningDataTimer->stop();
        }
    }

    delete g_sendPlanningDataTimer;
    delete g_planningDriver;
    delete g_sendRoutingDataTimer;
    delete g_routingDriver;
    delete g_xbee;
    LOG(INFO) << "Clean exit";
    exit(signum);
}

/// Function that receives the messages and handles them per type
void
receiveData(uint16_t addr, void *data, char rssi, timespec timestamp, size_t len)
{
    using namespace xbee_app_data;
    LOG(INFO) << "Got data from " << addr
	      << " rssi: " << +rssi << ") "
	      <<  " len: " << len;
    
    if (len > sizeof(Header))
    {
        Header header;
        memcpy(&header, data, sizeof(Header));
        LOG(INFO) << "Header: " << header;

	if( g_nPacketsRcv.find( header.src ) == g_nPacketsRcv.end())
	  g_nPacketsRcv[header.src]=0;
	g_nPacketsRcv[header.src]++;

	/// check sequence number consistency
	if( g_lastSeqnRcv.find( header.src) == g_lastSeqnRcv.end())
	  {
	    g_lastSeqnRcv[header.src] = header.seqn;
	    LOG(INFO) << "Got first packet from " << +header.src;
	  }
	else
	  {
	    if( header.seqn > g_lastSeqnRcv[header.src] + 1)
	      {
		int missing_n = +(header.seqn - g_lastSeqnRcv[header.src]);
		LOG(WARNING) << "MISSINGNPACKETSEQN NODE " << +header.src
			     << " " << missing_n;
		if( missing_n < 10 )
		  {
		    for(uint16_t i=g_lastSeqnRcv[header.src]+1;i<header.seqn; i++)
		      {
			LOG(WARNING) << "MISSINGPACKETSEQN NODE "
				     << +header.src
				     << " " << +(i);
		      }
		  }		
	      }
	    else if( header.seqn <= g_lastSeqnRcv[header.src])
	      {
		LOG(WARNING) << "ROLLEDSEQN NODE " << +header.src
			     << " " << +(header.seqn)
			     << " < " << +(g_lastSeqnRcv[header.src]);
	      }
	    g_lastSeqnRcv[header.src] = header.seqn;
	    
	  }
	
	   

        if (header.type == XBEEDATA_ENDNODEINFO )               /// Check the type of Header
        {
            /// Check the size of the packet
	  LOG(INFO) << "Header: EndNodeInfo";
            if(len == sizeof(Header) + sizeof(EndNodeInfo))     /// Packet is of proper size
            {
                EndNodeInfo eInfo;
                memcpy(&eInfo,
                       (unsigned char *)data + sizeof(Header),
                       sizeof(EndNodeInfo));
                LOG(INFO) << "EndNodeInfo from " << +header.src
			  << ": " << eInfo;
                LOG(INFO) << "GPS Data Received from " << +header.src
			  << " lon: "
			  << eInfo.longitude
			  << " lat: "
			  << eInfo.latitude
			  << " alt: "
			  << eInfo.altitude;
		g_gpsDriver->notifyPos(header.src, eInfo.longitude,
				       eInfo.latitude,
				       eInfo.altitude,
				       g_epsg);
				       
		
            }
            else
            {
	      LOG(INFO) << "Invalid length for EndNodeInfo msg";
            }
        }

	if (header.type == XBEEDATA_ENDNODEDEBUG )        /// Check the type of Header
        {
            /// Check the size of the packet
	  LOG(INFO) << "Header: EndNodeDebug";
            if(len == sizeof(Header) + sizeof(EndNodeDebug))     /// Packet is of proper size
            {
                EndNodeDebug eDebug;
                memcpy(&eDebug,
                       (unsigned char *)data + sizeof(Header),
                       sizeof(EndNodeDebug));
                LOG(INFO) << "EndNodeDebug from " << +header.src
			  << ": " << eDebug;
		g_debugManager->checkpoint(header.src, 
					   g_nPacketsSent,
					   g_nPacketsRcv[header.src]-1,
					   eDebug);
		if( g_debugManager)
		  g_debugManager->publishLCM(header.src, eDebug);
            }
            else
            {
	      LOG(INFO) << "Invalid length for EndNodeInfo msg";
            }
        }
		
	if (header.type == XBEEDATA_FLOWINFO )  /// Check the type of Header
        {
	  LOG(INFO) << "Got FlowInfo from " << +header.src;
	  if( len >= sizeof(Header) + sizeof(FlowInfoHdr) )
	    {
	      FlowInfoHdr fInfoHdr;
	      memcpy(&fInfoHdr,
		     (unsigned char *)data + sizeof(Header),
		     sizeof(FlowInfoHdr));
	      LOG(INFO) << "got info for "
			<< +fInfoHdr.nEntries << " flows";
	      FlowInfoEntry fEntry;
	      FlowList fList;
	      char *ptr = (char *) data + sizeof(Header)+sizeof(FlowInfoHdr);
	      for(int i=0; i< fInfoHdr.nEntries;i++)
		{
		  memcpy(&fEntry,
			 ptr,
			 sizeof(FlowInfoEntry));
		  ptr += sizeof(FlowInfoEntry);
		  LOG(INFO) << "Flow " << (int) fEntry.nodeId
			    << " -> " << (int) header.src
			    << " " << fEntry.dataRate;
		  FlowEntry fe;
		  fe.dst_addr.type = 'n';
		  fe.dst_addr.value[0] = header.src;
		  fe.src_addr.type = 'n';
		  fe.src_addr.value[0] = fEntry.nodeId;
		  fe.data_rate = fEntry.dataRate;
		  fList.push_back(fe);
		}
	      LOG(INFO) << "Notifying flows";
	      g_flowNotifier->notifyFlows((int) header.src, 0, fList);
	 
	    }
	  else
	    {
	      LOG(INFO) << "Invalid length for FlowInfo msg";
	    }
	}
    }
}

/////////////// Beginning of Main program //////////////////
int main(int argc, char * argv[])
{
    g_abort = false;
    g_lastRoutingDataSent.timestamp = 0;
    g_lastPlanningDataSent.timestamp = 0;
    g_dumpSent = NULL;
    g_dumpRcv = NULL;
    g_nPacketsSent=0;
    g_seqn = 0;



    signal(SIGINT, signalHandler);      /// register signal

    // Simple Command line parser
    GetPot   cl(argc, argv);
    if(cl.search(2, "--help", "-h") ) print_help(cl[0]);
    cl.init_multiple_occurrence();
    cl.enable_loop();
	
    const string  xbeeDev  = cl.follow("/dev/ttyUSB0", "--dev");
    const int     baudrate = cl.follow(57600, "--baud");
    const int     nodeId   = cl.follow(1, "--nodeid");
    const string  xbeeMode = cl.follow("xbee5", "--mode");
    const string  addrBook  = cl.follow("none", "--abook");
    const string  logDir  = cl.follow("/tmp/", "--logdir");
    const string  dumpFile  = cl.follow("none", "--dump-file-prefix");
    g_epsg  = cl.follow(32632, "--epsg");

    g_nodeId = nodeId;

    /// Initialize Log
    google::InitGoogleLogging(argv[0]);
    FLAGS_logbufsecs = 0;
    FLAGS_log_dir = logDir;
    LOG(INFO) << "Logging initialized";

    if( dumpFile != "none" )
      {
	stringstream ss;
	ss << logDir << "/" << dumpFile << "_out.dump";
	string dump_fn = ss.str();
	g_dumpSent = new std::ofstream(dump_fn.c_str(), ios::out | ios::binary);
	LOG(INFO) << "Initialized dump sent in " << dump_fn;

	ss.str("");
	ss << logDir << "/" << dumpFile << "_in.dump";
	dump_fn = ss.str();
	g_dumpRcv = new std::ofstream(dump_fn.c_str(), ios::out | ios::binary);
	LOG(INFO) << "Initialized dump rcv in " << dump_fn;
      }

    
    /// Xbee PARAMETERS
    XbeeInterfaceParam xbeePar;
    xbeePar.SourceAddress = nodeId;
    xbeePar.brate = baudrate;
    xbeePar.mode  = xbeeMode;
    xbeePar.Device = xbeeDev;
    xbeePar.writeParams = false;

    /// create mutexes
    if (pthread_mutex_init(&g_sendMutex, NULL) != 0)
    {
        fprintf(stderr, "mutex init failed\n");
        fflush(stderr);
        exit(-1);
    }

    g_routingDriver = NULL;
    g_sendRoutingDataTimer = NULL;
    g_planningDriver = NULL;
    g_sendPlanningDataTimer = NULL;
    g_flowNotifier = NULL;
    g_gpsDriver = NULL;
    g_debugManager = NULL;


    
    LOG(INFO) << "Creating xbee interface";

#ifndef NO_XBEE_TEST
    /// create Xbee communication
    g_xbee = new XbeeInterface(xbeePar);
    if( g_xbee->isOK() )
      {
        /// Listen for messages
        g_xbee->registerReceive(&receiveData);
      }
    else
        return 1;
#endif

    /// create routing Driver
    g_routingDriver = new ROUTINGDriver("udpm://239.255.76.67:7667?ttl=0", "RNP2", true);
    /// Periodically send the Routing Table
    g_sendRoutingDataTimer = new Timer(TIMER_SECONDS, sendRoutingDataTimerCB, NULL);
    g_sendRoutingDataTimer->startPeriodic(1);
    /// create planning Driver
    g_planningDriver = new PLANNINGDriver("udpm://239.255.76.67:7667?ttl=0", "PLAN", true);
    /// Periodically send the Plan
    g_sendPlanningDataTimer = new Timer(TIMER_SECONDS, sendPlanningDataTimerCB, NULL);
    g_sendPlanningDataTimer->startPeriodic(1);
    g_flowNotifier = new FlowNotifier("udpm://239.255.76.67:7667?ttl=0", "iflowip", false);
    g_gpsDriver = new GPSDriver(nodeId, "udpm://239.255.76.67:7667?ttl=0", "RCVPOSEGPS", false, false);

    g_debugManager = new DebugManager("udpm://239.255.76.67:7667?ttl=0",
				      "XBEEDEBUG", false);
    LOG(INFO) << "drivers up";
    
    if( addrBook != "none" )
      {
	g_flowNotifier->readAddressBook(addrBook);
	LOG(INFO) << "read addressbook " << addrBook;
      }

    /// Sleep
    for(;;)
    {
        sleep(1);
    }

    if( g_dumpSent )
      {
	g_dumpSent->close();
	delete g_dumpSent;
      }
    if( g_dumpRcv )
      {
	g_dumpRcv->close();
	delete g_dumpRcv;
      }
    

    return 0;
}
