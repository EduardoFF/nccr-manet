//#include "node_example_core.h"
#include "ros/ros.h"

#include <iostream>
#include <stdlib.h>
#include "xbee_interface.h"
#include "xbee_app_data.h"
#include <glog/logging.h>

#include <nccr_manet_msgs/XbeePacket.h>
#include <nccr_manet_msgs/ForwardingTableArray.h>
#include <nccr_manet_msgs/ForwardingTableEntry.h>
#include <nccr_manet_msgs/ForwardingTable.h>

using namespace std;

#ifndef IT
#define IT(c) __typeof((c).begin())
#endif

#ifndef FOREACH
#define FOREACH(i,c) for(__typeof((c).begin()) i=(c).begin();i!=(c).end();++i)
#endif




XbeeInterface *g_xbee;
char g_outBuf[130];
bool g_abort;
/// Mutex used to send one packet at a time
pthread_mutex_t g_sendMutex;
int g_nPacketsSent;
std::map<uint8_t, int> g_nPacketsRcv;
std::map<uint8_t, uint16_t>  g_lastSeqnRcv;

uint16_t g_seqn;
uint8_t g_nodeId;
uint16_t g_lastXbeeTableId;
nccr_manet_msgs::ForwardingTableArray g_lastRoutingDataSent;

using namespace std;


vector< vector<uint8_t> > g_routingXbeeMsgs;

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

xbee_app_data::Header
makeHeader(const char packet_type)
{
  xbee_app_data::Header hdr;
  hdr.type = packet_type;
  hdr.src = g_nodeId;
  hdr.seqn = ++g_seqn;
  return hdr;
}
void
routingRcvCallback(const nccr_manet_msgs::ForwardingTableArray::ConstPtr& msg)
{
  using namespace xbee_app_data;
  using namespace nccr_manet_msgs;
  ROS_INFO("Got forwarding table array with size [%d]", msg->tables.size());

  IT(msg->tables) it = msg->tables.begin();
  g_routingXbeeMsgs.clear();
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

      

      while(it != msg->tables.end() )
	{
	  uint8_t nid = it->node;
	  const std::vector<ForwardingTableEntry> &entries
	    = it->entries;
	  //LOG(INFO) << table.size() << " entries ";
	  /// can we fit all entries here ?
	  if( dataVec.size() + sizeof(xbee_app_data::RoutingTableHdr)
	      + sizeof(xbee_app_data::RoutingEntry)*entries.size()
	      <= XBEE_MAX_PAYLOAD_LENGTH )
	    {
	      RoutingTableHdr rtHdr;
	      /// we need somehow to map node identifiers to numeric ids (0-255)
	      rtHdr.nodeId = nid;
	      rtHdr.nEntries = static_cast<uint8_t>(entries.size());
	      dataVec.resize(dataVec.size() + sizeof(RoutingTableHdr) );
	      memcpy(&dataVec[dataVec.size() - sizeof(RoutingTableHdr)],
		     &rtHdr, sizeof(RoutingTableHdr));

	      for(int j=0; j<entries.size(); j++)
		{
		  RoutingEntry rtentry;
		  rtentry.dest = entries[j].destination;
		  rtentry.nextHop = entries[j].next_hop;
		  rtentry.weight = entries[j].weight;
		  dataVec.resize(dataVec.size() + sizeof(RoutingEntry) );
		  memcpy(&dataVec[dataVec.size() - sizeof(RoutingEntry)],
			 &rtentry, sizeof(RoutingEntry));
		}
	    }
	  else
	    {
	      ///
	      ROS_INFO("table too big - sorry");
	      abortSending=true;
	      break;
	    }
	  it++;
	}
      /// push the message
      g_routingXbeeMsgs.push_back(dataVec);
      if( it == msg->tables.end() )
	{
	  /// we are done
	  break;
	}
    }
  /// now we care about the fragments
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
  g_lastRoutingDataSent = *msg;


  if( g_routingXbeeMsgs.size() )
    ROS_INFO("sending %u  msgs", g_routingXbeeMsgs.size());
  for(int i=0; i< g_routingXbeeMsgs.size(); i++)
    {
      cout << "sending msgs " << (i+1)
	   << " out of " << g_routingXbeeMsgs.size()
	   << "with size " << g_routingXbeeMsgs[i].size()
	   << endl;
      cout << +(g_routingXbeeMsgs[i][0]) << endl;
	
      memcpy(g_outBuf, &g_routingXbeeMsgs[i][0], g_routingXbeeMsgs[i].size());
      size_t buflen = g_routingXbeeMsgs[i].size();
      XbeeInterface::TxInfo txInfo;
      txInfo.reqAck = true;
      txInfo.readCCA = false;
      xbeeSend(buflen);
    }
}








ros::Publisher pub_message;
/// Function that receives the messages and handles them per type
void
receiveData(uint16_t addr, void *data, char rssi, timespec timestamp, size_t len)
{
    using namespace xbee_app_data;
    printf("got msg of len %u\n", len);
    nccr_manet_msgs::XbeePacket msg;
    for(int i=0; i< len; i++)
      msg.data.push_back(((uint8_t*)data)[i]);
    msg.addr = addr;
    msg.rssi = rssi;
    pub_message.publish(msg);
}
 


/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "xbee_manager");
  ros::NodeHandle nh;

  g_lastXbeeTableId=0;
  // Create a new NodeExample object.
  //  NodeExample *node_example = new NodeExample();

  // Set up a dynamic reconfigure server.
  // This should be done before reading parameter server values.
  //dynamic_reconfigure::Server<node_example::node_example_paramsConfig> dr_srv;
  //dynamic_reconfigure::Server<node_example::node_example_paramsConfig>::CallbackType cb;
  //cb = boost::bind(&NodeExample::configCallback, node_example, _1, _2);
  //dr_srv.setCallback(cb);

  // Declare variables that can be modified by launch file or command line.
  int a;
  int b;
  string message;
  int rate;
  string routing_topic;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can
  // be run simultaneously while using different parameters.
  // Parameters defined in the .cfg file do not need to be initialized here
  // as the dynamic_reconfigure::Server does this for you.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, int(1));
  private_node_handle_.param("tx_routing_topic", routing_topic, string("out_routes"));


  string  xbeeDev;
  string  xbeeMode;
  int baudrate;
  int nodeId;
  
  private_node_handle_.param("dev", xbeeDev, string("/dev/ttyUSB0"));
  private_node_handle_.param("baud", baudrate, int(57600));
  private_node_handle_.param("nodeid", nodeId, int(1));
  private_node_handle_.param("mode", xbeeMode, string("xbee5"));

  g_nodeId = static_cast<uint8_t>(nodeId);


  
  // Create a publisher and name the topic.
  pub_message =
    nh.advertise<nccr_manet_msgs::XbeePacket>("xbee_in", 10);

  ros::Subscriber routing_sub =
    nh.subscribe(routing_topic.c_str(), 1000, routingRcvCallback);
  // Tell ROS how fast to run this node.
  ros::Rate r(rate);


  g_abort = false;
  g_nPacketsSent=0;
  g_seqn = 0;
    
  /// Xbee PARAMETERS
  XbeeInterfaceParam xbeePar;
  xbeePar.SourceAddress = g_nodeId;
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

    
  g_xbee = new XbeeInterface(xbeePar);
  if( g_xbee->isOK() )
    {
      /// Listen for messages
      g_xbee->registerReceive(&receiveData);
    }
  


  
  // Main loop.
  while (nh.ok())
  {
    // Publish the message.
    //    node_example->publishMessage(&pub_message);

    ros::spinOnce();
    r.sleep();
    if( !g_xbee->checkAlive() )
      printf("Xbee is not alive!!\n");
      
  }

  return 0;
} // end main()
