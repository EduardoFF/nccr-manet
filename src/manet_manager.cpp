//#include "node_example_core.h"
#include "ros/ros.h"

#include <iostream>
#include <stdlib.h>
#include <GetPot.hpp>
#include "gps_driver.h"
#include "routing_driver.h"
#include "planning_driver.h"
#include "xbee_app_data.h"
#include "timer.h"
#include <csignal>
#include <glog/logging.h>
#include "flow_notifier.h"
#include "debug_manager.h"
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <nccr_manet_msgs/XbeePacket.h>
#include <nccr_manet_msgs/EndNodeXbeeStatus.h>


using namespace std;

#ifndef IT
#define IT(c) __typeof((c).begin())
#endif

#ifndef FOREACH
#define FOREACH(i,c) for(__typeof((c).begin()) i=(c).begin();i!=(c).end();++i)
#endif




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
int g_nPacketsSent;
std::map<uint8_t, int> g_nPacketsRcv;
std::map<uint8_t, xbee_app_data::EndNodeDebug> g_lastDebugRcv;

std::map<uint8_t, uint16_t>  g_lastSeqnRcv;

uint16_t g_seqn;
uint8_t g_nodeId;
std::map<std::string, std::string> g_staticDiagnostics, g_dynamicDiagnostics;
int g_dynamicDiagErrorLevel, g_staticDiagErrorLevel;

ros::Publisher xbeestatus_pub;

using namespace std;

void dummy_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
	       "Launch is in a long time. Have a soda.");
  stat.add("Diagnostic Name", "dummy");
  FOREACH(it, g_nPacketsRcv)
    {
      stringstream ss;
      ss << "PacketsReceived[" << +(it->first)
	 << "]";
      stat.addf(ss.str().c_str(),
	       "%d",it->second);       
    }
}

void
addDiagnostic(const int errLevel,
	      const std::string &topicAndClass,
	      const std::string &message,
	      const bool staticDiag)
{
    if (staticDiag)
    {
      g_staticDiagnostics[topicAndClass] = message;
      g_staticDiagErrorLevel = std::max(g_staticDiagErrorLevel, errLevel);
    }
    else
      {
	g_dynamicDiagnostics[topicAndClass] = message;
	g_dynamicDiagErrorLevel = std::max(g_dynamicDiagErrorLevel, errLevel);
      }
  }

void
aggregateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &wrapper)
{
  wrapper.clear();
  wrapper.clearSummary();

  int maxErrLevel = std::max(g_staticDiagErrorLevel, g_dynamicDiagErrorLevel);

  // Report the overall status
  switch (maxErrLevel)
    {
    case diagnostic_msgs::DiagnosticStatus::ERROR:
      wrapper.summary(maxErrLevel,
		      "Erroneous data or settings detected for a robot_localization state estimation node.");
      break;
    case diagnostic_msgs::DiagnosticStatus::WARN:
      wrapper.summary(maxErrLevel,
		      "Potentially erroneous data or settings detected for "
		      "a robot_localization state estimation node.");
      break;
    case diagnostic_msgs::DiagnosticStatus::STALE:
      wrapper.summary(maxErrLevel,
		      "The state of the robot_localization state estimation node is stale.");
      break;
    case diagnostic_msgs::DiagnosticStatus::OK:
      wrapper.summary(maxErrLevel,
		      "The robot_localization state estimation node appears to be functioning properly.");
      break;
    default:
      break;
    }

  // Aggregate all the static messages
  for (std::map<std::string, std::string>::iterator diagIt = g_staticDiagnostics.begin();
       diagIt != g_staticDiagnostics.end();
       ++diagIt)
    {
      wrapper.add(diagIt->first, diagIt->second);
    }

  // Aggregate all the dynamic messages, then clear them
  for (std::map<std::string, std::string>::iterator diagIt = g_dynamicDiagnostics.begin();
       diagIt != g_dynamicDiagnostics.end();
       ++diagIt)
    {
      wrapper.add(diagIt->first, diagIt->second);
    }
  g_dynamicDiagnostics.clear();

  // Reset the warning level for the dynamic diagnostic messages
  g_dynamicDiagErrorLevel = diagnostic_msgs::DiagnosticStatus::OK;
}


void xbeeRcvCallback(const nccr_manet_msgs::XbeePacket::ConstPtr& msg)
{
  ROS_INFO("Got xbee packet size [%d]", msg->data.size());

  using namespace xbee_app_data;
  const unsigned char *data = &msg->data[0];
  
  size_t len = msg->data.size();
  if (len > sizeof(Header))
    {
      Header header;
      memcpy(&header, data, sizeof(Header));

      if( g_nPacketsRcv.find( header.src ) == g_nPacketsRcv.end())
	g_nPacketsRcv[header.src]=0;
      g_nPacketsRcv[header.src]++;

      /// check sequence number consistency
      if( g_lastSeqnRcv.find( header.src) == g_lastSeqnRcv.end())
	{
	  g_lastSeqnRcv[header.src] = header.seqn;
	}
      else
	{
	  if( header.seqn > g_lastSeqnRcv[header.src] + 1)
	    {
	      int missing_n = +(header.seqn - g_lastSeqnRcv[header.src]);
	      std::stringstream stream_msg;
	      stream_msg << "Detected " << missing_n << " packets from node "
			  << +(header.src);
	      std::stringstream stream_topic;
	      stream_topic << "topic";
	      addDiagnostic(diagnostic_msgs::DiagnosticStatus::WARN,
			    stream_topic.str(),
			    stream_msg.str(),
			    true);
	      //	      LOG(WARNING) << "MISSINGNPACKETSEQN NODE " << +header.src
	      //	   << " " << missing_n;
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
	      /*
	      g_gpsDriver->notifyPos(header.src, eInfo.longitude,
				     eInfo.latitude,
				     eInfo.altitude,
				     g_epsg);
	      */
				       
		
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
	      //LOG(INFO) << "EndNodeDebug from " << +header.src
	      //		<< ": " << eDebug;
	      nccr_manet_msgs::EndNodeXbeeStatus msg;
	      msg.packets_sent = eDebug.n_xbee_pkts_sent;
	      msg.packets_received = eDebug.n_xbee_pkts_rcv;
	      msg.bytes_sent = eDebug.n_xbee_bytes_sent;
	      msg.bytes_received = eDebug.n_xbee_bytes_rcv;
	      xbeestatus_pub.publish(msg);
		
	      
	      
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

void
sendRoutingDataTimerCB(const ros::TimerEvent&)
{
  
}

void
sendPlanningDataTimerCB(const ros::TimerEvent&)
{
  
}

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "manet_manager");
  ros::NodeHandle nh;

  // Create a new NodeExample object.

  //  NodeExample *node_example = new NodeExample();
  diagnostic_updater::Updater updater;
  updater.setHardwareID("none");
  updater.add("packet updater", dummy_diagnostic);
  updater.add("Function updater", aggregateDiagnostics);
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
  string topic;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can
  // be run simultaneously while using different parameters.
  // Parameters defined in the .cfg file do not need to be initialized here
  // as the dynamic_reconfigure::Server does this for you.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, int(40));
  private_node_handle_.param("topic", topic, string("example"));


  string  addrBook;
  int epsg;
  int nodeId;
  
  private_node_handle_.param("nodeid", nodeId, int(1));
  private_node_handle_.param("abook", addrBook, string("none"));
  private_node_handle_.param("epsg", g_epsg, int(32632));

  g_nodeId = static_cast<uint8_t>(nodeId);


  

  // Tell ROS how fast to run this node.
  ros::Rate r(rate);


  g_abort = false;
  g_lastRoutingDataSent.timestamp = 0;
  g_lastPlanningDataSent.timestamp = 0;
  g_nPacketsSent=0;
  g_seqn = 0;
    

  g_routingDriver = NULL;
  g_sendRoutingDataTimer = NULL;
  g_planningDriver = NULL;
  g_sendPlanningDataTimer = NULL;
  g_flowNotifier = NULL;
  g_gpsDriver = NULL;
  g_debugManager = NULL;

  ros::Subscriber sub = nh.subscribe("xbee_in", 1000, xbeeRcvCallback);
  
  xbeestatus_pub =
    nh.advertise<nccr_manet_msgs::EndNodeXbeeStatus>("endnode_xbee_status", 10);


  /// create routing Driver
  g_routingDriver = new ROUTINGDriver("udpm://239.255.76.67:7667?ttl=0", "RNP2", true);
  /// Periodically send the Routing Table

  ros::Timer routing_timer = nh.createTimer(ros::Duration(1), sendRoutingDataTimerCB);
  //  g_sendRoutingDataTimer = new Timer(TIMER_SECONDS, sendRoutingDataTimerCB, NULL);
  //g_sendRoutingDataTimer->startPeriodic(1);
  /// create planning Driver
  g_planningDriver = new PLANNINGDriver("udpm://239.255.76.67:7667?ttl=0", "PLAN", true);
  /// Periodically send the Plan
  ros::Timer planning_timer = nh.createTimer(ros::Duration(1), sendPlanningDataTimerCB);

  g_flowNotifier = new FlowNotifier("udpm://239.255.76.67:7667?ttl=0", "iflowip", false);
  g_gpsDriver = new GPSDriver(g_nodeId, "udpm://239.255.76.67:7667?ttl=0", "RCVPOSEGPS", false, false);

  g_debugManager = new DebugManager("udpm://239.255.76.67:7667?ttl=0",
				    "XBEEDEBUG", false);
    
  if( addrBook != "none" )
    {
      g_flowNotifier->readAddressBook(addrBook);
    }


  
  // Main loop.
  while (nh.ok())
  {
    // Publish the message.
    //    node_example->publishMessage(&pub_message);

    ros::spinOnce();
    r.sleep();
    updater.update();
  }

  return 0;
} // end main()
