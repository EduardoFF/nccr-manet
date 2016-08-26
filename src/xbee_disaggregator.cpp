//#include "node_example_core.h"
#include "ros/ros.h"
#include <iostream>
#include <stdlib.h>
#include "xbee_app_data.h"
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <nccr_manet_msgs/XbeePacket.h>
#include <std_msgs/UInt32.h>


using namespace std;

#ifndef IT
#define IT(c) __typeof((c).begin())
#endif

#ifndef FOREACH
#define FOREACH(i,c) for(__typeof((c).begin()) i=(c).begin();i!=(c).end();++i)
#endif





std::map<std::string, std::string> g_staticDiagnostics, g_dynamicDiagnostics;
int g_dynamicDiagErrorLevel, g_staticDiagErrorLevel;

std::map< char, ros::Publisher > g_xbeeDataTypeToPublisher;
std::map< char, ros::Publisher > g_xbeeCntToPublisher;
std::map< char, int > g_xbeeDataTypeCnt;
std::map< char, int> g_nPacketsRcv;
std::map< char, int> g_nBytesRcv;
  

using namespace std;

void
dummy_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
	       "Reception Summary");
  FOREACH(it, g_nPacketsRcv)
    {
      stringstream ss;
      ss << "PacketCnt (" << +(it->first)
	 << ")";
      stat.addf(ss.str().c_str(),
		"%d",it->second);
      ss.str("");
      ss << "BytesCnt (" << +(it->first)
	 << ")";
	    
      stat.addf(ss.str().c_str(),
	       "%d",g_nBytesRcv[it->first]);       

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
      g_nPacketsRcv[header.src]++;
      g_nBytesRcv[header.src]+=len;

      if (header.type == XBEEDATA_ENDNODEINFO )               /// Check the type of Header
        {
	  if(len == sizeof(Header) + sizeof(EndNodeInfo))     /// Packet is of proper size
            {
	      g_xbeeDataTypeToPublisher[header.type].publish(*msg);
	      g_xbeeDataTypeCnt[header.type]++;
            }
	  else
            {
	      g_xbeeDataTypeToPublisher[XBEEDATA_INVALID].publish(*msg);
	      g_xbeeDataTypeCnt[XBEEDATA_INVALID]++;
            }
        }

      if (header.type == XBEEDATA_ENDNODEDEBUG )        /// Check the type of Header
        {
	  if(len == sizeof(Header) + sizeof(EndNodeDebug))     /// Packet is of proper size
            {
	       g_xbeeDataTypeToPublisher[header.type].publish(*msg);
	       g_xbeeDataTypeCnt[header.type]++;
            }
	  else
            {
	      g_xbeeDataTypeToPublisher[XBEEDATA_INVALID].publish(*msg);
	      g_xbeeDataTypeCnt[XBEEDATA_INVALID]++;
            }
        }
		
      if (header.type == XBEEDATA_FLOWINFO )  /// Check the type of Header
        {
	  if( len >= sizeof(Header) + sizeof(FlowInfoHdr) )
	    {
	       g_xbeeDataTypeToPublisher[header.type].publish(*msg);
	       g_xbeeDataTypeCnt[header.type]++;
	    }
	  else
	    {
	      g_xbeeDataTypeToPublisher[XBEEDATA_INVALID].publish(*msg);
	      g_xbeeDataTypeCnt[XBEEDATA_INVALID]++;
	    }
	}
    }
}



/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "xbee_disaggregator");
  ros::NodeHandle nh;

  // Create a new NodeExample object.

  //  NodeExample *node_example = new NodeExample();
  diagnostic_updater::Updater updater;
  updater.setHardwareID("none");
  updater.add("Xbee Reception State", dummy_diagnostic);
  updater.add("Function updater", aggregateDiagnostics);
  // Set up a dynamic reconfigure server.
  // This should be done before reading parameter server values.
  //dynamic_reconfigure::Server<node_example::node_example_paramsConfig> dr_srv;
  //dynamic_reconfigure::Server<node_example::node_example_paramsConfig>::CallbackType cb;
  //cb = boost::bind(&NodeExample::configCallback, node_example, _1, _2);
  //dr_srv.setCallback(cb);

  // Declare variables that can be modified by launch file or command line.
  int rate;

  std::map< char, string> topics;
  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can
  // be run simultaneously while using different parameters.
  // Parameters defined in the .cfg file do not need to be initialized here
  // as the dynamic_reconfigure::Server does this for you.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, int(1));
  private_node_handle_.param("endnodeinfo_topic",
			     topics[xbee_app_data::XBEEDATA_ENDNODEINFO],
			     string("xbee_endnodeinfo"));
  private_node_handle_.param("routing_topic",
			     topics[xbee_app_data::XBEEDATA_ROUTING],
			     string("xbee_routing"));
  private_node_handle_.param("planning_topic",
			      topics[xbee_app_data::XBEEDATA_PLANNING],
			     string("xbee_planning"));
  private_node_handle_.param("flowinfo_topic",
			     topics[xbee_app_data::XBEEDATA_FLOWINFO],
			     string("xbee_flowinfo"));
  private_node_handle_.param("endnodebug_topic",
			     topics[xbee_app_data::XBEEDATA_ENDNODEDEBUG],
			     string("xbee_endnodedebug"));
  private_node_handle_.param("invalid_topic",
			      topics[xbee_app_data::XBEEDATA_INVALID],
			     string("xbee_invalid"));
  

  // Tell ROS how fast to run this node.
  ros::Rate r(rate);
    

  ros::Subscriber sub = nh.subscribe("xbee_in", 1000, xbeeRcvCallback);

  FOREACH(it, topics)
    {
      g_xbeeDataTypeToPublisher[it->first] =
	nh.advertise<nccr_manet_msgs::XbeePacket>( (it->second).c_str(), 10);
      stringstream topic_cnt_ss;
      topic_cnt_ss << it->second << "_cnt";
      g_xbeeCntToPublisher[it->first] =
	nh.advertise<std_msgs::UInt32>( topic_cnt_ss.str().c_str(), 10);
    }
    

  
  // Main loop.
  while (nh.ok())
  {
    // Publish the message.
    //    node_example->publishMessage(&pub_message);

    ros::spinOnce();
    r.sleep();
    FOREACH(it, g_xbeeCntToPublisher)
      {
	std_msgs::UInt32 msg;
	msg.data = g_xbeeDataTypeCnt[it->first];
	(it->second).publish(msg);
      }
    updater.update();
  }

  return 0;
} // end main()
