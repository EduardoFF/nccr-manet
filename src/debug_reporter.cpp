//#include "node_example_core.h"
#include "ros/ros.h"

#include <iostream>
#include <stdlib.h>
#include "xbee_app_data.h"
#include "timer.h"
#include <csignal>
#include <glog/logging.h>
#include "flow_notifier.h"
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

std::map<std::string, std::string> g_staticDiagnostics, g_dynamicDiagnostics;
int g_dynamicDiagErrorLevel, g_staticDiagErrorLevel;


std::map<uint8_t, int> g_nPacketsRcv;

ros::Publisher xbeestatus_pub;

using namespace std;

void dummy_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{

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
	      msg.node_id = header.src;
	      msg.packets_sent = eDebug.n_xbee_pkts_sent;
	      msg.packets_received = eDebug.n_xbee_pkts_rcv;
	      msg.bytes_sent = eDebug.n_xbee_bytes_sent;
	      msg.bytes_received = eDebug.n_xbee_bytes_rcv;
	      xbeestatus_pub.publish(msg);
		
	      
	      
	      //	      g_debugManager->checkpoint(header.src, 
	      //			 g_nPacketsSent,
	      //			 g_nPacketsRcv[header.src]-1,
	      //			 eDebug);
	      //if( g_debugManager)
	      //	g_debugManager->publishLCM(header.src, eDebug);
            }
	  else
            {
	      LOG(INFO) << "Invalid length for EndNodeInfo msg";
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
  ros::init(argc, argv, "debug_reporter");
  ros::NodeHandle nh;

  // Create a new NodeExample object.

  //  NodeExample *node_example = new NodeExample();
  diagnostic_updater::Updater updater;
  updater.setHardwareID("none");
  updater.add("Debug", dummy_diagnostic);
  updater.add("Status updater", aggregateDiagnostics);
  // Set up a dynamic reconfigure server.
  // This should be done before reading parameter server values.
  //dynamic_reconfigure::Server<node_example::node_example_paramsConfig> dr_srv;
  //dynamic_reconfigure::Server<node_example::node_example_paramsConfig>::CallbackType cb;
  //cb = boost::bind(&NodeExample::configCallback, node_example, _1, _2);
  //dr_srv.setCallback(cb);

  // Declare variables that can be modified by launch file or command line.
  string in_topic;
  string out_topic;
  int rate;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can
  // be run simultaneously while using different parameters.
  // Parameters defined in the .cfg file do not need to be initialized here
  // as the dynamic_reconfigure::Server does this for you.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, int(2));
  private_node_handle_.param("in_topic", in_topic, string("xbee_endnodedebug"));
  private_node_handle_.param("out_topic", out_topic, string("endnode_xbee_status"));


  // Tell ROS how fast to run this node.
  ros::Rate r(rate);



  ros::Subscriber sub = nh.subscribe(in_topic.c_str(), 1000, xbeeRcvCallback);
  xbeestatus_pub =
    nh.advertise<nccr_manet_msgs::EndNodeXbeeStatus>(out_topic.c_str(), 10);
  


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
