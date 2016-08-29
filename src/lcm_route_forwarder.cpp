//#include "node_example_core.h"
#include "ros/ros.h"

#include <iostream>
#include <stdlib.h>
#include "xbee_app_data.h"
#include <csignal>
#include <glog/logging.h>
#include "routing_driver.h"
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
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

std::map<std::string, std::string> g_staticDiagnostics, g_dynamicDiagnostics;
int g_dynamicDiagErrorLevel, g_staticDiagErrorLevel;

ros::Publisher route_pub;
ROUTINGDriver *g_routingDriver;
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


TimestampedROUTINGData g_lastRoutingDataSent;


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

/// Function that sends the node info packet for routing Tables ///
void
sendRoutingData()
{
  /// lock the mutex first
  using namespace xbee_app_data;

  /// make payload
  /// Structure the data packets for Xbee
  TimestampedROUTINGData routingData = g_routingDriver->data();

  nccr_manet_msgs::ForwardingTableArray omsg;
  /// first, check if the routing data has changed by comparing the
  /// timestamps
  if( routingData.timestamp > g_lastRoutingDataSent.timestamp )
    {
      //      omsg.stamp = routingData.timestamp;
      /// it has changed ...
      /// we need to create new xbee messages
      IT(routingData.route) it = routingData.route.begin();
      while(it != routingData.route.end() )
	{
	  std::string nodedesc = it->first;
	  //LOG(INFO) << "pushing table of " << nodedesc;
	  RoutingTable table = it->second;
	  //LOG(INFO) << table.size() << " entries ";
	    /// can we fit all entries here ?
	  /// we need somehow to map node identifiers to numeric ids (0-255)
	  uint8_t nid = getNodeId(nodedesc);
	  nccr_manet_msgs::ForwardingTable ftable;
	  ftable.node = nid;

	  for(int j=0; j<table.size(); j++)
	    {
	      nccr_manet_msgs::ForwardingTableEntry rtentry;
	      rtentry.src = 0; /// we dont care about source of flow yet
	      rtentry.destination = getNodeId(table[j].endNode);
	      rtentry.next_hop = getNodeId(table[j].nextHop);
	      rtentry.weight = table[j].weight;
	      ftable.entries.push_back(rtentry);
	    }
	  omsg.tables.push_back(ftable);
	  it++;
	}
      /// we're done
      route_pub.publish(omsg);
      g_lastRoutingDataSent = routingData;
	
    }
  else
    {
      /// we don't need to encode again the xbee payload since the
      /// routing hasn't changed
      //LOG(INFO) << "routing unchanged ...";
    }
}


/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "lcm_route_forwarder");
  ros::NodeHandle nh;

  // Create a new NodeExample object.

  //  NodeExample *node_example = new NodeExample();
  diagnostic_updater::Updater updater;
  updater.setHardwareID("none");
  updater.add("LcmRoutes", dummy_diagnostic);
  updater.add("Status updater", aggregateDiagnostics);
  // Set up a dynamic reconfigure server.
  // This should be done before reading parameter server values.
  //dynamic_reconfigure::Server<node_example::node_example_paramsConfig> dr_srv;
  //dynamic_reconfigure::Server<node_example::node_example_paramsConfig>::CallbackType cb;
  //cb = boost::bind(&NodeExample::configCallback, node_example, _1, _2);
  //dr_srv.setCallback(cb);

  // Declare variables that can be modified by launch file or command line.
  string lcm_topic;
  string out_topic;
  int rate;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can
  // be run simultaneously while using different parameters.
  // Parameters defined in the .cfg file do not need to be initialized here
  // as the dynamic_reconfigure::Server does this for you.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, int(1));
  private_node_handle_.param("lcm_topic", lcm_topic, string("RNP2"));
  private_node_handle_.param("out_topic", out_topic, string("out_routes"));


  // Tell ROS how fast to run this node.
  ros::Rate r(rate);
  
  route_pub =
    nh.advertise<nccr_manet_msgs::ForwardingTableArray>(out_topic.c_str(), 10);

  g_routingDriver = new ROUTINGDriver("udpm://239.255.76.67:7667?ttl=0",
				      lcm_topic.c_str(), true);  


  // Main loop.
  while (nh.ok())
  {
    // Publish the message.
    //    node_example->publishMessage(&pub_message);

    ros::spinOnce();
    r.sleep();
    updater.update();
    sendRoutingData();
  }

  return 0;
} // end main()
