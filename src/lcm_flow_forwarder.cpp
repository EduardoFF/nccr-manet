//#include "node_example_core.h"
#include "ros/ros.h"

#include <iostream>
#include <stdlib.h>
#include "xbee_app_data.h"
#include <csignal>
#define DISABLE_GLOG 1
#include "flow_notifier.h"
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <nccr_manet_msgs/FlowStatus.h>
#include <nccr_manet_msgs/FlowList.h>

using namespace std;

#ifndef IT
#define IT(c) __typeof((c).begin())
#endif

#ifndef FOREACH
#define FOREACH(i,c) for(__typeof((c).begin()) i=(c).begin();i!=(c).end();++i)
#endif



std::map<std::string, std::string> g_staticDiagnostics, g_dynamicDiagnostics;
int g_dynamicDiagErrorLevel, g_staticDiagErrorLevel;

ros::Publisher flow_pub;
FlowNotifier *g_flowNotifier;

uint8_t g_nodeId;
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
publishFlowData()
{
  /// lock the mutex first
  using namespace xbee_app_data;
  FlowList fList = g_flowNotifier->getFlows(2000);
  if( !fList.size()  )
    {
      ROS_INFO("No flows to notify");
      return;
    }
  ROS_INFO("Notifying %u flows",fList.size());

  /// lock the mutex first
  using namespace xbee_app_data;


  nccr_manet_msgs::FlowList omsg;
  omsg.node_id = g_nodeId;
  FOREACH(it, fList )
    {
      nccr_manet_msgs::FlowStatus fInfo;
      fInfo.sender_id = g_flowNotifier->getIdFromAddressBook(it->src_addr);
      if( fInfo.sender_id <= 0)
	continue;
      fInfo.receiver_id = g_flowNotifier->getIdFromAddressBook(it->dst_addr);
      /// only publish flows whose destination address matches this node's id
      if( fInfo.receiver_id != g_nodeId )
	{
	  cout << "Rejected flow from " << (int) fInfo.sender_id
	       << " to " << it->dst_addr.toString() << endl;
	continue;
	}
      fInfo.rate = it->data_rate;
      omsg.flows.push_back(fInfo);
    }
  flow_pub.publish(omsg);
}


/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "lcm_flow_forwarder");
  ros::NodeHandle nh;

  // Create a new NodeExample object.

  //  NodeExample *node_example = new NodeExample();
  diagnostic_updater::Updater updater;
  updater.setHardwareID("none");
  updater.add("Flows", dummy_diagnostic);
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
  int nodeId;
  string addrBook;
  string lcm_url;
  private_node_handle_.param("nodeid", nodeId, int(1));
  g_nodeId = static_cast<uint8_t>(nodeId);
  private_node_handle_.param("lcm_topic", lcm_topic, string("mineiflow"));
  private_node_handle_.param("out_topic", out_topic, string("out_flows"));
  private_node_handle_.param("address_book", addrBook, string("none"));
  private_node_handle_.param("lcm_url", lcm_url, string("udpm://239.255.76.67:7667?ttl=1"));


  // Tell ROS how fast to run this node.
  ros::Rate r(rate);
  
  flow_pub =
    nh.advertise<nccr_manet_msgs::FlowList>(out_topic.c_str(), 10);

  g_flowNotifier = new FlowNotifier(lcm_url.c_str(), lcm_topic.c_str(), true);
  
    if( addrBook != "none" )
      g_flowNotifier->readAddressBook(addrBook);

  // Main loop.
  while (nh.ok())
  {
    // Publish the message.
    //    node_example->publishMessage(&pub_message);

    ros::spinOnce();
    r.sleep();
    updater.update();
    publishFlowData();
  }

  return 0;
} // end main()
