#ifndef XBEE_APP_DATA_H
#define XBEE_APP_DATA_H
/*
    xbee_app_data.{cc,h} - application data sent through XBee


    Author:		        Eduardo Feo Flushing
                    Dalle Molle Institute for Artificial Intelligence
                IDSIA - Manno - Lugano - Switzerland
                (eduardo <at> idsia.ch)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.	If not, see <http://www.gnu.org/licenses/>.
*/
#include <iostream>
#include <unistd.h>
#include "route2_table_t.hpp"
#include "plan2_table_t.hpp"

namespace xbee_app_data
{

    typedef struct Packet Packet;
    typedef uint8_t NodeId;
    static const char     XBEEDATA_INVALID          = 0;
    static const char     XBEEDATA_ENDNODEINFO      = 1;
    static const char     XBEEDATA_ROUTING          = 2;
    static const char     XBEEDATA_PLANNING         = 3;
    static const char     XBEEDATA_FLOWINFO         = 4;
    static const char     XBEEDATA_ENDNODEDEBUG     = 5;
    static const char     XBEE_MAX_PAYLOAD_LENGTH   = 127;
    static const uint16_t XBEE_BROADCAST_ADDR       = 65535;

    struct __attribute__((__packed__)) EndNodeInfo
    {
        float latitude;
        float longitude;
        float altitude;
        int dataRate;
        int lastTabId;
    };

    struct __attribute__((__packed__)) EndNodeDebug
    {
      uint16_t n_xbee_pkts_sent;
      uint32_t n_xbee_bytes_sent;
      uint16_t n_xbee_pkts_rcv;
      uint32_t n_xbee_bytes_rcv;
      uint64_t timestamp;
      uint16_t last_flow_notify_time; /// seconds until last
      
      char     manet_alive;
    };

    struct __attribute__((__packed__)) FlowInfoHdr
    {
      uint8_t nEntries;
    };

    struct __attribute__((__packed__)) FlowInfoEntry
    {
      NodeId nodeId;
      /// if negative, means incoming flow
      double  dataRate;
    };

    struct __attribute__((__packed__)) Routing
    {
        uint16_t tabId;
        uint8_t  fragNb;
        uint8_t  nbOfFrag;
        uint8_t  nbBytes;
    };

    struct __attribute__((__packed__)) RoutingTableHdr
    {
        NodeId  nodeId;
        uint8_t nEntries;
    };

    struct __attribute__((__packed__)) RoutingEntry
    {
        NodeId  dest;
        NodeId  nextHop;
        uint8_t weight;
    };

    struct __attribute__((__packed__)) Planning
    {
        uint16_t tabId;
        uint8_t  fragNb;
        uint8_t  nbOfFrag;
        uint8_t  nbBytes;
    };

    struct __attribute__((__packed__)) PlanningTableHdr
    {
        NodeId  nodeId;
        uint8_t nEntries;
    };

    struct __attribute__((__packed__)) PlanningEntry
    {
        float latitude;
        float longitude;
        float altitude;
	std::string action;
	std::string option;
        uint64_t timestamp;
    };

    struct __attribute__((__packed__)) Header
    {
        char   type;
	NodeId src;
	uint16_t seqn;
    };

};
std::ostream &operator<<(std::ostream &os, const xbee_app_data::EndNodeDebug &);
std::ostream &operator<<(std::ostream &os, const xbee_app_data::EndNodeInfo &);
std::ostream &operator<<(std::ostream &os, const xbee_app_data::Routing &);
std::ostream &operator<<(std::ostream &os, const xbee_app_data::Planning &);
std::ostream &operator<<(std::ostream &os, const xbee_app_data::Header &);

unsigned long
checksum(unsigned char *str, size_t len);

#endif
