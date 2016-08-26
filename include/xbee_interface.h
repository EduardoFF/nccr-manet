#ifndef XBEE_INTERFACE_H
#define XBEE_INTERFACE_H
/*
    xbee_interface.{cc,h} - a front-end of libxbee to simplify its use
             It works with Xbee series 1 using 16-bit addressing

    Author:		        Eduardo Feo
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

#include <map>
#include <string>
#include <stdint.h>

namespace libxbee{
class XBee;
class Pkt;
class Con;
}

class MyConnection;
//! GetPot stuff
#define STR_EXPAND(tok) #tok
#define STR(tok) STR_EXPAND(tok)
#define GETPOTIFY(g,prefix, x) x = (g)(STR(prefix) STR(x), x)
#define GETPOTIFYSTR(g,prefix, x) x = (g)(STR(prefix) STR(x), (x).c_str())
#define IT(x) __typeof((x).begin())


struct XbeeInterfaceParam
{
    //! Networking
    uint8_t Channel;
    uint16_t PanId;
    std::string  NodeIdentifier;
    uint16_t SourceAddress;

    //! RF Interfacing
    uint8_t PowerLevel;
    uint8_t CCATreshold;


    //! LibXbee params
    uint32_t  brate;
    std::string    mode;
    std::string    Device;
    bool       writeParams;
    XbeeInterfaceParam()
    {
        Channel = 26;
        PanId = 13106;
        NodeIdentifier = "Default";
        SourceAddress = 0;
        PowerLevel = 1;

        brate = 57600;
        mode  = "xbee1";
        Device = "/dev/ttyUSB0";
        writeParams = false;
    }
    void loadFromFile(const char *cfg);


};

class MyConnection;


class XbeeInterface
{

    friend class MyConnection;

    /// This is what we need in the upper layers
    /// addr: who sent the pkt
    /// data: pointer to buffer
    /// rssi: rssi of pkt
    /// timestamp ? of reception?
    /// size: size of buffer
    typedef void (*ReceiveCB)(uint16_t addr,
                              void *data,
                              char rssi,
                              timespec timestamp,
                              size_t size);
    libxbee::XBee *mXbee;
    XbeeInterfaceParam mParam;
    //MyConnection *mATCon;
    libxbee::Con *mATCon;
    std::map< uint16_t, MyConnection *> mConn;
    MyConnection *mDefaultCon;
    ReceiveCB mReceiveCB;
    bool m_ok;

    MyConnection *createConnection(uint16_t addr);
    void setup();
    

    void receive(libxbee::Pkt **pkt);
    bool init();
    
public:
    enum TxStatus{
        NO_ACK,  		//! Tx failed due to missing ack
        TX_CCA0, 		//! Tx succesful with 0 CCA failures
        TX_CCA1, 		//! Tx succesful with 1 CCA failure
        TX_CCA2,
        TX_CCA3,
        TX_CCA4,
        TX_MAC_BUSY, 	//! Tx failed due to CSMA MAC failure (channel busy)
	TX_DEVICE_ERROR,
    } ;

    typedef struct {
        bool reqAck; 	//! Set it if ACK is required
        bool readCCA;
        TxStatus status; 	//! TX status (set by 'send' method)
    } TxInfo;

    /** configuration file (see GetPot reference file)
   * */
    XbeeInterface(const char *cfgfile=NULL);
    XbeeInterface(const XbeeInterfaceParam &par);
    ~XbeeInterface();

    /** send a packet to
   * */
    int send(uint16_t addr, TxInfo &, const void *buf, size_t len);

    /** register a receive callback, which is called when a packet
   *  is received
   *  */
    void registerReceive(ReceiveCB);

    bool isOK();
    void sendReset();
    bool checkAlive();

};
#endif
