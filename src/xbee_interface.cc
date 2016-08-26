#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <GetPot.hpp>
#include <list>

#include "xbeep.h"
#include "xbee_interface.h"
#include "dbg.h"

using namespace libxbee;
using namespace std;

/* ========================================================================== */

#define __XBEE_868 1
char rcv_msg[130];

class MyConnection: public libxbee::ConCallback {
public:
  XbeeInterface *mParent;
  explicit MyConnection(XbeeInterface *parent,
			std::string type, 
			struct xbee_conAddress *address = NULL): 
    mParent(parent),
    libxbee::ConCallback(*(parent->mXbee), type, address) 
  {};
  void xbee_conCallback(libxbee::Pkt **pkt);
};

void MyConnection::xbee_conCallback(libxbee::Pkt **pkt) {
  int i;
  debug("Response is %d bytes long:", (*pkt)->size());

#ifndef NDEBUG
  for (i = 0; i < (*pkt)->size(); i++) {
    std::cout << "[" <<  (**pkt)[i] << "]";
    printf("%3d: 0x%02x - %c - %X\n" 
	   ,i
	   ,(unsigned char)(**pkt)[i]
	   ,((((**pkt)[i] >= ' ') && ((**pkt)[i] <= '~'))?(**pkt)[i]:'.')
	   ,(**pkt)[i] );
  }
  std::cout << "\n";
#endif

  /// Promote to parent
  mParent->receive(pkt);

  //std::cout << myData;

  /* if you want to keep the packet, then you MUST do the following:
     libxbee::Pkt *myhandle = *pkt;
     *pkt = NULL;
     and then later, you MUST delete the packet to free up the memory:
     delete myhandle;

     if you do not want to keep the packet, then just leave everything as-is, and it will be free'd for you */
}

/* ========================================================================== */


//template<typename T>
//T readParameter(libxbee::Con *con, std::string par)
//{

//}

/// Sets a parameter using a Local AT Connection
/// values are expressed as decimal, so this method makes the conversion
/// nbytes should be between 1 and 4
void
setParameter(libxbee::Con *con, std::string par, uint32_t val, uint8_t nbytes)
{
  debug("setting parameter %s to %d", par.c_str(), val);

  char cmd[10];
  unsigned int b[4];
  if( nbytes > 4)
    {
      log_err("Error: Invalid parameter (nbytes = %d)", nbytes);
      return;
    }
  if( par.size() > 2)
    {
      log_err("Warning: parameter name (%s) has size > 2",
	      par.c_str());
    }

  for(int i=0; i< nbytes; i++)
    {
      b[i] =  ( ( val >> (8*(i))) & 0xff);
      debug("byte %d 0x%02X %d %x", 
	    i, b[i], 
	    b[i],
	    (unsigned)(unsigned char)b[i]);
    }

  sprintf(cmd, "%s", par.c_str());
  for(int i=0; i< nbytes; i++)
    {
      cmd[par.size()+(nbytes-i-1)]=(unsigned char)b[i];
    }
  cmd[par.size()+nbytes] = '\0';
  debug("sending cmd <%s> size %ld", cmd, par.size()+nbytes);
  con->Tx((const unsigned char *)cmd, par.size()+nbytes);
}


void
XbeeInterfaceParam::loadFromFile(const char *config_file)
{
  debug("Reading XBee configuration from file: %s",
	config_file);

  GetPot ifile(config_file);
  GETPOTIFYSTR(ifile, NETWORKING/, NodeIdentifier);
  GETPOTIFY(ifile, NETWORKING/, SourceAddress);
  GETPOTIFY(ifile, NETWORKING/, PanId);
  GETPOTIFY(ifile, RF/, PowerLevel);
  GETPOTIFYSTR(ifile, SERIAL/,Device);
}


void 
XbeeInterface::setup()
{
  ///
#ifdef __XBEE_868
  //  setParameter(mATCon, "NI", (uint16_t) mParam.SourceAddress, 2);
#else
  setParameter(mATCon, "MY", (uint16_t) mParam.SourceAddress, 2);
#endif
  setParameter(mATCon, "ID", mParam.PanId, 2); 
  //setParameter(mATCon, "PL", mParam.PowerLevel, 1);
  if( mParam.writeParams )
    setParameter(mATCon, "WR", 0, 0);
}

MyConnection *
XbeeInterface::createConnection(uint16_t addr)
{
  xbee_conAddress *xbeeAddr = new xbee_conAddress();
  #ifdef __XBEE_868
  xbeeAddr->addr64_enabled = 1;
  xbeeAddr->addr16_enabled = 0;
  xbeeAddr->addr64[7] = 0xff;
  xbeeAddr->addr64[6] = 0xff;
  xbeeAddr->addr64[5] = 0;
  xbeeAddr->addr64[4] = 0;
  xbeeAddr->addr64[3] = 0;
  xbeeAddr->addr64[2] = 0;
  xbeeAddr->addr64[1] = 0;
  xbeeAddr->addr64[0] = 0;
  debug("creating Data connection\n");
  MyConnection *con =  
    new MyConnection(this, "Data", xbeeAddr);
  if( addr == 0xffff ) /// broadcast!
    {
      debug("connection is broadcast\n");
      struct xbee_conSettings settings;
      con->getSettings(&settings);
      settings.disableAck = 1;
      settings.catchAll = 1;
      con->setSettings(&settings);
    }
  else
    {
      debug("connection is NOT broadcast\n");
    }
#else
  xbeeAddr->addr64_enabled = 0;
  xbeeAddr->addr16_enabled = 1;
  xbeeAddr->addr16[1] = ((addr >> 0) & 0xff);
  xbeeAddr->addr16[0] = ((addr >> 8) & 0xff);
    //TODO Find out if libxbee takes ownership of xbeeAddr
  MyConnection *con =  
    new MyConnection(this, "16-bit Data", xbeeAddr);
  if( addr == 0xffff ) /// broadcast!
    {
      struct xbee_conSettings settings;
      con->getSettings(&settings);
      settings.disableAck = 1;
      settings.catchAll = 1;
      con->setSettings(&settings);
    }
  #endif


  mConn[addr] = con;
  debug("connection to %d - done", addr);
  return con;
}

bool
XbeeInterface::checkAlive()
{
  try
  {
    debug("checking alive");
    (*mATCon) << "AP";//    or like this 
  }
 catch (...)
   {
     debug("not alive");
     return false;
   }
  debug("alive");
 return true;
}

void
XbeeInterface::sendReset()
{
    (*mATCon) << "FR";//    or like this 
    sleep(1);  
}
int 
XbeeInterface::send(uint16_t addr, TxInfo &txPar, const void *data, size_t size)
{
  MyConnection *con;
  IT(mConn) it = mConn.find(addr);
  if( it == mConn.end())
    {
      debug("Connection not found - creating new");
      con = createConnection(addr);
    } else
    {
      con = it->second;
    }
    bool xbee_error = false;
  try {
    unsigned char err; 
    bool restore_settings = false;
    struct xbee_conSettings prev_settings, settings;
    con->getSettings(&prev_settings);
    con->getSettings(&settings);

    
    if( txPar.reqAck )
      {
	if( settings.disableAck)
	  {
	    restore_settings = true;
	    settings.disableAck = 0;
	    con->setSettings(&settings);
	  }
      } else {
      if( !settings.disableAck )
	{
	  restore_settings = true;
	  settings.disableAck = 1;
	  con->setSettings(&settings);
	}
    }

    debug("Trying to transmit data packet to %d of size %ld",
	  addr, size);
#ifndef NDEBUG
    unsigned char *cdata = (unsigned char *)data;
    for (int i = 0; i < size; i++) {
      std::cout << "[" <<  (cdata)[i] << "]";
      printf("%3d: 0x%02x - %c - %X\n" 
	     ,i
	     ,(unsigned char)(cdata)[i]
	     ,((((cdata)[i] >= ' ') && ((cdata)[i] <= '~'))?(cdata)[i]:'.')
	     ,(cdata)[i] );
    }
    std::cout << "\n";
#endif

    try
      {  
	if( err = con->Tx((const unsigned char *)data, (int) size) )
	  {
	    debug("xbee ret %d", err);
	    log_info("xbee tx returned %d", err);
	    xbee_error = (err != XBEE_ENONE );
	    printf("xbee tx returned %d", err);
	  }
      }
    catch (xbee_err ret)
      {
	log_err("xbee exception %d", ret);
	xbee_error = true;
      }
    catch (...)
      {
	log_err("xbee exception ");
	xbee_error = true;
      }
    

    if( restore_settings)
      con->setSettings(&prev_settings);
  } catch (xbee_err ret) {
    log_err("xbee exception %d", ret);
    xbee_error = true;
  } catch ( xbee_etx etx )
    {
      xbee_error = true;
      log_err("xbee etx exception err %d retval %d",
	      etx.ret, etx.retVal);
    }
  catch (...)
    {
      xbee_error = true;
      printf("error!\n");
    }

  if( xbee_error )
    return TX_DEVICE_ERROR;

  if( txPar.readCCA )
    {
      std::vector<char> ret_data;
      debug("requesting EC");
      (*mATCon) << "EC";
      (*mATCon) >> ret_data;
      debug("got EC response - size %ld", ret_data.size());
      uint16_t ec = 0xffff;
      if( ret_data.size() == 2)
	{
	  debug("ec response %02x %02x",ret_data[0], ret_data[1]);
	  ec = 0;
	  ec = (ec & 0xff00 ) | (ret_data[1]);
	  ec = (ec & 0x00ff )  | (ret_data[0] << 8);
	  debug("EC = %d", ec);
	} else {
	debug("can not determine CCA");
      }
      //setParameter(
    }
  txPar.status = TX_CCA1;
  debug("send done");
  return txPar.status;
}

void
XbeeInterface::receive(libxbee::Pkt **pkt)
{
  if(!pkt)
    return;
  debug("got valid packet from %s", (*pkt)->getHnd()->conType);
  if( strcmp((*pkt)->getHnd()->conType,"16-bit Data")==0)
    {
      if( mReceiveCB )
	{
	  struct xbee_pkt *xpkt = (*pkt)->getHnd();
	  uint16_t addr;
	  addr = (addr & 0xff00 ) | (xpkt->address.addr16[1]);
	  addr = (addr & 0x00ff )  | (xpkt->address.addr16[0] << 8);
	  debug("translated addr %d %d to %d", 
		xpkt->address.addr16[0],
		xpkt->address.addr16[1],
		addr);
	  unsigned char rssi = xpkt->rssi;
	  timespec timestamp = xpkt->timestamp;
	  int dataLen = xpkt->dataLen;
	  memcpy(rcv_msg, xpkt->data, dataLen);
	  debug("passing received packet with size %d", dataLen);
	  mReceiveCB(addr, rcv_msg, rssi, timestamp, dataLen);
	}

    }
  else if( strcmp((*pkt)->getHnd()->conType,"64-bit Data")==0
	   ||  strcmp((*pkt)->getHnd()->conType,"Data")==0 )
    {
      if( mReceiveCB )
	{
	  struct xbee_pkt *xpkt = (*pkt)->getHnd();
	  uint16_t addr;
	  addr = (addr & 0xff00 ) | (xpkt->address.addr64[1]);
	  addr = (addr & 0x00ff )  | (xpkt->address.addr64[0] << 8);
	  debug("translated addr %d %d to %d", 
		xpkt->address.addr64[0],
		xpkt->address.addr64[1],
		addr);
	  unsigned char rssi = xpkt->rssi;
	  timespec timestamp = xpkt->timestamp;
	  int dataLen = xpkt->dataLen;
	  memcpy(rcv_msg, xpkt->data, dataLen);
	  debug("passing received packet with size %d", dataLen);
	  mReceiveCB(addr, rcv_msg, rssi, timestamp, dataLen);
	}

    }
}
void
XbeeInterface::registerReceive(ReceiveCB cb)
{
  mReceiveCB = cb;
}

XbeeInterface::XbeeInterface(const XbeeInterfaceParam &par):
  mParam(par),
  mReceiveCB(NULL)
{
  if(init())
    m_ok = false;
  else
    m_ok = true;
}

bool
XbeeInterface::isOK()
{
  return m_ok;
}

XbeeInterface::XbeeInterface(const char *config_file):
  mReceiveCB(NULL)
{
  if( config_file != NULL )
    mParam.loadFromFile(config_file);
  init();
}

bool
XbeeInterface::init()
{  
  try {
    mXbee = new XBee(mParam.mode, mParam.Device, mParam.brate);
#ifndef NDEBUG
    /* get available connection types */
    try {
      std::list<std::string> types = mXbee->getConTypes();
      std::list<std::string>::iterator i;

      std::cout << "Available connection types:\n";
      for (i = types.begin(); i != types.end(); i++) {
	std::cout << "  " << *i;
      }
      std::cout << "\n";
    } catch (xbee_err ret) {
      std::cout << "Error while retrieving connection types...\n";
      return 1;
    }
#endif

    mXbee->setLogLevel(100);
    //mATCon = new MyConnection(this, "Local AT"); /* with a callback */

    mATCon = new libxbee::Con(*mXbee, "Local AT", NULL); /* without a callback */

    /* tried this to enable checkalive (blocks when modules is disconn)
       it does not work 
    struct xbee_conSettings settings;
    mATCon->getSettings(&settings);
    settings.noBlock = 1;
    settings.noWaitForAck
    mATCon->setSettings(&settings);
    */
    
   
    /*
      mDefaultCon = new MyConnection(this, "16-bit Data", NULL);
      struct xbee_conSettings settings;
      mDefaultCon->getSettings(&settings);
      settings.disableAck = 0;
      settings.catchAll = 1;
      mDefaultCon->setSettings(&settings);
    */

    #if 0
    //    con.myData = "Testing, 1... 2... 3...\n";
    printf("Requesting power level\n");
    (*mATCon) << "PL";//    or like this 
			    sleep(1);

    printf("Requesting node identifier\n");
    (*mATCon).Tx("NI");// like this 
			  sleep(1);

    printf("Requesting parameter AP\n");
    (*mATCon) << "AP";//    or like this 
			    sleep(1);

    printf("Requesting parameter ID\n");
    (*mATCon) << "ID";//    or like this 
			    sleep(1);

    printf("Requesting parameter MY\n");
    (*mATCon) << "MY";//    or like this 
    sleep(1);
#endif
    printf("Created AT connection\n");
    fflush(stdout);

    setup();
    printf("Done setup\n");
    fflush(stdout);

    mConn[0xffff] = createConnection(0xffff);
    printf("Created connection to 0xffff\n");
    fflush(stdout);
    return 0;
    
  } catch (xbee_err ret) {
    log_err("xbee exception %d", ret);
    return 1;
  }
  return 1;

}

XbeeInterface::~XbeeInterface()
{
  // Shutdown Xbee
  printf("shutting down XBee ...\n");
  if( mATCon )
    delete mATCon;
  if( mXbee )
    delete mXbee;
  
}
