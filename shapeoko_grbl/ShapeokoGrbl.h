///////////////////////////////////////////////////////////////////////////////
// FILE:          ShapeokoGrbl.h
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   The example implementation of the demo camera.
//                Simulates generic digital camera and associated automated
//                microscope devices and enables testing of the rest of the
//                system without the need to connect to the actual hardware. 
//                
// AUTHOR:        Nenad Amodaj, nenad@amodaj.com, 06/08/2005
//                
//                Karl Hoover (stuff such as programmable CCD size  & the various image processors)
//                Arther Edelstein ( equipment error simulation)
//
// COPYRIGHT:     University of California, San Francisco, 2006-2015
//                100X Imaging Inc, 2008
//
// LICENSE:       This file is distributed under the BSD license.
//                License text is included with the source distribution.
//
//                This file is distributed in the hope that it will be useful,
//                but WITHOUT ANY WARRANTY; without even the implied warranty
//                of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
//                IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//                CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//                INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES.

#ifndef _SHAPEOKO_GRBL_H_
#define _SHAPEOKO_GRBL_H_

#include "DeviceBase.h"
#include "DeviceThreads.h"
#include <string>
#include <map>
#include <algorithm>

//////////////////////////////////////////////////////////////////////////////
// Error codes
//
// #define ERR_UNKNOWN_MODE         102
// #define ERR_UNKNOWN_POSITION     103
// #define ERR_IN_SEQUENCE          104
// #define ERR_SEQUENCE_INACTIVE    105
#define ERR_STAGE_MOVING         110

#define ERR_UNKNOWN_POSITION 101
#define ERR_INITIALIZE_FAILED 102
#define ERR_WRITE_FAILED 103
#define ERR_CLOSE_FAILED 104
#define ERR_BOARD_NOT_FOUND 105
#define ERR_PORT_OPEN_FAILED 106
#define ERR_COMMUNICATION 107
#define ERR_NO_PORT_SET 108
#define ERR_VERSION_MISMATCH 109


////////////////////////
// ShapeokoGrblHub
//////////////////////

class ShapeokoGrblHub : public HubBase<ShapeokoGrblHub>
{
public:
  ShapeokoGrblHub();
  ~ShapeokoGrblHub() { Shutdown();}

   // Device API
   // ---------
   int Initialize();
  int Shutdown() {initialized_ = false; return DEVICE_OK;};
   void GetName(char* pName) const; 
   bool Busy() { return busy_;} ;

   // property handlers
  int OnVersion(MM::PropertyBase* pProp, MM::ActionType pAct);
   int OnPort(MM::PropertyBase* pPropt, MM::ActionType eAct);
   int OnCommand(MM::PropertyBase* pProp, MM::ActionType pAct);

   // HUB api
   int DetectInstalledDevices();

  int SendCommand(std::string command, std::string terminator="\r");
  int ReceiveResponse(std::string &returnString, float timeout = 300.0);
   int SetAnswerTimeoutMs(double timout);
   MM::DeviceDetectionStatus DetectDevice(void);
   int PurgeComPortH() {return PurgeComPort(port_.c_str());}
   int WriteToComPortH(const unsigned char* command, unsigned len) {return WriteToComPort(port_.c_str(), command, len);}
   int ReadFromComPortH(unsigned char* answer, unsigned maxLen, unsigned long& bytesRead)
   {
      return ReadFromComPort(port_.c_str(), answer, maxLen, bytesRead);
   }
   int SetCommandComPortH(const char* command, const char* term)
   {
           return SendSerialCommand(port_.c_str(),command,term);
   }
    int GetSerialAnswerComPortH (std::string& ans,  const char* term)
        {
                return GetSerialAnswer(port_.c_str(),term,ans);
        }
  int GetStatus(); 
  std::string GetState(); 
  void GetPos(float &x, float &y); 
  int ResetDevice();
  int GetControllerVersion(std::string& version);

private:
   void GetPeripheralInventory();
   std::vector<std::string> peripherals_;
   bool initialized_;
   bool busy_;
  std::string version_;
   MMThreadLock lock_;
   MMThreadLock executeLock_;
   std::string port_;
   bool portAvailable_;
   std::string commandResult_;
  std::string state_;
   double MPos[3];
   double WPos[3];
};


#endif //_SHAPEOKO_GRBL_H_
