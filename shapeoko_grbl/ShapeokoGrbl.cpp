///////////////////////////////////////////////////////////////////////////////
// FILE:          ShapeokoTinyGCamera.cpp
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
// COPYRIGHT:     University of California, San Francisco, 2006
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

#include "ShapeokoGrbl.h"
#include "XYStage.h"
#include "ZStage.h"
#include <cstdio>
#include <string>
#include <math.h>
#include "ModuleInterface.h"
#include <sstream>
#include <algorithm>
#include <iostream>


using namespace std;

// External names used used by the rest of the system
// to load particular device from the "ShapeokoTinyGCamera.dll" library
const char* g_XYStageDeviceName = "DXYStage";
const char* g_ZStageDeviceName = "DZStage";
const char* g_HubDeviceName = "DHub";
const char* g_versionProp = "Version";

///////////////////////////////////////////////////////////////////////////////
// Exported MMDevice API
///////////////////////////////////////////////////////////////////////////////

MODULE_API void InitializeModuleData()
{
   RegisterDevice(g_XYStageDeviceName, MM::XYStageDevice, "ShapeokoGrbl XY stage");
     RegisterDevice(g_ZStageDeviceName, MM::StageDevice, "ShapeokoTinyG Z stage");
   RegisterDevice(g_HubDeviceName, MM::HubDevice, "DHub");
}

MODULE_API MM::Device* CreateDevice(const char* deviceName)
{
   if (deviceName == 0)
      return 0;

   if (strcmp(deviceName, g_XYStageDeviceName) == 0)
   {
      // create stage
      return new CShapeokoGrblXYStage();
   }
   else if (strcmp(deviceName, g_ZStageDeviceName) == 0)
   {
      // create stage
      return new ZStage();
   }
   else if (strcmp(deviceName, g_HubDeviceName) == 0)
   {
	  return new ShapeokoGrblHub();
   }

   // ...supplied name not recognized
   return 0;
}

MODULE_API void DeleteDevice(MM::Device* pDevice)
{
   delete pDevice;
}



ShapeokoGrblHub::ShapeokoGrblHub():
      initialized_(false),
      busy_(false)
{
  LogMessage("Constructor");
  WPos[0] = 0.0;
  WPos[1] = 0.0;
  WPos[2] = 0.0;
  MPos[0] = 0.0;
  MPos[1] = 0.0;
  MPos[2] = 0.0;

  CPropertyAction* pAct  = new CPropertyAction(this, &ShapeokoGrblHub::OnPort);
  CreateProperty(MM::g_Keyword_Port, "Undefined", MM::String, false, pAct, true);
}

int ShapeokoGrblHub::Initialize()
{
  LogMessage("Initialize");
   /* From EVA's XYStage */
   int ret = DEVICE_ERR;

   // initialize device and get hardware information


   // confirm that the device is supported


   // check if we are already homed

   CPropertyAction* pAct;

   // Step size
   // CreateProperty(g_StepSizeXProp, CDeviceUtils::ConvertToString(stepSizeUm), MM::Float, true);
   // CreateProperty(g_StepSizeYProp, CDeviceUtils::ConvertToString(stepSizeUm), MM::Float, true);

   // Max Speed
   // pAct = new CPropertyAction (this, &MyShapeokoTinyg::OnMaxVelocity);
   // CreateProperty(g_MaxVelocityProp, "100.0", MM::Float, false, pAct);
   //SetPropertyLimits(g_MaxVelocityProp, 0.0, 31999.0);

   // Acceleration
   // pAct = new CPropertyAction (this, &MyShapeokoTinyg::OnAcceleration);
   // CreateProperty(g_AccelProp, "100.0", MM::Float, false, pAct);
   //SetPropertyLimits("Acceleration", 0.0, 150);

   // Move timeout
   // pAct = new CPropertyAction (this, &MyShapeokoTinyg::OnMoveTimeout);
   // CreateProperty(g_MoveTimeoutProp, "10000.0", MM::Float, false, pAct);
   //SetPropertyLimits("Acceleration", 0.0, 150);

   // // Sync Step
   // pAct = new CPropertyAction (this, &MyShapeokoTinyg::OnSyncStep);
   // CreateProperty(g_SyncStepProp, "1.0", MM::Float, false, pAct);


   ret = UpdateStatus();
   if (ret != DEVICE_OK)
      return ret;

   /* From EVA_NDE_Grbl */

   MMThreadGuard myLock(lock_);

   // pAct = new CPropertyAction(this, &MyShapeokoTinyg::OnCommand);
   // ret = CreateProperty("Command","", MM::String, false, pAct);
   // if (DEVICE_OK != ret)
   //    return ret;

   // // turn off verbose serial debug messages
   GetCoreCallback()->SetDeviceProperty(port_.c_str(), "Verbose", "1");
   // synchronize all properties
   // --------------------------

   pAct = new CPropertyAction(this, &ShapeokoGrblHub::OnVersion);
   std::ostringstream sversion;
   sversion << version_;
   CreateProperty(g_versionProp, sversion.str().c_str(), MM::String, true, pAct);

   CDeviceUtils::SleepMs(1000);
   // At port opening, the arduino will sleep for 1-2 seconds then output a version string.
   ret = GetControllerVersion(version_);
   if( DEVICE_OK != ret)
      return ret;
   PurgeComPortH();

   
   LogMessage("Unlock device.");
   ret = SendCommand("$X");
   if(DEVICE_OK != ret){
     return DEVICE_ERR;
   }
   std::string returnString;
   ret = ReceiveResponse(returnString, 1000);
   if(DEVICE_OK != ret){
     return DEVICE_ERR;
   }
   LogMessage("Unlocked device.");
   PurgeComPortH();
   
   LogMessage("resetting device origin.");
   ret = SendCommand("G92 X0 Y0 Z0");
   if(DEVICE_OK != ret){
     return DEVICE_ERR;
   }
   ret = ReceiveResponse(returnString);
   if(DEVICE_OK != ret){
     return DEVICE_ERR;
   }

   LogMessage("reset device origin.");
   CDeviceUtils::SleepMs(500);
   PurgeComPortH();

   ret = GetStatus();
   if (ret != DEVICE_OK)
      return ret;
   PurgeComPortH();

   ret = UpdateStatus();
   if (ret != DEVICE_OK)
      return ret;

   initialized_ = true;
   return DEVICE_OK;
}

// private and expects caller to:
// 1. guard the port
// 2. purge the port
int ShapeokoGrblHub::GetControllerVersion(string& version)
{
  LogMessage("GetControllerVersion");
   int ret = DEVICE_OK;
   MMThreadGuard(this->executeLock_);
   // Ignore initial empty string
   std::string returnString;
   ret = ReceiveResponse(returnString, 1000);
   if(DEVICE_OK != ret){
     return DEVICE_ERR;
   }
   if (returnString != "") {
     LogMessage("Expected empty string, got ");
     LogMessage(returnString);
     return DEVICE_ERR;
   }
   ret = ReceiveResponse(returnString, 1000);
   if(DEVICE_OK != ret){
     return DEVICE_ERR;
   }
     LogMessage("got: ");
     LogMessage(returnString);

   std::vector<std::string> tokenInput;
   char* pEnd;
   CDeviceUtils::Tokenize(returnString, tokenInput, "[");
   
   if(tokenInput.size() != 2) {
     LogMessage("tokensize:");
     LogMessage(std::to_string(tokenInput.size()).c_str());
     return DEVICE_ERR;
   }

   // Depending on lock state, an optional message will be output on how to unlock.
   CDeviceUtils::SleepMs(1000);
   PurgeComPortH();

   version_ = tokenInput[0];
   return ret;
}

int ShapeokoGrblHub::DetectInstalledDevices()
{
  LogMessage("DetectInstalledDevices");
   ClearInstalledDevices();

   // make sure this method is called before we look for available devices
   InitializeModuleData();

   char hubName[MM::MaxStrLength];
   GetName(hubName); // this device name
   for (unsigned i=0; i<GetNumberOfDevices(); i++)
   {
      char deviceName[MM::MaxStrLength];
      LogMessage("Get device");
      bool success = GetDeviceName(i, deviceName, MM::MaxStrLength);
      if (success && (strcmp(hubName, deviceName) != 0))
      {
        LogMessage("Got device", deviceName);
         MM::Device* pDev = CreateDevice(deviceName);
         AddInstalledDevice(pDev);
      }
   }
   return DEVICE_OK;
}

void ShapeokoGrblHub::GetName(char* pName) const
{
   CDeviceUtils::CopyLimitedString(pName, g_HubDeviceName);
}

int ShapeokoGrblHub::OnVersion(MM::PropertyBase* pProp, MM::ActionType pAct)
{
   if (pAct == MM::BeforeGet)
   {
     pProp->Set(version_.c_str());
   }
   return DEVICE_OK;
}
int ShapeokoGrblHub::OnPort(MM::PropertyBase* pProp, MM::ActionType pAct)
{
  LogMessage("OnPort");
   if (pAct == MM::BeforeGet)
   {
      pProp->Set(port_.c_str());
   }
   else if (pAct == MM::AfterSet)
   {
      pProp->Get(port_);
      portAvailable_ = true;
   }
   return DEVICE_OK;
}

int ShapeokoGrblHub::OnCommand(MM::PropertyBase* pProp, MM::ActionType pAct)
{
   PurgeComPortH();
  LogMessage("OnCommand");
   if (pAct == MM::BeforeGet)
   {
	   pProp->Set(commandResult_.c_str());
   }
   else if (pAct == MM::AfterSet)
   {
	   std::string cmd;
      pProp->Get(cmd);
	  if(cmd.compare(commandResult_) ==0)  // command result still there
		  return DEVICE_OK;
	  int ret = SendCommand(cmd);
	  if(DEVICE_OK != ret){
		  return DEVICE_ERR;
	  }
	  ret = ReceiveResponse(commandResult_);
	  if(DEVICE_OK != ret){
		  return DEVICE_ERR;
	  }
   }
   return DEVICE_OK;
}

int ShapeokoGrblHub::SendCommand(std::string command, std::string terminator)
{
  LogMessage("SendCommand");
  LogMessage("command=" + command);
   if(!portAvailable_)
	   return ERR_NO_PORT_SET;
   // needs a lock because the other Thread will also use this function
   MMThreadGuard(this->executeLock_);
   int ret = DEVICE_OK;

   LogMessage("Write command.");
   ret = SetCommandComPortH(command.c_str(), terminator.c_str());
   LogMessage("set command, ret=" + ret);
   if (ret != DEVICE_OK)
   {
	    LogMessage("command write fail");
	   return ret;
   }
}
int ShapeokoGrblHub::ReceiveResponse(std::string &returnString, float timeout)
{
  MMThreadGuard(this->executeLock_);
  SetAnswerTimeoutMs(timeout);

  std::string an;
  try
    {

      int ret = GetSerialAnswerComPortH(an,"\r\n");
      if (ret != DEVICE_OK)
	{
	  LogMessage(std::string("answer get error!_"));
	  return ret;
	}
      LogMessage("answer:");
      LogMessage(std::string(an));
      returnString.assign(an);
      return DEVICE_OK;
    }
  catch(...)
    {
      LogMessage("Exception in send command!");
      return DEVICE_ERR;
    }

}

MM::DeviceDetectionStatus ShapeokoGrblHub::DetectDevice(void)
{
  LogMessage("DetectDevice");
  if (initialized_)
      return MM::CanCommunicate;

   // all conditions must be satisfied...
   MM::DeviceDetectionStatus result = MM::Misconfigured;
   char answerTO[MM::MaxStrLength];

   try
   {
      std::string portLowerCase = port_;
      for( std::string::iterator its = portLowerCase.begin(); its != portLowerCase.end(); ++its)
      {
         *its = (char)tolower(*its);
      }
      if( 0< portLowerCase.length() &&  0 != portLowerCase.compare("undefined")  && 0 != portLowerCase.compare("unknown") )
      {
         result = MM::CanNotCommunicate;
         // record the default answer time out
         GetCoreCallback()->GetDeviceProperty(port_.c_str(), "AnswerTimeout", answerTO);

         // device specific default communication parameters
         // for Arduino Duemilanova
         GetCoreCallback()->SetDeviceProperty(port_.c_str(), MM::g_Keyword_Handshaking, "Off");
         GetCoreCallback()->SetDeviceProperty(port_.c_str(), MM::g_Keyword_BaudRate, "9600");
         GetCoreCallback()->SetDeviceProperty(port_.c_str(), MM::g_Keyword_StopBits, "1");
         // Arduino timed out in GetControllerVersion even if AnswerTimeout  = 300 ms
         GetCoreCallback()->SetDeviceProperty(port_.c_str(), "AnswerTimeout", "500.0");
         GetCoreCallback()->SetDeviceProperty(port_.c_str(), "DelayBetweenCharsMs", "0");
         MM::Device* pS = GetCoreCallback()->GetDevice(this, port_.c_str());
         pS->Initialize();
         // The first second or so after opening the serial port, the Arduino is waiting for firmwareupgrades.  Simply sleep 1 second.
         CDeviceUtils::SleepMs(2000);
         MMThreadGuard myLock(executeLock_);
         PurgeComPort(port_.c_str());
         int ret = GetStatus();
         // later, Initialize will explicitly check the version #
         if( DEVICE_OK != ret )
         {
           LogMessage("Got:");
            LogMessageCode(ret,true);
         }
         else
         {
            // to succeed must reach here....
            result = MM::CanCommunicate;
         }
         pS->Shutdown();
         // always restore the AnswerTimeout to the default
         GetCoreCallback()->SetDeviceProperty(port_.c_str(), "AnswerTimeout", answerTO);

      }
   }
   catch(...)
   {
      LogMessage("Exception in DetectDevice!",false);
   }

   return result;
}

int ShapeokoGrblHub::SetAnswerTimeoutMs(double timeout)
{
      if(!portAvailable_)
	   return ERR_NO_PORT_SET;
     GetCoreCallback()->SetDeviceProperty(port_.c_str(), "AnswerTimeout",  CDeviceUtils::ConvertToString(timeout));
   return DEVICE_OK;
}

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}


std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

template <class Type>
Type stringToNum(const std::string& str)
{
	std::istringstream iss(str);
	Type num;
	iss >> num;
	return num;
}

std::string ShapeokoGrblHub::GetState() {
  return state_;
}

void ShapeokoGrblHub::GetPos(float &x, float &y) {
  x = MPos[0];
  y = MPos[1];
}

// private and expects caller to:
// 1. guard the port
// 2. purge the port
int ShapeokoGrblHub::GetStatus()
{
  LogMessage("GetStatus");
  int ret = SendCommand("?", "");
  if(DEVICE_OK != ret){
    return DEVICE_ERR;
  }
  std::string returnString;
  ret = ReceiveResponse(returnString);
  if(DEVICE_OK != ret){
    return DEVICE_ERR;
  }
  
  LogMessage("returnString=" + returnString);
  std::vector<std::string> tokenInput;
  char* pEnd;
  CDeviceUtils::Tokenize(returnString, tokenInput, "<>,:\r\n");
  //sample: <Idle,MPos:0.000,0.000,0.000,WPos:0.000,0.000,0.000>
  if(tokenInput.size() != 9)
    {
      LogMessage(returnString.c_str());
      LogMessage("echo error!");
      return DEVICE_ERR;
    }
  state_.assign(tokenInput[0].c_str());
  MPos[0] = stringToNum<double>(tokenInput[2]);
  MPos[1] = stringToNum<double>(tokenInput[3]);
  MPos[2] = stringToNum<double>(tokenInput[4]);
  WPos[0] = stringToNum<double>(tokenInput[6]);
  WPos[1] = stringToNum<double>(tokenInput[7]);
  WPos[2] = stringToNum<double>(tokenInput[8]);
   
  return DEVICE_OK;
}
