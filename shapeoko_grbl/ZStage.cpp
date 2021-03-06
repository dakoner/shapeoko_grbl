///////////////////////////////////////////////////////////////////////////////
// FILE:       ZStage.cpp
// PROJECT:    MicroManage
// SUBSYSTEM:  DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:
// Zeiss CAN bus controller for Axioscop 2 MOT, Z-stage
//


#ifdef WIN32
   #include <windows.h>
   #define snprintf _snprintf
#endif
#include "ShapeokoGrbl.h"

#include "ZStage.h"
using namespace std;


extern const char* g_ZStageDeviceName;
extern const char* g_Keyword_LoadSample;

ZStage::ZStage() :
// http://www.shapeoko.com/wiki/index.php/Zaxis_ACME
stepSize_um_ (5.),
	posZ_um_(0.0),
initialized_ (false)
{
   InitializeDefaultErrorMessages();

   SetErrorText(ERR_SCOPE_NOT_ACTIVE, "Zeiss Scope is not initialized.  It is needed for the Focus drive to work");
   SetErrorText(ERR_NO_FOCUS_DRIVE, "No focus drive found in this microscopes");
}

ZStage::~ZStage()
{
   Shutdown();
}

void ZStage::GetName(char* Name) const
{
   CDeviceUtils::CopyLimitedString(Name, g_ZStageDeviceName);
}

int ZStage::Initialize()
{
  
	 InitializeDefaultErrorMessages();

   // parent ID display
   CreateHubIDProperty();
   // set property list
   // ----------------

   // Name
   int ret = CreateProperty(MM::g_Keyword_Name, g_ZStageDeviceName, MM::String, true);
   if (DEVICE_OK != ret)
      return ret;

   // Description
   ret = CreateProperty(MM::g_Keyword_Description, "Z-drive", MM::String, true);
   if (DEVICE_OK != ret)
      return ret;

   // Position
   CPropertyAction* pAct = new CPropertyAction(this, &ZStage::OnPosition);
   ret = CreateProperty(MM::g_Keyword_Position, "0", MM::Float,false, pAct);
   if (ret != DEVICE_OK)
      return ret;

   // Update lower and upper limits.  These values are cached, so if they change during a session, the adapter will need to be re-initialized
   ret = UpdateStatus();
   if (ret != DEVICE_OK)
      return ret;

   initialized_ = true;

   return DEVICE_OK;
}

int ZStage::Shutdown()
{
   initialized_ = false;

   return DEVICE_OK;
}

bool ZStage::Busy()
{
   return false;
}

int ZStage::SetPositionUm(double pos)
{
   long steps = (long)(pos / stepSize_um_ + 0.5);
   int ret = SetPositionSteps(steps);
   if (ret != DEVICE_OK)
      return ret;

   return DEVICE_OK;
}

int ZStage::GetPositionUm(double& pos)
{
   long steps;
   int ret = GetPositionSteps(steps);
   if (ret != DEVICE_OK)
      return ret;
   pos = steps * stepSize_um_;

   return DEVICE_OK;
}

/*
 * Requests movement to new z postion from the controller.  This function does the actual communication
 */
int ZStage::SetPositionSteps(long steps)
{
  LogMessage("ZStage: SetPositionSteps");
  //  if (timeOutTimer_ != 0)
  //  {
  // LogMessage("ZStage: 1");
  //     if (!timeOutTimer_->expired(GetCurrentMMTime()))
  //        return ERR_STAGE_MOVING;
  //     delete (timeOutTimer_);
  // LogMessage("ZStage: 2");
  //  }
   posZ_um_ = steps * stepSize_um_;
   

   char buff[100];
   sprintf(buff, "G0 Z%f", posZ_um_/1000.);
   LogMessage("ZStage buff:");
   LogMessage(buff);
   std::string buffAsStdStr = buff;
  LogMessage("ZStage: SetPositionSteps get hub");
   ShapeokoGrblHub* pHub = static_cast<ShapeokoGrblHub*>(GetParentHub());
   LogMessage("ZStage: SetPositionSteps got hub");
   int ret = pHub->SendCommand(buffAsStdStr);
   LogMessage("ZStage: SetPositionSteps sent command");
   if (ret != DEVICE_OK)
      return ret;
   ret = pHub->ReceiveResponse(buffAsStdStr);
   if (ret != DEVICE_OK)
      return ret;

     CDeviceUtils::SleepMs(250);
    ret = pHub->PurgeComPortH();
    bool done;
  while(!done) {
    ret = pHub->PurgeComPortH();
    
    int ret = pHub->SendCommand("?", "");
    if(DEVICE_OK != ret){
      return ret;
    }
    std::string returnString;
    ret = pHub->ReceiveResponse(returnString);
    if(DEVICE_OK != ret){
      return ret;
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
    if (tokenInput[0] == "Idle")
      {
	  LogMessage("got idle");
	  break;
      }
  }


   // ret = OnZStagePositionChanged(posZ_um_);
   
   return DEVICE_OK;
}

/*
 * Requests current z postion from the controller.  This function does the actual communication
 */
int ZStage::GetPositionSteps(long& steps)
{
  LogMessage("ZStage: GetPositionSteps");
   steps = (long)(posZ_um_ / stepSize_um_);
   return DEVICE_OK;
}

int ZStage::SetOrigin()
{
   // const char* cmd ="HPZP0" ;
   // int ret = g_hub.ExecuteCommand(*this, *GetCoreCallback(),  cmd);
   // if (ret != DEVICE_OK)
   //    return ret;

  // TODO(dek): run gcode to set origin to current location (G28.3 ?)

   return DEVICE_OK;
}

// TODO(dek): implement GetUpperLimit and GetLowerLimit

///////////////////////////////////////////////////////////////////////////////
// Action handlers
///////////////////////////////////////////////////////////////////////////////
/*
 * Uses the Get and Set PositionUm functions to communicate with controller
 */
int ZStage::OnPosition(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      double pos;
      int ret = GetPositionUm(pos);
      if (ret != DEVICE_OK)
         return ret;
      pProp->Set(pos);
   }
   else if (eAct == MM::AfterSet)
   {
      double pos;
      pProp->Get(pos);
      int ret = SetPositionUm(pos);
      if (ret != DEVICE_OK)
         return ret;
   }

   return DEVICE_OK;
}


// TODO(dek): implement OnStageLoad
