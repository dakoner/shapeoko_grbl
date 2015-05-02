#include "ShapeokoGrbl.h"
#include "XYStage.h"

///////////////////////////////////////////////////////////////////////////////
// CShapeokoGrblXYStage implementation
// ~~~~~~~~~~~~~~~~~~~~~~~~~

CShapeokoGrblXYStage::CShapeokoGrblXYStage() :
CXYStageBase<CShapeokoGrblXYStage>(),
stepSize_um_(0.025),
posX_um_(0.0),
posY_um_(0.0),
busy_(false),
velocity_(10.0), // in micron per second
initialized_(false),
lowerLimit_(0.0),
upperLimit_(20000.0)
{
   InitializeDefaultErrorMessages();

   // parent ID display
   CreateHubIDProperty();
}

CShapeokoGrblXYStage::~CShapeokoGrblXYStage()
{
   Shutdown();
}

extern const char* g_XYStageDeviceName;
const char* NoHubError = "Parent Hub not defined.";



void CShapeokoGrblXYStage::GetName(char* Name) const
{
  LogMessage("XYStage: GetName");
   CDeviceUtils::CopyLimitedString(Name, g_XYStageDeviceName);
}

int CShapeokoGrblXYStage::Initialize()
{
  LogMessage("XYStage: initialize");
   ShapeokoGrblHub* pHub = static_cast<ShapeokoGrblHub*>(GetParentHub());
   if (pHub)
   {
      char hubLabel[MM::MaxStrLength];
      pHub->GetLabel(hubLabel);
      SetParentID(hubLabel); // for backward comp.
   }
   else
      LogMessage(NoHubError);

   if (initialized_)
      return DEVICE_OK;

   // set property list
   // -----------------

   // Name
   int ret = CreateStringProperty(MM::g_Keyword_Name, g_XYStageDeviceName, true);
   if (DEVICE_OK != ret)
      return ret;

   // Description
   ret = CreateStringProperty(MM::g_Keyword_Description, "ShapeokoGrbl XY stage driver", true);
   if (DEVICE_OK != ret)
      return ret;

   ret = UpdateStatus();
   if (ret != DEVICE_OK)
      return ret;

   initialized_ = true;

   return DEVICE_OK;
}

int CShapeokoGrblXYStage::Shutdown()
{
   if (initialized_)
   {
      initialized_ = false;
   }
   return DEVICE_OK;
}

bool CShapeokoGrblXYStage::Busy()
{
  ShapeokoGrblHub* pHub = static_cast<ShapeokoGrblHub*>(GetParentHub());
  int ret = pHub->GetStatus();
  if (ret != DEVICE_OK) {
    LogMessage("Got error from machien when calling busy.");
    return ret;
  }
  std::string state = pHub->GetState();
  if (state.compare(0, 5, "Idle") == 0) {
    LogMessage("I am idle");
    return false;
  }
  LogMessage("I am busy");
  return true;
}

int CShapeokoGrblXYStage::SetPositionSteps(long x, long y)
{
  LogMessage("XYStage: SetPositionSteps");
  if (Busy())
    return ERR_STAGE_MOVING;
  double newPosX = x * stepSize_um_;
  double newPosY = y * stepSize_um_;
  double difX = newPosX - posX_um_;
  double difY = newPosY - posY_um_;
  double distance = sqrt( (difX * difX) + (difY * difY) );
  posX_um_ = x * stepSize_um_;
  posY_um_ = y * stepSize_um_;

  char buff[100];
  sprintf(buff, "G0 X%f Y%f", posX_um_/1000., posY_um_/1000.);
  std::string buffAsStdStr = buff;
  ShapeokoGrblHub* pHub = static_cast<ShapeokoGrblHub*>(GetParentHub());
  int ret = pHub->SendCommand(buffAsStdStr);
  if (ret != DEVICE_OK)
    return ret;
  ret = pHub->ReceiveResponse(buffAsStdStr);
  if (ret != DEVICE_OK)
    return ret;

  ret = OnXYStagePositionChanged(posX_um_, posY_um_);
  if (ret != DEVICE_OK)
    return ret;
  
  return DEVICE_OK;
}

int CShapeokoGrblXYStage::GetPositionSteps(long& x, long& y)
{
  ShapeokoGrblHub* pHub = static_cast<ShapeokoGrblHub*>(GetParentHub());
  int ret = pHub->GetStatus();
  if (ret != DEVICE_OK) {
    LogMessage("Got error from machien when calling busy.");
    return ret;
  }
  float tx, ty;
  pHub->GetPos(tx, ty);
  tx *= 1000.;
  ty *= 1000.;
  LogMessage("XYStage: GetPositionSteps");
  LogMessage(std::to_string(tx));
  LogMessage(std::to_string(ty));
  x = (long)(tx / stepSize_um_);
  y = (long)(ty / stepSize_um_);
  LogMessage(std::to_string(x));
  LogMessage(std::to_string(y));
  return DEVICE_OK;
}

int CShapeokoGrblXYStage::SetRelativePositionSteps(long x, long y)
{
  LogMessage("XYStage: SetRelativePositioNSteps");
   long xSteps, ySteps;
   GetPositionSteps(xSteps, ySteps);

   return this->SetPositionSteps(xSteps+x, ySteps+y);
}


///////////////////////////////////////////////////////////////////////////////
// Action handlers
///////////////////////////////////////////////////////////////////////////////
// none implemented
