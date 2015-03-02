#include "ShapeokoGrbl.h"
#include "XYStage.h"

///////////////////////////////////////////////////////////////////////////////
// CShapeokoGrblXYStage implementation
// ~~~~~~~~~~~~~~~~~~~~~~~~~

CShapeokoGrblXYStage::CShapeokoGrblXYStage() :
CXYStageBase<CShapeokoGrblXYStage>(),
stepSize_um_(0.015),
posX_um_(0.0),
posY_um_(0.0),
busy_(false),
timeOutTimer_(0),
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
   CDeviceUtils::CopyLimitedString(Name, g_XYStageDeviceName);
}

int CShapeokoGrblXYStage::Initialize()
{
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
   if (timeOutTimer_ == 0)
      return false;
   if (timeOutTimer_->expired(GetCurrentMMTime()))
   {
      // delete(timeOutTimer_);
      return false;
   }
   return true;
}

int CShapeokoGrblXYStage::SetPositionSteps(long x, long y)
{
   if (timeOutTimer_ != 0)
   {
      if (!timeOutTimer_->expired(GetCurrentMMTime()))
         return ERR_STAGE_MOVING;
      delete (timeOutTimer_);
   }
   double newPosX = x * stepSize_um_;
   double newPosY = y * stepSize_um_;
   double difX = newPosX - posX_um_;
   double difY = newPosY - posY_um_;
   double distance = sqrt( (difX * difX) + (difY * difY) );
   long timeOut = (long) (distance / velocity_);
   timeOutTimer_ = new MM::TimeoutMs(GetCurrentMMTime(),  timeOut);
   posX_um_ = x * stepSize_um_;
   posY_um_ = y * stepSize_um_;
   int ret = OnXYStagePositionChanged(posX_um_, posY_um_);
   if (ret != DEVICE_OK)
      return ret;

   return DEVICE_OK;
}

int CShapeokoGrblXYStage::GetPositionSteps(long& x, long& y)
{
   x = (long)(posX_um_ / stepSize_um_);
   y = (long)(posY_um_ / stepSize_um_);
   return DEVICE_OK;
}

int CShapeokoGrblXYStage::SetRelativePositionSteps(long x, long y)
{
   long xSteps, ySteps;
   GetPositionSteps(xSteps, ySteps);

   return this->SetPositionSteps(xSteps+x, ySteps+y);
}


///////////////////////////////////////////////////////////////////////////////
// Action handlers
///////////////////////////////////////////////////////////////////////////////
// none implemented
