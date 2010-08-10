/*
 *  Copyright (c) 2010 LAAS-CNRS
 *
 *  Author: Florent Lamiraux
 */

#ifndef HPP_CORE_HUMANOID_ROBOT_HH
#define HPP_CORE_HUMANOID_ROBOT_HH

#include <KineoModel/kppDeviceComponent.h>

KIT_PREDEF_CLASS(ChppHumanoidRobot);
class ChppJoint;

namespace hpp {
  namespace core {
    namespace io {
      KIT_PREDEF_CLASS(HumanoidRobot);

      class exception : public std::exception{
      public:
	exception (const std::string& inMessage) throw() : std::exception()
	{
	  message_ = inMessage;
	}
	exception (const exception&) throw() : std::exception() {};
	virtual ~exception() throw() {};
	virtual const char* what() const throw()
	{
	  return message_.c_str();
	}
      private:
	std::string message_;
      };

      ///
      /// \brief Intermediate humanoid robot.
      ///
      /// This class implements a humanoid robot deriving from
      /// CkppDeviceComponent only for kxml input output purposes.
      ///
      /// Once read from a file the device containing this joint will be
      /// transformed into a ChppHumanoidRobot.

      class HumanoidRobot : public CkppDeviceComponent
      {
      public:
	virtual bool isComponentClonable () const
	{
	  return false;
	}
	static HumanoidRobotShPtr create(const std::string& inName)
	{
	  HumanoidRobot *ptr = new HumanoidRobot();
	  HumanoidRobotShPtr shPtr = HumanoidRobotShPtr(ptr);
	  HumanoidRobotWkPtr wkPtr = HumanoidRobotWkPtr(shPtr);

	  if (ptr->init(wkPtr, inName) != KD_OK) {
	    shPtr.reset();
	    return shPtr;
	  }
	  return shPtr;
	}
	void
	fillPropertyVector(std::vector<CkppPropertyShPtr>& inOutPropertyVector)
	  const;

	/// Create an instance of ChppHumanoidRobot from this object.
	ChppHumanoidRobotShPtr createHppHumanoidRobot() const;

	// Properties
	static const CkppProperty::TPropertyID LEFTANKLE_ID;
	static const std::string LEFTANKLE_STRING_ID;
	static const CkppProperty::TPropertyID RIGHTANKLE_ID;
	static const std::string RIGHTANKLE_STRING_ID;
	static const CkppProperty::TPropertyID LEFTWRIST_ID;
	static const std::string LEFTWRIST_STRING_ID;
	static const CkppProperty::TPropertyID RIGHTWRIST_ID;
	static const std::string RIGHTWRIST_STRING_ID;
	static const CkppProperty::TPropertyID WAIST_ID;
	static const std::string WAIST_STRING_ID;
	static const CkppProperty::TPropertyID CHEST_ID;
	static const std::string CHEST_STRING_ID;
	static const CkppProperty::TPropertyID ANKLEPOSINLEFTFOOTFRAMEX_ID;
	static const std::string ANKLEPOSINLEFTFOOTFRAMEX_STRING_ID;
	static const CkppProperty::TPropertyID ANKLEPOSINLEFTFOOTFRAMEY_ID;
	static const std::string ANKLEPOSINLEFTFOOTFRAMEY_STRING_ID;
	static const CkppProperty::TPropertyID ANKLEPOSINLEFTFOOTFRAMEZ_ID;
	static const std::string ANKLEPOSINLEFTFOOTFRAMEZ_STRING_ID;
	static const CkppProperty::TPropertyID SOLECENTERINLEFTFOOTFRAMEX_ID;
	static const std::string SOLECENTERINLEFTFOOTFRAMEX_STRING_ID;
	static const CkppProperty::TPropertyID SOLECENTERINLEFTFOOTFRAMEY_ID;
	static const std::string SOLECENTERINLEFTFOOTFRAMEY_STRING_ID;
	static const CkppProperty::TPropertyID SOLECENTERINLEFTFOOTFRAMEZ_ID;
	static const std::string SOLECENTERINLEFTFOOTFRAMEZ_STRING_ID;
	static const CkppProperty::TPropertyID SOLELENGTH_ID;
	static const std::string SOLELENGTH_STRING_ID;
	static const CkppProperty::TPropertyID SOLEWIDTH_ID;
	static const std::string SOLEWIDTH_STRING_ID;
	static const CkppProperty::TPropertyID LEFTHANDCENTERX_ID;
	static const std::string LEFTHANDCENTERX_STRING_ID;
	static const CkppProperty::TPropertyID LEFTHANDCENTERY_ID;
	static const std::string LEFTHANDCENTERY_STRING_ID;
	static const CkppProperty::TPropertyID LEFTHANDCENTERZ_ID;
	static const std::string LEFTHANDCENTERZ_STRING_ID;
	static const CkppProperty::TPropertyID LEFTTHUMBAXISX_ID;
	static const std::string LEFTTHUMBAXISX_STRING_ID;
	static const CkppProperty::TPropertyID LEFTTHUMBAXISY_ID;
	static const std::string LEFTTHUMBAXISY_STRING_ID;
	static const CkppProperty::TPropertyID LEFTTHUMBAXISZ_ID;
	static const std::string LEFTTHUMBAXISZ_STRING_ID;
	static const CkppProperty::TPropertyID LEFTFOREFINGERAXISX_ID;
	static const std::string LEFTFOREFINGERAXISX_STRING_ID;
	static const CkppProperty::TPropertyID LEFTFOREFINGERAXISY_ID;
	static const std::string LEFTFOREFINGERAXISY_STRING_ID;
	static const CkppProperty::TPropertyID LEFTFOREFINGERAXISZ_ID;
	static const std::string LEFTFOREFINGERAXISZ_STRING_ID;
	static const CkppProperty::TPropertyID LEFTPALMNORMALX_ID;
	static const std::string LEFTPALMNORMALX_STRING_ID;
	static const CkppProperty::TPropertyID LEFTPALMNORMALY_ID;
	static const std::string LEFTPALMNORMALY_STRING_ID;
	static const CkppProperty::TPropertyID LEFTPALMNORMALZ_ID;
	static const std::string LEFTPALMNORMALZ_STRING_ID;

	/// Left ankle joint name
	CkppStringPropertyShPtr leftAnkle;
	/// Right ankle joint name
	CkppStringPropertyShPtr rightAnkle;
	/// Left wrist joit name
	CkppStringPropertyShPtr leftWrist;
	/// Right wrist joit name
	CkppStringPropertyShPtr rightWrist;
	/// waist joint name
	CkppStringPropertyShPtr waist;
	/// chest joint name
	CkppStringPropertyShPtr chest;
	/// Ankle position in left foot frame
	CkppDoublePropertyShPtr anklePosInLeftFootFrameX;
	CkppDoublePropertyShPtr anklePosInLeftFootFrameY;
	CkppDoublePropertyShPtr anklePosInLeftFootFrameZ;
	/// Sole center in local in left foot local frame
	CkppDoublePropertyShPtr soleCenterInLeftFootFrameX;
	CkppDoublePropertyShPtr soleCenterInLeftFootFrameY;
	CkppDoublePropertyShPtr soleCenterInLeftFootFrameZ;
	/// Sole size
	CkppDoublePropertyShPtr soleLength;
	CkppDoublePropertyShPtr soleWidth;
	// Left hand center
	CkppDoublePropertyShPtr leftHandCenterX;
	CkppDoublePropertyShPtr leftHandCenterY;
	CkppDoublePropertyShPtr leftHandCenterZ;
	// left hand thumb axis
	CkppDoublePropertyShPtr leftThumbAxisX;
	CkppDoublePropertyShPtr leftThumbAxisY;
	CkppDoublePropertyShPtr leftThumbAxisZ;
	// Left fore-finger axis
	CkppDoublePropertyShPtr leftForeFingerAxisX;
	CkppDoublePropertyShPtr leftForeFingerAxisY;
	CkppDoublePropertyShPtr leftForeFingerAxisZ;
	// Left palm normal
	CkppDoublePropertyShPtr leftPalmNormalX;
	CkppDoublePropertyShPtr leftPalmNormalY;
	CkppDoublePropertyShPtr leftPalmNormalZ;

      protected:
	HumanoidRobot() : CkppDeviceComponent() {}
	ktStatus init (const HumanoidRobotWkPtr &inWeakPtr,
		       const std::string &inName);
      private:
	/// Recursively builds the kinematic chain of the ChppHumanoidRobot
	ChppJoint*
	buildKinematicChain(ChppHumanoidRobotShPtr inRobot,
			    const CkppJointComponentShPtr& inJoint) const;
      };
    } // namespace io
  } // namespace core
} // namespace hpp
#endif //HPP_CORE_HUMANOID_ROBOT_HH
