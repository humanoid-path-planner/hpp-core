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
	  const
	{
	  CkppDeviceComponent::fillPropertyVector(inOutPropertyVector);
	}

	/// Create an instance of ChppHumanoidRobot from this object.
	ChppHumanoidRobotShPtr createHppHumanoidRobot() const;

      protected:
	HumanoidRobot() : CkppDeviceComponent() {}
	ktStatus init (const HumanoidRobotWkPtr &inWeakPtr,
		       const std::string &inName)
	{
	  ktStatus status = KD_OK;
	  status = CkppDeviceComponent::init(inWeakPtr, inName);
	  return status;
	}
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
