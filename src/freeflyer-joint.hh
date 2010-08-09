/*
 *  Copyright (c) 2010 LAAS-CNRS
 *
 *  Author: Florent Lamiraux
 */

#ifndef HPP_CORE_FREEFLYER_JOINT_HH
#define HPP_CORE_FREEFLYER_JOINT_HH

#include <KineoModel/kppFreeFlyerJointComponent.h>

#include "joint-properties.hh"

namespace hpp {
  namespace core {
    namespace io {
      KIT_PREDEF_CLASS(FreeflyerJoint);
      ///
      /// \brief Intermediate freeflyer joint.
      ///
      /// This class implements a freeflyer joint deriving from
      /// CkppFreeFlyerJointComponent only for kxml input output purposes.
      ///
      /// Joints of this class contain inertia data as CkppDoubleProperty
      /// attributes.
      ///
      /// Once read from a file the device containing this joint will be
      /// transformed into a ChppHumanoidRobot.

      class FreeflyerJoint : public CkppFreeFlyerJointComponent,
			     public JointProperties
      {
      public:
	virtual bool isComponentClonable () const
	{
	  return false;
	}
	static FreeflyerJointShPtr create(const std::string& inName)
	{
	  FreeflyerJoint *ptr = new FreeflyerJoint();
	  FreeflyerJointShPtr shPtr = FreeflyerJointShPtr(ptr);
	  FreeflyerJointWkPtr wkPtr = FreeflyerJointWkPtr(shPtr);

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
	  CkppFreeFlyerJointComponent::fillPropertyVector(inOutPropertyVector);
	  inOutPropertyVector.push_back(mass);
	  inOutPropertyVector.push_back(comX);
	  inOutPropertyVector.push_back(comY);
	  inOutPropertyVector.push_back(comZ);
	  inOutPropertyVector.push_back(inertiaMatrixXX);
	  inOutPropertyVector.push_back(inertiaMatrixYY);
	  inOutPropertyVector.push_back(inertiaMatrixZZ);
	  inOutPropertyVector.push_back(inertiaMatrixXY);
	  inOutPropertyVector.push_back(inertiaMatrixXZ);
	  inOutPropertyVector.push_back(inertiaMatrixYZ);
	}

      protected:
	FreeflyerJoint() : CkppFreeFlyerJointComponent() {}
	ktStatus init (const FreeflyerJointWkPtr &inWeakPtr,
		       const std::string &inName)
	{
	  ktStatus status = KD_OK;
	  status = CkppFreeFlyerJointComponent::init(inWeakPtr, inName);
	  if (status == KD_ERROR) return KD_ERROR;
	  return JointProperties::init(inWeakPtr);
	}
      };
    } // namespace io
  } // namespace core
} // namespace hpp
#endif //HPP_CORE_FREEFLYER_JOINT_HH
