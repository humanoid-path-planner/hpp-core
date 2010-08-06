#ifndef HPP_CORE_FREEFLYER_JOINT_HH
#define HPP_CORE_FREEFLYER_JOINT_HH

#include <KineoModel/kppFreeFlyerJointComponent.h>

#include "hpp/core/joint-properties.hh"

namespace hpp {
  namespace core {
    KIT_PREDEF_CLASS(FreeflyerJoint);
    class FreeflyerJoint : public CkppFreeFlyerJointComponent,
			   public hpp::core::JointProperties
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
      fillPropertyVector(std::vector<CkppPropertyShPtr>& outPropertyVector)
	const
      {
	CkppFreeFlyerJointComponent::fillPropertyVector(outPropertyVector);
	outPropertyVector.push_back(mass);
	outPropertyVector.push_back(comX);
	outPropertyVector.push_back(comY);
	outPropertyVector.push_back(comZ);
	outPropertyVector.push_back(inertiaMatrixXX);
	outPropertyVector.push_back(inertiaMatrixYY);
	outPropertyVector.push_back(inertiaMatrixZZ);
	outPropertyVector.push_back(inertiaMatrixXY);
	outPropertyVector.push_back(inertiaMatrixXZ);
	outPropertyVector.push_back(inertiaMatrixYZ);
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
  } // namespace core
} // namespace hpp
#endif //HPP_CORE_FREEFLYER_JOINT_HH
