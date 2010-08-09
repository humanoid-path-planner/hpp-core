#ifndef HPP_CORE_TRANSLATION_JOINT_HH
#define HPP_CORE_TRANSLATION_JOINT_HH

#include <KineoModel/kppTranslationJointComponent.h>

#include "hpp/core/joint-properties.hh"

namespace hpp {
  namespace core {
    KIT_PREDEF_CLASS(TranslationJoint);
    ///
    /// \brief Intermediate translation joint.
    ///
    /// This class implements a translation joint deriving from
    /// CkppTranslationJointComponent only for kxml input output purposes.
    ///
    /// Joints of this class contain inertia data as CkppDoubleProperty
    /// attributes.
    ///
    /// Once read from a file the device containing this joint will be
    /// transformed into a ChppHumanoidRobot.

    class TranslationJoint : public CkppTranslationJointComponent,
			   public JointProperties
    {
    public:
      virtual bool isComponentClonable () const
      {
	return false;
      }
      static TranslationJointShPtr create(const std::string& inName)
      {
	TranslationJoint *ptr = new TranslationJoint();
	TranslationJointShPtr shPtr = TranslationJointShPtr(ptr);
	TranslationJointWkPtr wkPtr = TranslationJointWkPtr(shPtr);

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
	CkppTranslationJointComponent::fillPropertyVector(inOutPropertyVector);
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
      TranslationJoint() : CkppTranslationJointComponent() {}
      ktStatus init (const TranslationJointWkPtr &inWeakPtr,
		     const std::string &inName)
      {
	ktStatus status = KD_OK;
	status = CkppTranslationJointComponent::init(inWeakPtr, inName);
	if (status == KD_ERROR) return KD_ERROR;
	return JointProperties::init(inWeakPtr);
      }
    };
  } // namespace core
} // namespace hpp
#endif //HPP_CORE_TRANSLATION_JOINT_HH
