/*
  Copyright 2010 CNRS-LAAS
  Author: Florent Lamiraux
*/

#ifndef HPP_CORE_JOINT_HH
#define HPP_CORE_JOINT_HH

#include <KineoModel/kppProperty.h>

KIT_PREDEF_CLASS(CkppDoubleProperty);
KIT_PREDEF_CLASS(CkwsJoint);

namespace hpp {
  namespace core {
    namespace io {
      KIT_PREDEF_CLASS(Joint)
      class Joint
      {
      public:
	virtual CkwsJointShPtr kwsJoint() const = 0;
	// Mass
	static const CkppProperty::TPropertyID MASS_ID;
	static const std::string MASS_STRING_ID;

	// Local center of mass
	static const CkppProperty::TPropertyID COM_X_ID;
	static const std::string COM_X_STRING_ID;
	static const CkppProperty::TPropertyID COM_Y_ID;
	static const std::string COM_Y_STRING_ID;
	static const CkppProperty::TPropertyID COM_Z_ID;
	static const std::string COM_Z_STRING_ID;

	// Inertia matrix
	static const CkppProperty::TPropertyID INERTIA_MATRIX_XX_ID;
	static const std::string INERTIA_MATRIX_XX_STRING_ID;
	static const CkppProperty::TPropertyID INERTIA_MATRIX_YY_ID;
	static const std::string INERTIA_MATRIX_YY_STRING_ID;
	static const CkppProperty::TPropertyID INERTIA_MATRIX_ZZ_ID;
	static const std::string INERTIA_MATRIX_ZZ_STRING_ID;
	static const CkppProperty::TPropertyID INERTIA_MATRIX_XY_ID;
	static const std::string INERTIA_MATRIX_XY_STRING_ID;
	static const CkppProperty::TPropertyID INERTIA_MATRIX_XZ_ID;
	static const std::string INERTIA_MATRIX_XZ_STRING_ID;
	static const CkppProperty::TPropertyID INERTIA_MATRIX_YZ_ID;
	static const std::string INERTIA_MATRIX_YZ_STRING_ID;
      
	CkppDoublePropertyShPtr mass;
	CkppDoublePropertyShPtr comX;
	CkppDoublePropertyShPtr comY;
	CkppDoublePropertyShPtr comZ;
	CkppDoublePropertyShPtr inertiaMatrixXX;
	CkppDoublePropertyShPtr inertiaMatrixYY;
	CkppDoublePropertyShPtr inertiaMatrixZZ;
	CkppDoublePropertyShPtr inertiaMatrixXY;
	CkppDoublePropertyShPtr inertiaMatrixXZ;
	CkppDoublePropertyShPtr inertiaMatrixYZ;
      
      protected:
	Joint();
	/// Create properties
	ktStatus init(const CkppComponentWkPtr& inComponent);
      };
    } // namespace io
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_JOINT_HH
