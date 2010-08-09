/*
  Copyright 2010 CNRS-LAAS
  Author: Florent Lamiraux
*/

#include <KineoModel/kppDoubleProperty.h>

#include "joint.hh"

using hpp::core::io::Joint;

// Mass
const CkppProperty::TPropertyID
Joint::MASS_ID(CkppProperty::makeID());
const std::string Joint::MASS_STRING_ID("MASS");
const CkppProperty::TPropertyID

// Center of mass
// X coordinate
Joint::COM_X_ID(CkppProperty::makeID());
const std::string Joint::COM_X_STRING_ID("COM_X");
// Y coordinate
const CkppProperty::TPropertyID
Joint::COM_Y_ID(CkppProperty::makeID());
const std::string Joint::COM_Y_STRING_ID("COM_Y");
// Z coordinate
const CkppProperty::TPropertyID
Joint::COM_Z_ID(CkppProperty::makeID());
const std::string Joint::COM_Z_STRING_ID("COM_Z");

// Inertia matrix
// XX coefficient
const CkppProperty::TPropertyID
Joint::INERTIA_MATRIX_XX_ID(CkppProperty::makeID());
const std::string
Joint::INERTIA_MATRIX_XX_STRING_ID("INERTIA_MATRIX_XX");
// YY coefficient
const CkppProperty::TPropertyID
Joint::INERTIA_MATRIX_YY_ID(CkppProperty::makeID());
const std::string
Joint::INERTIA_MATRIX_YY_STRING_ID("INERTIA_MATRIX_YY");
// ZZ coefficient
const CkppProperty::TPropertyID
Joint::INERTIA_MATRIX_ZZ_ID(CkppProperty::makeID());
const std::string
Joint::INERTIA_MATRIX_ZZ_STRING_ID("INERTIA_MATRIX_ZZ");
// XY coefficient
const CkppProperty::TPropertyID
Joint::INERTIA_MATRIX_XY_ID(CkppProperty::makeID());
const std::string
Joint::INERTIA_MATRIX_XY_STRING_ID("INERTIA_MATRIX_XY");
// XZ coefficient
const CkppProperty::TPropertyID
Joint::INERTIA_MATRIX_XZ_ID(CkppProperty::makeID());
const std::string
Joint::INERTIA_MATRIX_XZ_STRING_ID("INERTIA_MATRIX_XZ");
// YZ coefficient
const CkppProperty::TPropertyID
Joint::INERTIA_MATRIX_YZ_ID(CkppProperty::makeID());
const std::string
Joint::INERTIA_MATRIX_YZ_STRING_ID("INERTIA_MATRIX_YZ");

Joint::Joint()
{
}

ktStatus Joint::init(const CkppComponentWkPtr& inWeakPtr)
{
  CkppComponentShPtr component = inWeakPtr.lock();
  mass = CkppDoubleProperty::create("MASS", component,
  				    MASS_ID , MASS_STRING_ID);
  if (!mass) return KD_ERROR;

  comX = CkppDoubleProperty::create("COM_X", component,
				    COM_X_ID, COM_X_STRING_ID);
  if (!comX) return KD_ERROR;

  comY = CkppDoubleProperty::create("COM_Y", component,
				    COM_Y_ID, COM_Y_STRING_ID);
  if (!comY) return KD_ERROR;

  comZ = CkppDoubleProperty::create("COM_Z", component,
				    COM_Z_ID, COM_Z_STRING_ID);
  if (!comZ) return KD_ERROR;

  inertiaMatrixXX =
    CkppDoubleProperty::create("INERTIA_MATRIX_XX", component,
			       INERTIA_MATRIX_XX_ID,
			       INERTIA_MATRIX_XX_STRING_ID);
  if (!inertiaMatrixXX) return KD_ERROR;

  inertiaMatrixYY =
    CkppDoubleProperty::create("INERTIA_MATRIX_YY", component,
			       INERTIA_MATRIX_YY_ID,
			       INERTIA_MATRIX_YY_STRING_ID);
  if (!inertiaMatrixYY) return KD_ERROR;

  inertiaMatrixZZ =
    CkppDoubleProperty::create("INERTIA_MATRIX_ZZ", component,
			       INERTIA_MATRIX_ZZ_ID,
			       INERTIA_MATRIX_ZZ_STRING_ID);
  if (!inertiaMatrixZZ) return KD_ERROR;

  inertiaMatrixXY =
    CkppDoubleProperty::create("INERTIA_MATRIX_XY", component,
			       INERTIA_MATRIX_XY_ID,
			       INERTIA_MATRIX_XY_STRING_ID);
  if (!inertiaMatrixXY) return KD_ERROR;

  inertiaMatrixXZ =
    CkppDoubleProperty::create("INERTIA_MATRIX_XZ", component,
			       INERTIA_MATRIX_XZ_ID,
			       INERTIA_MATRIX_XZ_STRING_ID);
  if (!inertiaMatrixXZ) return KD_ERROR;

  inertiaMatrixYZ =
    CkppDoubleProperty::create("INERTIA_MATRIX_YZ", component,
			       INERTIA_MATRIX_YZ_ID,
			       INERTIA_MATRIX_YZ_STRING_ID);
  if (!inertiaMatrixYZ) return KD_ERROR;

  return KD_OK;
}
