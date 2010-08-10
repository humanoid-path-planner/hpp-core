/*
 *  Copyright (c) 2010 LAAS-CNRS
 *
 *  Author: Florent Lamiraux
 */

#include <KineoModel/kppSolidComponentRef.h>
#include <hppModel/hppHumanoidRobot.h>
#include <hppModel/hppBody.h>
#include <hppModel/hppJoint.h>
#include "humanoid-robot.hh"
#include "joint.hh"
#include "freeflyer-joint.hh"
#include "rotation-joint.hh"
#include "translation-joint.hh"

using hpp::core::io::HumanoidRobot;
using hpp::core::io::HumanoidRobotShPtr;
using hpp::core::io::FreeflyerJoint;
using hpp::core::io::FreeflyerJointShPtr;
using hpp::core::io::RotationJoint;
using hpp::core::io::RotationJointShPtr;
using hpp::core::io::TranslationJoint;
using hpp::core::io::TranslationJointShPtr;

ChppHumanoidRobotShPtr HumanoidRobot::createHppHumanoidRobot() const
{
  ChppHumanoidRobotShPtr robot = ChppHumanoidRobot::create(name());
  try {
    ChppJoint* rootJoint = buildKinematicChain(robot, rootJointComponent());
    robot->setRootJoint(rootJoint);
    robot->initialize();
  }
  catch (const std::exception& exc) {
    std::cerr <<
      "Exception in hpp::core::io::HumanoidRobot::createHppHumanoidRobot():"
	      << exc.what() << std::endl;
    robot.reset();
  }
  return robot;
}

ChppJoint*
HumanoidRobot::buildKinematicChain(ChppHumanoidRobotShPtr inRobot,
				   const CkppJointComponentShPtr& inJoint)
  const
{
  std::string message;
  JointShPtr kxmlJoint;
  ChppJoint* hppJoint = NULL;

  // Create ChppJoint depending on input joint type.
  if (FreeflyerJointShPtr joint =
      KIT_DYNAMIC_PTR_CAST(FreeflyerJoint, inJoint)) {
    hppJoint = inRobot->createFreeFlyer(inJoint->name(),
				       inJoint->kwsJoint()->initialPosition());
    kxmlJoint = joint;
  } else if (RotationJointShPtr joint =
	     KIT_DYNAMIC_PTR_CAST(RotationJoint, inJoint)) {
    hppJoint = inRobot->createRotation(inJoint->name(),
				      inJoint->kwsJoint()->initialPosition());
    kxmlJoint = joint;
  } else if (TranslationJointShPtr joint =
	     KIT_DYNAMIC_PTR_CAST(TranslationJoint, inJoint)) {
    hppJoint =
      inRobot->createTranslation(inJoint->name(),
				 inJoint->kwsJoint()->initialPosition());
    kxmlJoint = joint;
  } else {
    message = "Joint " + inJoint->name() +
      " is neither of type FreeflyerJoint, RotationJoint nor TranslationJoint.";
    throw exception(message);
  }
  // create ChppBody
  ChppBodyShPtr hppBody = ChppBody::create(std::string(inJoint->name()));
  matrix3d inertiaMatrix
    (kxmlJoint->inertiaMatrixXX->value(),
     kxmlJoint->inertiaMatrixXY->value(),
     kxmlJoint->inertiaMatrixXZ->value(),
     kxmlJoint->inertiaMatrixXY->value(),
     kxmlJoint->inertiaMatrixYY->value(),
     kxmlJoint->inertiaMatrixYZ->value(),
     kxmlJoint->inertiaMatrixXZ->value(),
     kxmlJoint->inertiaMatrixYZ->value(),
     kxmlJoint->inertiaMatrixZZ->value());
  
  MAL_S3_VECTOR(hppBodyRelCom, double);
  MAL_S3_VECTOR_ACCESS(hppBodyRelCom, 0) = kxmlJoint->comX->value();
  MAL_S3_VECTOR_ACCESS(hppBodyRelCom, 1) = kxmlJoint->comY->value();
  MAL_S3_VECTOR_ACCESS(hppBodyRelCom, 2) = kxmlJoint->comZ->value();

  // Set inertia parameters
  hppBody->mass(kxmlJoint->mass->value());
  hppBody->inertiaMatrix(inertiaMatrix);
  hppBody->localCenterOfMass(hppBodyRelCom);

  // Attach body to joint
  hppJoint->setAttachedBody(hppBody);
  
  // Detach geometry from this device to insert it as a child component of
  // this joint.
  std::vector< CkppSolidComponentRefShPtr > solidCompVector;
  inJoint->getSolidComponentRefVector (solidCompVector);
  for (unsigned int iSolid=0; iSolid < solidCompVector.size(); iSolid++) {
    CkppSolidComponentRefShPtr solidCompRef = solidCompVector[iSolid];
    if (inJoint->removeSolidComponentRef(solidCompRef) != KD_OK) {
      message = "CkppSolidComponentRef " +  solidCompRef->name() +
	" is not a child of " + inJoint->name() + ".";
      throw exception(message);
    }
    CkitMat4 position;
    solidCompRef->referencedSolidComponent()->getAbsolutePosition(position);
    if (!hppBody->addInnerObject(solidCompRef, position)) {
      message = "Cannot add inner object " +  solidCompRef->name() +
	" to joint " + inJoint->name() + ".";
      throw exception(message);
    }      
  }
  for (unsigned int iJoint = 0;
       iJoint < inJoint->countChildJointComponents();
       iJoint++) {
    ChppJoint* childJoint = 
      buildKinematicChain(inRobot, inJoint->childJointComponent(iJoint));
    if (!hppJoint->addChildJoint(childJoint)) {
      message = "Cannot add joint " + childJoint->kppJoint()->name()
	+ " as child of " + hppJoint->kppJoint()->name() + ".";
      throw exception(message);
    }
  }
  return hppJoint;
}

