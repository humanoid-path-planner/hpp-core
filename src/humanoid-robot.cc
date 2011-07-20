/*
 *  Copyright (c) 2010 LAAS-CNRS
 *
 *  Author: Florent Lamiraux
 */

#include <KineoModel/kppSolidComponentRef.h>
#include <hpp/model/humanoid-robot.hh>
#include <hpp/model/body.hh>
#include <hpp/model/joint.hh>
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


ktStatus HumanoidRobot::init (const HumanoidRobotWkPtr &inWeakPtr,
			      const std::string &name)
{
  if (CkppDeviceComponent::init(inWeakPtr, name) != KD_OK) return KD_ERROR;

  return KD_OK;
}

ChppHumanoidRobotShPtr HumanoidRobot::createHppHumanoidRobot()
{
  ChppHumanoidRobotShPtr robot = ChppHumanoidRobot::create(name());
  std::cout << "HumanoidRobot::createHppHumanoidRobot()" << std::endl;
  try {
    ChppJoint* rootJoint = buildKinematicChain(robot, rootJointComponent());
    robot->setRootJoint(rootJoint);
    robot->initialize();
    setHumanoidProperties(robot);
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
HumanoidRobot::buildKinematicChain(ChppHumanoidRobotShPtr robot,
				   const CkppJointComponentShPtr& inJoint)
{
  std::string message;
  JointShPtr kxmlJoint;
  ChppJoint* hppJoint = NULL;

  // Create ChppJoint depending on input joint type.
  if (FreeflyerJointShPtr joint =
      KIT_DYNAMIC_PTR_CAST(FreeflyerJoint, inJoint)) {
    hppJoint = robot->createFreeFlyer(inJoint->name(),
				       inJoint->kwsJoint()->initialPosition());
    kxmlJoint = joint;
  } else if (RotationJointShPtr joint =
	     KIT_DYNAMIC_PTR_CAST(RotationJoint, inJoint)) {
    hppJoint = robot->createRotation(inJoint->name(),
				      inJoint->kwsJoint()->initialPosition());
    kxmlJoint = joint;
  } else if (TranslationJointShPtr joint =
	     KIT_DYNAMIC_PTR_CAST(TranslationJoint, inJoint)) {
    hppJoint =
      robot->createTranslation(inJoint->name(),
				 inJoint->kwsJoint()->initialPosition());
    kxmlJoint = joint;
  } else {
    message = "Joint " + inJoint->name() +
      " is neither of type FreeflyerJoint, RotationJoint nor TranslationJoint.";
    throw exception(message);
  }
  // Set joint bounds
  for (unsigned int iDof = 0; iDof < kxmlJoint->kwsJoint()->countDofs();
       iDof++) {
    bool isBounded = kxmlJoint->kwsJoint()->dof(iDof)->isBounded();
    double vmin = kxmlJoint->kwsJoint()->dof(iDof)->vmin();
    double vmax = kxmlJoint->kwsJoint()->dof(iDof)->vmax();
    
    hppJoint->kppJoint()->kwsJoint()->dof(iDof)->isBounded(isBounded);
    hppJoint->kppJoint()->kwsJoint()->dof(iDof)->vmin(vmin);
    hppJoint->kppJoint()->kwsJoint()->dof(iDof)->vmax(vmax);

    hppJoint->jrlJoint()->lowerBound(iDof, vmin);
    hppJoint->jrlJoint()->upperBound(iDof, vmax);
  }
  // Register joint in map
  jointMap_[inJoint->name()] = hppJoint;

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
      buildKinematicChain(robot, inJoint->childJointComponent(iJoint));
    if (!hppJoint->addChildJoint(childJoint)) {
      message = "Cannot add joint " + childJoint->kppJoint()->name()
	+ " as child of " + hppJoint->kppJoint()->name() + ".";
      throw exception(message);
    }
  }
  return hppJoint;
}

void HumanoidRobot::setHumanoidProperties(const ChppHumanoidRobotShPtr& robot)
{
  CjrlJoint* jrlJoint = NULL;
  std::string name;
  std::string message;

  //gaze
  name = gaze->value();
  if (jointMap_.count(name) != 1) {
    message = "Joint with name " + name + "does not seem to exist.";
    throw exception(message);
  }
  jrlJoint = jointMap_[name]->jrlJoint();
  robot->gazeJoint(jrlJoint);
  vector3d dir,origin;
  origin[0] = gazeOriginX->value();
  origin[1] = gazeOriginY->value();
  origin[2] = gazeOriginZ->value();
  dir[0] = gazeDirectionX->value();
  dir[1] = gazeDirectionY->value();
  dir[2] = gazeDirectionZ->value();
  robot->gaze(dir, origin);

  // left ankle
  name = leftAnkle->value();
  if (jointMap_.count(name) != 1) {
    message = "Joint with name " + name + "does not seem to exist.";
    throw exception(message);
  }
  jrlJoint = jointMap_[name]->jrlJoint();
  robot->leftAnkle(jrlJoint);
  CjrlFoot *foot = objectFactory_.createFoot(jrlJoint);
  foot->setAssociatedAnkle(jrlJoint);
  vector3d anklePosInFootFrame;
  anklePosInFootFrame(0) = anklePosInLeftFootFrameX->value();
  anklePosInFootFrame(1) = anklePosInLeftFootFrameY->value();
  anklePosInFootFrame(2) = anklePosInLeftFootFrameZ->value();
  foot->setAnklePositionInLocalFrame(anklePosInFootFrame);
  foot->setSoleSize(soleLength->value(), soleWidth->value());
  vector3d vzero;
  robot->leftFoot(foot);
  // right ankle
  name = rightAnkle->value();
  if (jointMap_.count(name) != 1) {
    message = "Joint with name " + name + "does not seem to exist.";
    throw exception(message);
  }
  jrlJoint = jointMap_[name]->jrlJoint();
  robot->rightAnkle(jrlJoint);
  foot = objectFactory_.createFoot(jrlJoint);
  foot->setAssociatedAnkle(jrlJoint);
  // Get right foot values by symmetry
  anklePosInFootFrame(1) *= -1;
  robot->rightFoot(foot);
  foot->setAnklePositionInLocalFrame(anklePosInFootFrame);
  foot->setSoleSize(soleLength->value(), soleWidth->value());

  // left wrist
  name = leftWrist->value();
  if (jointMap_.count(name) != 1) {
    message = "Joint with name " + name + "does not seem to exist.";
    throw exception(message);
  }
  jrlJoint = jointMap_[name]->jrlJoint();
  robot->leftWrist(jrlJoint);
  CjrlHand* hand=objectFactory_.createHand(jrlJoint);
  hand->setAssociatedWrist(jrlJoint);
  robot->leftHand(hand);
  vector3d handCenter;
  handCenter(0) = leftHandCenterX->value();
  handCenter(1) = leftHandCenterY->value();
  handCenter(2) = leftHandCenterZ->value();
  hand->setCenter(handCenter);
  vector3d thumbAxis;
  thumbAxis(0) = leftThumbAxisX->value();
  thumbAxis(1) = leftThumbAxisY->value();
  thumbAxis(2) = leftThumbAxisZ->value();
  hand->setThumbAxis(thumbAxis);
  vector3d foreFingerAxis;
  foreFingerAxis(0) = leftForeFingerAxisX->value();
  foreFingerAxis(1) = leftForeFingerAxisY->value();
  foreFingerAxis(2) = leftForeFingerAxisZ->value();
  hand->setForeFingerAxis(foreFingerAxis);
  vector3d palmNormal;
  palmNormal(0) = leftPalmNormalX->value();
  palmNormal(1) = leftPalmNormalY->value();
  palmNormal(2) = leftPalmNormalZ->value();
  hand->setPalmNormal(palmNormal);

  // right wrist
  name = rightWrist->value();
  if (jointMap_.count(name) != 1) {
    message = "Joint with name " + name + "does not seem to exist.";
    throw exception(message);
  }
  jrlJoint = jointMap_[name]->jrlJoint();
  robot->rightWrist(jrlJoint);
  hand=objectFactory_.createHand(jrlJoint);
  hand->setAssociatedWrist(jrlJoint);
  robot->rightHand(hand);
  handCenter(1) *= -1;
  hand->setCenter(handCenter);
  thumbAxis(1) *= -1;
  hand->setThumbAxis(thumbAxis);
  foreFingerAxis(1) *= -1;
  hand->setForeFingerAxis(foreFingerAxis);
  palmNormal(1) *= -1;
  hand->setPalmNormal(palmNormal);

  // waist
  name = waist->value();
  if (jointMap_.count(name) != 1) {
    message = "Joint with name " + name + "does not seem to exist.";
    throw exception(message);
  }
  jrlJoint = jointMap_[name]->jrlJoint();
  robot->waist(jrlJoint);

  // chest
  name = chest->value();
  if (jointMap_.count(name) != 1) {
    message = "Joint with name " + name + "does not seem to exist.";
    throw exception(message);
  }
  jrlJoint = jointMap_[name]->jrlJoint();
  robot->chest(jrlJoint);
}
