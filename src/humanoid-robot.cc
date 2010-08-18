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

CimplObjectFactory HumanoidRobot::objectFactory_=CimplObjectFactory();
const CkppProperty::TPropertyID
HumanoidRobot::GAZE_ID(CkppProperty::makeID());
const std::string HumanoidRobot::GAZE_STRING_ID("GAZE");
const CkppProperty::TPropertyID
HumanoidRobot::LEFTANKLE_ID(CkppProperty::makeID());
const std::string HumanoidRobot::LEFTANKLE_STRING_ID("LEFTANKLE");
const CkppProperty::TPropertyID
HumanoidRobot::RIGHTANKLE_ID(CkppProperty::makeID());
const std::string HumanoidRobot::RIGHTANKLE_STRING_ID("RIGHTANKLE");
const CkppProperty::TPropertyID
HumanoidRobot::LEFTWRIST_ID(CkppProperty::makeID());
const std::string HumanoidRobot::LEFTWRIST_STRING_ID("LEFTWRIST");
const CkppProperty::TPropertyID
HumanoidRobot::RIGHTWRIST_ID(CkppProperty::makeID());
const std::string HumanoidRobot::RIGHTWRIST_STRING_ID("RIGHTWRIST");
const CkppProperty::TPropertyID
HumanoidRobot::WAIST_ID(CkppProperty::makeID());
const std::string HumanoidRobot::WAIST_STRING_ID("WAIST");
const CkppProperty::TPropertyID
HumanoidRobot::CHEST_ID(CkppProperty::makeID());
const std::string HumanoidRobot::CHEST_STRING_ID("CHEST");
const CkppProperty::TPropertyID
HumanoidRobot::GAZEORIGINX_ID(CkppProperty::makeID());
const std::string HumanoidRobot::GAZEORIGINX_STRING_ID("GAZEORIGINX");
const CkppProperty::TPropertyID
HumanoidRobot::GAZEORIGINY_ID(CkppProperty::makeID());
const std::string HumanoidRobot::GAZEORIGINY_STRING_ID("GAZEORIGINY");
const CkppProperty::TPropertyID
HumanoidRobot::GAZEORIGINZ_ID(CkppProperty::makeID());
const std::string HumanoidRobot::GAZEORIGINZ_STRING_ID("GAZEORIGINZ");
const CkppProperty::TPropertyID
HumanoidRobot::GAZEDIRECTIONX_ID(CkppProperty::makeID());
const std::string HumanoidRobot::GAZEDIRECTIONX_STRING_ID("GAZEDIRECTIONX");
const CkppProperty::TPropertyID
HumanoidRobot::GAZEDIRECTIONY_ID(CkppProperty::makeID());
const std::string HumanoidRobot::GAZEDIRECTIONY_STRING_ID("GAZEDIRECTIONY");
const CkppProperty::TPropertyID
HumanoidRobot::GAZEDIRECTIONZ_ID(CkppProperty::makeID());
const std::string HumanoidRobot::GAZEDIRECTIONZ_STRING_ID("GAZEDIRECTIONZ");
const CkppProperty::TPropertyID
HumanoidRobot::ANKLEPOSINLEFTFOOTFRAMEX_ID(CkppProperty::makeID());
const std::string
HumanoidRobot::ANKLEPOSINLEFTFOOTFRAMEX_STRING_ID("ANKLEPOSINLEFTFOOTFRAMEX");
const CkppProperty::TPropertyID
HumanoidRobot::ANKLEPOSINLEFTFOOTFRAMEY_ID(CkppProperty::makeID());
const std::string
HumanoidRobot::ANKLEPOSINLEFTFOOTFRAMEY_STRING_ID("ANKLEPOSINLEFTFOOTFRAMEY");
const CkppProperty::TPropertyID
HumanoidRobot::ANKLEPOSINLEFTFOOTFRAMEZ_ID(CkppProperty::makeID());
const std::string
HumanoidRobot::ANKLEPOSINLEFTFOOTFRAMEZ_STRING_ID("ANKLEPOSINLEFTFOOTFRAMEZ");
const CkppProperty::TPropertyID
HumanoidRobot::SOLECENTERINLEFTFOOTFRAMEX_ID(CkppProperty::makeID());
const std::string
HumanoidRobot::SOLECENTERINLEFTFOOTFRAMEX_STRING_ID
("SOLECENTERINLEFTFOOTFRAMEX");
const CkppProperty::TPropertyID
HumanoidRobot::SOLECENTERINLEFTFOOTFRAMEY_ID(CkppProperty::makeID());
const std::string
HumanoidRobot::SOLECENTERINLEFTFOOTFRAMEY_STRING_ID
("SOLECENTERINLEFTFOOTFRAMEY");
const CkppProperty::TPropertyID
HumanoidRobot::SOLECENTERINLEFTFOOTFRAMEZ_ID(CkppProperty::makeID());
const std::string
HumanoidRobot::SOLECENTERINLEFTFOOTFRAMEZ_STRING_ID
("SOLECENTERINLEFTFOOTFRAMEZ");
const CkppProperty::TPropertyID
HumanoidRobot::SOLELENGTH_ID(CkppProperty::makeID());
const std::string HumanoidRobot::SOLELENGTH_STRING_ID("SOLELENGTH");
const CkppProperty::TPropertyID
HumanoidRobot::SOLEWIDTH_ID(CkppProperty::makeID());
const std::string HumanoidRobot::SOLEWIDTH_STRING_ID("SOLEWIDTH");
const CkppProperty::TPropertyID
HumanoidRobot::LEFTHANDCENTERX_ID(CkppProperty::makeID());
const std::string HumanoidRobot::LEFTHANDCENTERX_STRING_ID("LEFTHANDCENTERX");
const CkppProperty::TPropertyID
HumanoidRobot::LEFTHANDCENTERY_ID(CkppProperty::makeID());
const std::string HumanoidRobot::LEFTHANDCENTERY_STRING_ID("LEFTHANDCENTERY");
const CkppProperty::TPropertyID
HumanoidRobot::LEFTHANDCENTERZ_ID(CkppProperty::makeID());
const std::string HumanoidRobot::LEFTHANDCENTERZ_STRING_ID("LEFTHANDCENTERZ");
const CkppProperty::TPropertyID
HumanoidRobot::LEFTTHUMBAXISX_ID(CkppProperty::makeID());
const std::string HumanoidRobot::LEFTTHUMBAXISX_STRING_ID("LEFTTHUMBAXISX");
const CkppProperty::TPropertyID
HumanoidRobot::LEFTTHUMBAXISY_ID(CkppProperty::makeID());
const std::string HumanoidRobot::LEFTTHUMBAXISY_STRING_ID("LEFTTHUMBAXISY");
const CkppProperty::TPropertyID
HumanoidRobot::LEFTTHUMBAXISZ_ID(CkppProperty::makeID());
const std::string HumanoidRobot::LEFTTHUMBAXISZ_STRING_ID("LEFTTHUMBAXISZ");
const CkppProperty::TPropertyID
HumanoidRobot::LEFTFOREFINGERAXISX_ID(CkppProperty::makeID());
const std::string
HumanoidRobot::LEFTFOREFINGERAXISX_STRING_ID("LEFTFOREFINGERAXISX");
const CkppProperty::TPropertyID
HumanoidRobot::LEFTFOREFINGERAXISY_ID(CkppProperty::makeID());
const std::string
HumanoidRobot::LEFTFOREFINGERAXISY_STRING_ID("LEFTFOREFINGERAXISY");
const CkppProperty::TPropertyID
HumanoidRobot::LEFTFOREFINGERAXISZ_ID(CkppProperty::makeID());
const std::string
HumanoidRobot::LEFTFOREFINGERAXISZ_STRING_ID("LEFTFOREFINGERAXISZ");
const CkppProperty::TPropertyID
HumanoidRobot::LEFTPALMNORMALX_ID(CkppProperty::makeID());
const std::string HumanoidRobot::LEFTPALMNORMALX_STRING_ID("LEFTPALMNORMALX");
const CkppProperty::TPropertyID
HumanoidRobot::LEFTPALMNORMALY_ID(CkppProperty::makeID());
const std::string HumanoidRobot::LEFTPALMNORMALY_STRING_ID("LEFTPALMNORMALY");
const CkppProperty::TPropertyID
HumanoidRobot::LEFTPALMNORMALZ_ID(CkppProperty::makeID());
const std::string HumanoidRobot::LEFTPALMNORMALZ_STRING_ID("LEFTPALMNORMALZ");

ktStatus HumanoidRobot::init (const HumanoidRobotWkPtr &inWeakPtr,
			      const std::string &inName)
{
  if (CkppDeviceComponent::init(inWeakPtr, inName) != KD_OK) return KD_ERROR;
  // Initialize properties
  CkppComponentShPtr component = inWeakPtr.lock();

  gaze = CkppStringProperty::create("GAZE", component,
				    GAZE_ID,
				    GAZE_STRING_ID);
  if (!gaze) return KD_ERROR;

  leftAnkle = CkppStringProperty::create("LEFTANKLE", component,
					 LEFTANKLE_ID,
					 LEFTANKLE_STRING_ID);
  if (!leftAnkle) return KD_ERROR;

  rightAnkle = CkppStringProperty::create("RIGHTANKLE", component,
					  RIGHTANKLE_ID,
					  RIGHTANKLE_STRING_ID);
  if (!rightAnkle) return KD_ERROR;

  leftWrist = CkppStringProperty::create("LEFTWRIST", component,
					 LEFTWRIST_ID,
					 LEFTWRIST_STRING_ID);
  if (!leftWrist) return KD_ERROR;

  rightWrist = CkppStringProperty::create("RIGHTWRIST", component,
					  RIGHTWRIST_ID,
					  RIGHTWRIST_STRING_ID);
  if (!rightWrist) return KD_ERROR;

  waist = CkppStringProperty::create("WAIST", component,
				     WAIST_ID,
				     WAIST_STRING_ID);
  if (!waist) return KD_ERROR;

  chest = CkppStringProperty::create("CHEST", component,
				     CHEST_ID,
				     CHEST_STRING_ID);
  if (!chest) return KD_ERROR;

  gazeOriginX =
    CkppDoubleProperty::create("GAZEORIGINX", component,
			       GAZEORIGINX_ID,
			       GAZEORIGINX_STRING_ID);
  if (!gazeOriginX) return KD_ERROR;

  gazeOriginY =
    CkppDoubleProperty::create("GAZEORIGINY", component,
			       GAZEORIGINY_ID,
			       GAZEORIGINY_STRING_ID);
  if (!gazeOriginY) return KD_ERROR;

  gazeOriginZ =
    CkppDoubleProperty::create("GAZEORIGINZ", component,
			       GAZEORIGINZ_ID,
			       GAZEORIGINZ_STRING_ID);
  if (!gazeOriginZ) return KD_ERROR;

  gazeDirectionX =
    CkppDoubleProperty::create("GAZEDIRECTIONX", component,
			       GAZEDIRECTIONX_ID,
			       GAZEDIRECTIONX_STRING_ID);
  if (!gazeDirectionX) return KD_ERROR;

  gazeDirectionY =
    CkppDoubleProperty::create("GAZEDIRECTIONY", component,
			       GAZEDIRECTIONY_ID,
			       GAZEDIRECTIONY_STRING_ID);
  if (!gazeDirectionY) return KD_ERROR;

  gazeDirectionZ =
    CkppDoubleProperty::create("GAZEDIRECTIONZ", component,
			       GAZEDIRECTIONZ_ID,
			       GAZEDIRECTIONZ_STRING_ID);
  if (!gazeDirectionZ) return KD_ERROR;

  anklePosInLeftFootFrameX =
    CkppDoubleProperty::create("ANKLEPOSINLEFTFOOTFRAMEX", component,
			       ANKLEPOSINLEFTFOOTFRAMEX_ID,
			       ANKLEPOSINLEFTFOOTFRAMEX_STRING_ID);
  if (!anklePosInLeftFootFrameX) return KD_ERROR;

  anklePosInLeftFootFrameY =
    CkppDoubleProperty::create("ANKLEPOSINLEFTFOOTFRAMEY", component,
			       ANKLEPOSINLEFTFOOTFRAMEY_ID,
			       ANKLEPOSINLEFTFOOTFRAMEY_STRING_ID);
  if (!anklePosInLeftFootFrameY) return KD_ERROR;

  anklePosInLeftFootFrameZ =
    CkppDoubleProperty::create("ANKLEPOSINLEFTFOOTFRAMEZ", component,
			       ANKLEPOSINLEFTFOOTFRAMEZ_ID,
			       ANKLEPOSINLEFTFOOTFRAMEZ_STRING_ID);
  if (!anklePosInLeftFootFrameZ) return KD_ERROR;

  soleCenterInLeftFootFrameX =
    CkppDoubleProperty::create("SOLECENTERINLEFTFOOTFRAMEX", component,
			       SOLECENTERINLEFTFOOTFRAMEX_ID,
			       SOLECENTERINLEFTFOOTFRAMEX_STRING_ID);
  if (!soleCenterInLeftFootFrameX) return KD_ERROR;

  soleCenterInLeftFootFrameY =
    CkppDoubleProperty::create("SOLECENTERINLEFTFOOTFRAMEY", component,
			       SOLECENTERINLEFTFOOTFRAMEY_ID,
			       SOLECENTERINLEFTFOOTFRAMEY_STRING_ID);
  if (!soleCenterInLeftFootFrameY) return KD_ERROR;

  soleCenterInLeftFootFrameZ =
    CkppDoubleProperty::create("SOLECENTERINLEFTFOOTFRAMEZ", component,
			       SOLECENTERINLEFTFOOTFRAMEZ_ID,
			       SOLECENTERINLEFTFOOTFRAMEZ_STRING_ID);
  if (!soleCenterInLeftFootFrameZ) return KD_ERROR;

  soleLength = CkppDoubleProperty::create("SOLELENGTH", component,
					  SOLELENGTH_ID,
					  SOLELENGTH_STRING_ID);
  if (!soleLength) return KD_ERROR;

  soleWidth = CkppDoubleProperty::create("SOLEWIDTH", component,
					 SOLEWIDTH_ID,
					 SOLEWIDTH_STRING_ID);
  if (!soleWidth) return KD_ERROR;

  leftHandCenterX = CkppDoubleProperty::create("LEFTHANDCENTERX", component,
					       LEFTHANDCENTERX_ID,
					       LEFTHANDCENTERX_STRING_ID);
  if (!leftHandCenterX) return KD_ERROR;

  leftHandCenterY = CkppDoubleProperty::create("LEFTHANDCENTERY", component,
					       LEFTHANDCENTERY_ID,
					       LEFTHANDCENTERY_STRING_ID);
  if (!leftHandCenterY) return KD_ERROR;

  leftHandCenterZ = CkppDoubleProperty::create("LEFTHANDCENTERZ", component,
					       LEFTHANDCENTERZ_ID,
					       LEFTHANDCENTERZ_STRING_ID);
  if (!leftHandCenterZ) return KD_ERROR;

  leftThumbAxisX = CkppDoubleProperty::create("LEFTTHUMBAXISX", component,
					      LEFTTHUMBAXISX_ID,
					      LEFTTHUMBAXISX_STRING_ID);
  if (!leftThumbAxisX) return KD_ERROR;

  leftThumbAxisY = CkppDoubleProperty::create("LEFTTHUMBAXISY", component,
					      LEFTTHUMBAXISY_ID,
					      LEFTTHUMBAXISY_STRING_ID);
  if (!leftThumbAxisY) return KD_ERROR;

  leftThumbAxisZ = CkppDoubleProperty::create("LEFTTHUMBAXISZ", component,
					      LEFTTHUMBAXISZ_ID,
					      LEFTTHUMBAXISZ_STRING_ID);
  if (!leftThumbAxisZ) return KD_ERROR;

  leftForeFingerAxisX =
    CkppDoubleProperty::create("LEFTFOREFINGERAXISX", component,
			       LEFTFOREFINGERAXISX_ID,
			       LEFTFOREFINGERAXISX_STRING_ID);
  if (!leftForeFingerAxisX) return KD_ERROR;

  leftForeFingerAxisY =
    CkppDoubleProperty::create("LEFTFOREFINGERAXISY", component,
			       LEFTFOREFINGERAXISY_ID,
			       LEFTFOREFINGERAXISY_STRING_ID);
  if (!leftForeFingerAxisY) return KD_ERROR;

  leftForeFingerAxisZ =
    CkppDoubleProperty::create("LEFTFOREFINGERAXISZ", component,
			       LEFTFOREFINGERAXISZ_ID,
			       LEFTFOREFINGERAXISZ_STRING_ID);
  if (!leftForeFingerAxisZ) return KD_ERROR;

  leftPalmNormalX = CkppDoubleProperty::create("LEFTPALMNORMALX", component,
					       LEFTPALMNORMALX_ID,
					       LEFTPALMNORMALX_STRING_ID);
  if (!leftPalmNormalX) return KD_ERROR;

  leftPalmNormalY = CkppDoubleProperty::create("LEFTPALMNORMALY", component,
					       LEFTPALMNORMALY_ID,
					       LEFTPALMNORMALY_STRING_ID);
  if (!leftPalmNormalY) return KD_ERROR;

  leftPalmNormalZ = CkppDoubleProperty::create("LEFTPALMNORMALZ", component,
					       LEFTPALMNORMALZ_ID,
					       LEFTPALMNORMALZ_STRING_ID);
  if (!leftPalmNormalZ) return KD_ERROR;

  return KD_OK;
}

void
HumanoidRobot::fillPropertyVector(std::vector<CkppPropertyShPtr>&
				  inOutPropertyVector) const
{
  CkppDeviceComponent::fillPropertyVector(inOutPropertyVector);
  inOutPropertyVector.push_back(gaze);
  inOutPropertyVector.push_back(leftAnkle);
  inOutPropertyVector.push_back(rightAnkle);
  inOutPropertyVector.push_back(leftWrist);
  inOutPropertyVector.push_back(rightWrist);
  inOutPropertyVector.push_back(waist);
  inOutPropertyVector.push_back(chest);
  inOutPropertyVector.push_back(gazeOriginX);
  inOutPropertyVector.push_back(gazeOriginY);
  inOutPropertyVector.push_back(gazeOriginZ);
  inOutPropertyVector.push_back(gazeDirectionX);
  inOutPropertyVector.push_back(gazeDirectionY);
  inOutPropertyVector.push_back(gazeDirectionZ);
  inOutPropertyVector.push_back(anklePosInLeftFootFrameX);
  inOutPropertyVector.push_back(anklePosInLeftFootFrameY);
  inOutPropertyVector.push_back(anklePosInLeftFootFrameZ);
  inOutPropertyVector.push_back(soleCenterInLeftFootFrameX);
  inOutPropertyVector.push_back(soleCenterInLeftFootFrameY);
  inOutPropertyVector.push_back(soleCenterInLeftFootFrameZ);
  inOutPropertyVector.push_back(soleLength);
  inOutPropertyVector.push_back(soleWidth);
  inOutPropertyVector.push_back(leftHandCenterX);
  inOutPropertyVector.push_back(leftHandCenterY);
  inOutPropertyVector.push_back(leftHandCenterZ);
  inOutPropertyVector.push_back(leftThumbAxisX);
  inOutPropertyVector.push_back(leftThumbAxisY);
  inOutPropertyVector.push_back(leftThumbAxisZ);
  inOutPropertyVector.push_back(leftForeFingerAxisX);
  inOutPropertyVector.push_back(leftForeFingerAxisY);
  inOutPropertyVector.push_back(leftForeFingerAxisZ);
  inOutPropertyVector.push_back(leftPalmNormalX);
  inOutPropertyVector.push_back(leftPalmNormalY);
  inOutPropertyVector.push_back(leftPalmNormalZ);
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
HumanoidRobot::buildKinematicChain(ChppHumanoidRobotShPtr inRobot,
				   const CkppJointComponentShPtr& inJoint)
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
      buildKinematicChain(inRobot, inJoint->childJointComponent(iJoint));
    if (!hppJoint->addChildJoint(childJoint)) {
      message = "Cannot add joint " + childJoint->kppJoint()->name()
	+ " as child of " + hppJoint->kppJoint()->name() + ".";
      throw exception(message);
    }
  }
  return hppJoint;
}

void HumanoidRobot::setHumanoidProperties(const ChppHumanoidRobotShPtr& inRobot)
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
  inRobot->gazeJoint(jrlJoint);
  vector3d dir,origin;
  origin[0] = gazeOriginX->value();
  origin[1] = gazeOriginY->value();
  origin[2] = gazeOriginZ->value();
  dir[0] = gazeDirectionX->value();
  dir[1] = gazeDirectionY->value();
  dir[2] = gazeDirectionZ->value();
  inRobot->gaze(dir, origin);

  // left ankle
  name = leftAnkle->value();
  if (jointMap_.count(name) != 1) {
    message = "Joint with name " + name + "does not seem to exist.";
    throw exception(message);
  }
  jrlJoint = jointMap_[name]->jrlJoint();
  inRobot->leftAnkle(jrlJoint);
  CjrlFoot *foot = objectFactory_.createFoot(jrlJoint);
  foot->setAssociatedAnkle(jrlJoint);
  vector3d anklePosInFootFrame;
  anklePosInFootFrame(0) = anklePosInLeftFootFrameX->value();
  anklePosInFootFrame(1) = anklePosInLeftFootFrameY->value();
  anklePosInFootFrame(2) = anklePosInLeftFootFrameZ->value();
  foot->setAnklePositionInLocalFrame(anklePosInFootFrame);
  vector3d soleCenterInFootFrame;
  soleCenterInFootFrame(0) = soleCenterInLeftFootFrameX->value();
  soleCenterInFootFrame(1) = soleCenterInLeftFootFrameY->value();
  soleCenterInFootFrame(2) = soleCenterInLeftFootFrameZ->value();
  foot->setSoleCenterInLocalFrame(soleCenterInFootFrame);
  foot->setSoleSize(soleLength->value(), soleWidth->value());
  vector3d vzero;
  vzero(0) =  vzero(1) =  vzero(2) = 0.0;
  foot->setProjectionCenterLocalFrameInSole(vzero);
  inRobot->leftFoot(foot);
  // right ankle
  name = rightAnkle->value();
  if (jointMap_.count(name) != 1) {
    message = "Joint with name " + name + "does not seem to exist.";
    throw exception(message);
  }
  jrlJoint = jointMap_[name]->jrlJoint();
  inRobot->rightAnkle(jrlJoint);
  foot = objectFactory_.createFoot(jrlJoint);
  foot->setAssociatedAnkle(jrlJoint);
  // Get right foot values by symmetry
  anklePosInFootFrame(1) *= -1;
  soleCenterInFootFrame(1) *= -1;
  foot->setAnklePositionInLocalFrame(anklePosInFootFrame);
  foot->setSoleCenterInLocalFrame(soleCenterInFootFrame);
  foot->setSoleSize(soleLength->value(), soleWidth->value());
  foot->setProjectionCenterLocalFrameInSole(vzero);
  inRobot->rightFoot(foot);

  // left wrist
  name = leftWrist->value();
  if (jointMap_.count(name) != 1) {
    message = "Joint with name " + name + "does not seem to exist.";
    throw exception(message);
  }
  jrlJoint = jointMap_[name]->jrlJoint();
  inRobot->leftWrist(jrlJoint);
  CjrlHand* hand=objectFactory_.createHand(jrlJoint);
  hand->setAssociatedWrist(jrlJoint);
  inRobot->leftHand(hand);
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
  inRobot->rightWrist(jrlJoint);
  hand=objectFactory_.createHand(jrlJoint);
  hand->setAssociatedWrist(jrlJoint);
  inRobot->rightHand(hand);
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
  inRobot->waist(jrlJoint);

  // chest
  name = chest->value();
  if (jointMap_.count(name) != 1) {
    message = "Joint with name " + name + "does not seem to exist.";
    throw exception(message);
  }
  jrlJoint = jointMap_[name]->jrlJoint();
  inRobot->chest(jrlJoint);
}
