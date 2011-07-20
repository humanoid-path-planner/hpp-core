/*
 *  Copyright (c) 2010 LAAS-CNRS
 *
 *  Author: Florent Lamiraux
 */

#include <iostream>
#include <typeinfo>
#include <kprParserXML/kprParserManager.h>
#include <hpp/model/humanoid-robot.hh>
#include "hpp/core/parser.hh"
#include "freeflyer-joint.hh"
#include "rotation-joint.hh"
#include "translation-joint.hh"
#include "humanoid-robot.hh"

hpp::core::Parser::Parser()
{
  ktStatus status = KD_ERROR;
  // Write humanoid robot
  CkprParserManager::defaultManager()->addXMLWriterMethod < Parser >
    (this, &Parser::writeHumanoidRobot);
  // Read humanoid robot
  status =
    CkprParserManager::defaultManager()->addXMLInheritedBuilderMethod < Parser >
    ("HPP_HUMANOID_ROBOT", "DEVICE", this, &Parser::buildHumanoidRobot, NULL);
  assert(status == KD_OK);
  // Write freeflyer joint
  CkprParserManager::defaultManager()->addXMLWriterMethod < Parser >
    (this, &Parser::writeHppFreeflyerJoint);
  // Read freeflyer joint
  status =
    CkprParserManager::defaultManager()->addXMLInheritedBuilderMethod < Parser >
    ("HPP_FREEFLYER_JOINT", "FREEFLYER_JOINT", this,
     &Parser::buildHppFreeflyerJoint, NULL);
  assert(status == KD_OK);
  // Write rotation joint
  CkprParserManager::defaultManager()->addXMLWriterMethod < Parser >
    (this, &Parser::writeHppRotationJoint);
  // Read rotation joint
  status =
    CkprParserManager::defaultManager()->addXMLInheritedBuilderMethod < Parser >
    ("HPP_ROTATION_JOINT", "ROTATION_JOINT", this,
     &Parser::buildHppRotationJoint, NULL);
  assert(status == KD_OK);
  // Write translation joint
  CkprParserManager::defaultManager()->addXMLWriterMethod < Parser >
    (this, &Parser::writeHppTranslationJoint);
  // Read translation joint
  status =
    CkprParserManager::defaultManager()->addXMLInheritedBuilderMethod < Parser >
    ("HPP_TRANSLATION_JOINT", "TRANSLATION_JOINT", this,
     &Parser::buildHppTranslationJoint, NULL);
  assert(status == KD_OK);
}

hpp::core::Parser::~Parser()
{
}

ktStatus hpp::core::Parser::writeHumanoidRobot
(const CkppComponentConstShPtr& inComponent,
 CkprXMLWriterShPtr& inOutWriter,
 CkprXMLTagShPtr& inOutTag)
{
  if (KIT_DYNAMIC_PTR_CAST(const io::HumanoidRobot, inComponent)) {
    inOutTag->name("HPP_HUMANOID_ROBOT");
    return KD_OK;
  }
  return KD_ERROR;
}

ktStatus hpp::core::Parser::buildHumanoidRobot
(const CkprXMLTagConstShPtr& inTag,
 const CkppComponentShPtr& inOutParentComponent,
 std::vector< CkppComponentShPtr >& inPrebuiltChildComponentVector,
 CkprXMLBuildingContextShPtr& inOutContext,
 CkppComponentShPtr& outComponent)
{
  outComponent = io::HumanoidRobot::create("Humanoid Robot");
  return KD_OK;
}

ktStatus hpp::core::Parser::
writeHppFreeflyerJoint(const CkppComponentConstShPtr& inComponent,
		       CkprXMLWriterShPtr& inOutWriter,
		       CkprXMLTagShPtr& inOutTag)
{
  if (KIT_DYNAMIC_PTR_CAST(const io::FreeflyerJoint, inComponent)) {
    inOutTag->name("HPP_FREEFLYER_JOINT");
    return KD_OK;
  }
  return KD_ERROR;
}

ktStatus hpp::core::Parser::
buildHppFreeflyerJoint(const CkprXMLTagConstShPtr& inTag,
		       const CkppComponentShPtr& inOutParentComponent,
		       std::vector< CkppComponentShPtr >&
		       inPrebuiltChildComponentVector,
		       CkprXMLBuildingContextShPtr& inOutContext,
		       CkppComponentShPtr& outComponent)
{
  outComponent = io::FreeflyerJoint::create("FREEFLYER");
  return KD_OK;
}

ktStatus hpp::core::Parser::
writeHppRotationJoint(const CkppComponentConstShPtr& inComponent,
		       CkprXMLWriterShPtr& inOutWriter,
		       CkprXMLTagShPtr& inOutTag)
{
  if (KIT_DYNAMIC_PTR_CAST(const io::RotationJoint, inComponent)) {
    inOutTag->name("HPP_ROTATION_JOINT");
    return KD_OK;
  }
  return KD_ERROR;
}

ktStatus hpp::core::Parser::
buildHppRotationJoint(const CkprXMLTagConstShPtr& inTag,
		       const CkppComponentShPtr& inOutParentComponent,
		       std::vector< CkppComponentShPtr >&
		       inPrebuiltChildComponentVector,
		       CkprXMLBuildingContextShPtr& inOutContext,
		       CkppComponentShPtr& outComponent)
{
  outComponent = io::RotationJoint::create("ROTATION");
  return KD_OK;
}

ktStatus hpp::core::Parser::
writeHppTranslationJoint(const CkppComponentConstShPtr& inComponent,
		       CkprXMLWriterShPtr& inOutWriter,
		       CkprXMLTagShPtr& inOutTag)
{
  if (KIT_DYNAMIC_PTR_CAST(const io::TranslationJoint, inComponent)) {
    inOutTag->name("HPP_TRANSLATION_JOINT");
    return KD_OK;
  }
  return KD_ERROR;
}

ktStatus hpp::core::Parser::
buildHppTranslationJoint(const CkprXMLTagConstShPtr& inTag,
		       const CkppComponentShPtr& inOutParentComponent,
		       std::vector< CkppComponentShPtr >&
		       inPrebuiltChildComponentVector,
		       CkprXMLBuildingContextShPtr& inOutContext,
		       CkppComponentShPtr& outComponent)
{
  outComponent = io::TranslationJoint::create("TRANSLATION");
  return KD_OK;
}

CkppDeviceComponentShPtr hpp::core::Parser::buildDummyDevice()
{
  // Create a humanoid robot with one freeflyer joint
  io::HumanoidRobotShPtr robot = io::HumanoidRobot::create("TEST");
  io::FreeflyerJointShPtr joint = io::FreeflyerJoint::create("FF");
  joint->mass->value(1.55);
  robot->setRootJoint(joint);
  return robot;
}

