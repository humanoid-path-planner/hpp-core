/*
 *  Copyright (c) 2010 LAAS-CNRS
 *
 *  Author: Florent Lamiraux
 */

#include <iostream>
#include <typeinfo>
#include <kprParserXML/kprParserManager.h>
#include <hppModel/hppHumanoidRobot.h>
#include "hpp/core/parser.hh"
#include "hpp/core/freeflyer-joint.hh"

ktStatus hpp::core::Parser::propertyFiller(const std::string& inContent,
					   CkppPropertyShPtr& inOutProperty)
{
  std::cout << "propertyFiller(" << inContent << ")"
	    << std::endl
	    << "property type: " << typeid(inOutProperty.get()).name()
	    << std::endl;
  return KD_OK;
}

hpp::core::Parser::Parser()
{
  ktStatus status = KD_ERROR;
  std::cout << "Registering Parser::writeHumanoidRobot" << std::endl;
  CkprParserManager::defaultManager()->addXMLWriterMethod < Parser >
    (this, &Parser::writeHumanoidRobot);
  std::cout << "Registering Parser::buildHumanoidRobot" << std::endl;
  status =
    CkprParserManager::defaultManager()->addXMLInheritedBuilderMethod < Parser >
    ("HPP_HUMANOID_ROBOT", "DEVICE", this, &Parser::buildHumanoidRobot, NULL);
  assert(status == KD_OK);
  std::cout << "Registering Parser::writeHppFreeflyerJoint" << std::endl;
  CkprParserManager::defaultManager()->addXMLWriterMethod < Parser >
    (this, &Parser::writeHppFreeflyerJoint);
  std::cout << "Registering Parser::buildHppFreeflyerJoint" << std::endl;
  status =
    CkprParserManager::defaultManager()->addXMLInheritedBuilderMethod < Parser >
    ("HPP_FREEFLYER_JOINT", "FREEFLYER_JOINT", this,
     &Parser::buildHppFreeflyerJoint, NULL);
  assert(status == KD_OK);
  // std::cout << "Registering " << std::endl;
  // CkprParserManager::defaultManager()->addPropertyFillerMethod < Parser >
  //   (this, &hpp::core::Parser::propertyFiller);
}

hpp::core::Parser::~Parser()
{
}

ktStatus hpp::core::Parser::writeHumanoidRobot
(const CkppComponentConstShPtr& inComponent,
 CkprXMLWriterShPtr& inOutWriter,
 CkprXMLTagShPtr& inOutTag)
{
  std::cout << "hpp::core::Parser::writeHumanoidRobot" << std::endl;
  if (KIT_DYNAMIC_PTR_CAST(const ChppHumanoidRobot, inComponent)) {
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
  std::cout << "Creating a humanoid robot" << std::endl;
  outComponent = ChppHumanoidRobot::create("Humanoid Robot");
  return KD_OK;
}

ktStatus hpp::core::Parser::
writeHppFreeflyerJoint(const CkppComponentConstShPtr& inComponent,
		       CkprXMLWriterShPtr& inOutWriter,
		       CkprXMLTagShPtr& inOutTag)
{
  std::cout << "hpp::core::Parser::writeHppFreeflyerJoint" << std::endl;
  if (KIT_DYNAMIC_PTR_CAST(const FreeflyerJoint, inComponent)) {
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
  std::cout << "Creating a joint" << std::endl;
  outComponent = FreeflyerJoint::create("FREEFLYER");
  return KD_OK;
}

