/*
 *  Copyright (c) 2010 LAAS-CNRS
 *
 *  Author: Florent Lamiraux
 */

#include <iostream>
#include <kprParserXML/kprParserManager.h>
#include <hppModel/hppHumanoidRobot.h>
#include "hpp/core/parser.hh"
#include "hpp/core/freeflyer-joint.hh"

hpp::core::Parser::Parser()
{
  CkprParserManager::defaultManager()->addXMLWriterMethod < Parser >
    (this, &Parser::writeHumanoidRobot);
  CkprParserManager::defaultManager()->addXMLTagBuilderMethod < Parser >
    ("HUMANOID_ROBOT", this, &Parser::buildHumanoidRobot, NULL);
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
  if (KIT_DYNAMIC_PTR_CAST(ChppHumanoidRobot const, inComponent)) {
    inOutTag->name("HUMANOID_ROBOT");
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
  if (KIT_DYNAMIC_PTR_CAST(FreeflyerJoint const, inComponent)) {
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

