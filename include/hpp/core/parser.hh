/*
 *  Copyright (c) 2010 LAAS-CNRS
 *
 *  Author: Florent Lamiraux
 */

#ifndef HPPMODEL_PARSER_HH
#define HPPMODEL_PARSER_HH

#include <KineoModel/kppComponent.h>
#include <kprParserXML/kprXMLTag.h>
#include <kprParserXML/kprXMLBuildingContext.h>
#include <kprParserXML/kprXMLWriter.h>

namespace hpp {
  namespace core {
    class Parser {
    public:
      Parser();
      ~Parser();

      ktStatus propertyFiller(const std::string& inContent, 
			      CkppPropertyShPtr& inOutProperty);

      ktStatus writeHumanoidRobot(const CkppComponentConstShPtr& inComponent,
				  CkprXMLWriterShPtr& inOutWriter,
				  CkprXMLTagShPtr& inOutTag);
      ktStatus buildHumanoidRobot(const CkprXMLTagConstShPtr& inTag,
				  const CkppComponentShPtr&
				  inOutParentComponent,
				  std::vector< CkppComponentShPtr >&
				  inPrebuiltChildComponentVector,
				  CkprXMLBuildingContextShPtr& inOutContext,
				  CkppComponentShPtr& outComponent);
      ktStatus
      writeHppFreeflyerJoint(const CkppComponentConstShPtr& inComponent,
			     CkprXMLWriterShPtr& inOutWriter,
			     CkprXMLTagShPtr& inOutTag);
      ktStatus
      buildHppFreeflyerJoint(const CkprXMLTagConstShPtr& inTag,
			     const CkppComponentShPtr&
			     inOutParentComponent,
			     std::vector< CkppComponentShPtr >&
			     inPrebuiltChildComponentVector,
			     CkprXMLBuildingContextShPtr& inOutContext,
			     CkppComponentShPtr& outComponent);
    }; // Parser
  } // namespace core
} // namespace hpp

#endif // HPPMODEL_PARSER_HH
