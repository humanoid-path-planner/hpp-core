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

KIT_PREDEF_CLASS(CkppDeviceComponent);

namespace hpp {
  namespace core {
    class Parser {
    public:
      Parser();
      ~Parser();

      ///
      /// \name Call back for kxml read write
      /// @{

      /// \brief Write local class representing a humanoid robot into kxml file.
      ktStatus writeHumanoidRobot(const CkppComponentConstShPtr& inComponent,
				  CkprXMLWriterShPtr& inOutWriter,
				  CkprXMLTagShPtr& inOutTag);
      /// \brief Build local class representing a humanoid robot from kxml file.
      ktStatus buildHumanoidRobot(const CkprXMLTagConstShPtr& inTag,
				  const CkppComponentShPtr&
				  inOutParentComponent,
				  std::vector< CkppComponentShPtr >&
				  inPrebuiltChildComponentVector,
				  CkprXMLBuildingContextShPtr& inOutContext,
				  CkppComponentShPtr& outComponent);
      /// \brief Write local class representing a freeflyer joint into kxml file.
      ktStatus
      writeHppFreeflyerJoint(const CkppComponentConstShPtr& inComponent,
			     CkprXMLWriterShPtr& inOutWriter,
			     CkprXMLTagShPtr& inOutTag);
      /// \brief Build local class representing a freeflyer joint from kxml file.
      ktStatus
      buildHppFreeflyerJoint(const CkprXMLTagConstShPtr& inTag,
			     const CkppComponentShPtr&
			     inOutParentComponent,
			     std::vector< CkppComponentShPtr >&
			     inPrebuiltChildComponentVector,
			     CkprXMLBuildingContextShPtr& inOutContext,
			     CkppComponentShPtr& outComponent);
      /// \brief Write local class representing a rotation joint into kxml file.
      ktStatus
      writeHppRotationJoint(const CkppComponentConstShPtr& inComponent,
			    CkprXMLWriterShPtr& inOutWriter,
			    CkprXMLTagShPtr& inOutTag);
      /// \brief Build local class representing a rotation joint from kxml file.
      ktStatus
      buildHppRotationJoint(const CkprXMLTagConstShPtr& inTag,
			    const CkppComponentShPtr&
			    inOutParentComponent,
			    std::vector< CkppComponentShPtr >&
			    inPrebuiltChildComponentVector,
			    CkprXMLBuildingContextShPtr& inOutContext,
			    CkppComponentShPtr& outComponent);
      /// \brief Write local class representing a translation joint into kxml file.
      ktStatus
      writeHppTranslationJoint(const CkppComponentConstShPtr& inComponent,
			       CkprXMLWriterShPtr& inOutWriter,
			       CkprXMLTagShPtr& inOutTag);
      /// \brief Build local class representing a translation joint from kxml file.
      ktStatus
      buildHppTranslationJoint(const CkprXMLTagConstShPtr& inTag,
			       const CkppComponentShPtr&
			       inOutParentComponent,
			       std::vector< CkppComponentShPtr >&
			       inPrebuiltChildComponentVector,
			       CkprXMLBuildingContextShPtr& inOutContext,
			       CkppComponentShPtr& outComponent);
      /// @}
      static CkppDeviceComponentShPtr buildDummyDevice();
    }; // Parser
  } // namespace core
} // namespace hpp

#endif // HPPMODEL_PARSER_HH
