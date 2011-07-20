//
// Copyright (c) 2010, 2011 CNRS
// Authors: Florent Lamiraux
//
// This file is part of hpp-core
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_CORE_PARSER_HH
#define HPP_CORE_PARSER_HH

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

#endif // HPP_CORE_PARSER_HH
