//
// Copyright (c) 2007, 2008, 2009, 2010, 2011 CNRS
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

#include <iostream>

#include "KineoModel/kppSolidComponentRef.h"
#include "KineoModel/kppJointComponent.h"
#include "KineoKCDModel/kppKCDPolyhedron.h"
#include "KineoKCDModel/kppKCDAssembly.h"
#include "kcd2/kcdExactDistanceReport.h"
#include "kcd2/kcdAnalysis.h"
#include "kcd2/kcdGeometrySubElement.h"
#include "KineoWorks2/kwsJoint.h"

#include <hpp/util/debug.hh>

#include "hpp/model/body.hh"

namespace hpp {
  namespace model {

    BodyShPtr Body::create(const std::string& name)
    {
      Body* hppBody = new Body(name);
      BodyShPtr hppBodyShPtr(hppBody);
      BodyWkPtr hppBodyWkPtr = hppBodyShPtr;

      if (hppBody->init(hppBodyWkPtr) != KD_OK) {
	hppDout(error, " failed to create an object.");
	hppBodyShPtr.reset();
      }
      return hppBodyShPtr;
    }

    //=========================================================================

    ktStatus Body::init(const BodyWkPtr weakPtr)
    {
      weakPtr_ = weakPtr;
      return CkwsKCDBody::init(weakPtr);
    }

    //=========================================================================

    bool
    Body::addInnerObject(const CkppSolidComponentRefShPtr& solidComponentRef,
			 const CkitMat4& position,
			 bool distanceComputation)
    {
      CkppSolidComponentShPtr solidComponent =
	solidComponentRef->referencedSolidComponent();

#ifdef HPP_DEBUG
      std::string innerName = solidComponent->name();
      std::string outerName;
#endif
      /*
	Attach solid component to the joint associated to the body
      */
      CkwsJointShPtr bodyKwsJoint = CkwsBody::joint();
      CkppJointComponentShPtr bodyKppJoint =
	KIT_DYNAMIC_PTR_CAST(CkppJointComponent, bodyKwsJoint);

      // Test that body is attached to a joint
      if (bodyKppJoint) {
	solidComponent->setAbsolutePosition(position);
	bodyKppJoint->addSolidComponentRef(solidComponentRef);
      }
      else {
	hppDout(error, "The body is not attached to any joint");
	return false;
      }

      /*
	If requested, add the object in the list of objects the distance to which
	needs to be computed and build the corresponding analyses.
      */
      if (distanceComputation) {
	CkcdObjectShPtr innerObject =
	  KIT_DYNAMIC_PTR_CAST(CkcdObject, solidComponent);
	if (innerObject) {
	  hppDout(info, ":addInnerObject: adding " << solidComponent->name()
		  << " to list of objects for distance computation.");
	  innerObjForDist_.push_back(innerObject);
	  /*
	    Build Exact distance computation analyses for this object
	  */
	  const std::vector<CkcdObjectShPtr>& outerList = outerObjForDist_;
	  for (std::vector<CkcdObjectShPtr>::const_iterator it = outerList.begin();
	       it != outerList.end(); it++) {
	    const CkcdObjectShPtr& outerObject = *it;

#ifdef HPP_DEBUG
	    CkppSolidComponentShPtr solidComp =
	      KIT_DYNAMIC_PTR_CAST(CkppSolidComponent, outerObject);
	    if (solidComp) {
	      outerName = solidComp->name();
	    } else {
	      outerName = std::string("");
	    }
#endif
	    /* Instantiate the analysis object */
	    CkcdAnalysisShPtr analysis = CkcdAnalysis::create();
	    analysis->analysisType(CkcdAnalysisType::EXACT_DISTANCE);
	    /*
	      Ignore tolerance for distance computations
	    */
	    analysis->ignoreTolerance(true);

	    /* retrieve the test trees associated with the objects */
	    CkcdTestTreeShPtr leftTree = innerObject->collectTestTrees();
	    CkcdTestTreeShPtr rightTree = outerObject->collectTestTrees();

	    // associate the lists with the analysis object
	    analysis->leftTestTree(leftTree);
	    analysis->rightTestTree(rightTree);

	    hppDout(info, ":addInnerObject: creating analysis between "
		    << innerName << " and "
		    << outerName);
	    distCompPairs_.push_back(analysis);
	  }
	}
	else {
	  hppDout(error, "Cannot cast solid component into CkcdObject.");
	}
      }
      return true;
    }

    //=========================================================================

    void Body::addOuterObject(const CkcdObjectShPtr& outerObject,
			      bool distanceComputation)

    {
#if HPP_DEBUG
      std::string innerName;
      std::string outerName;
      CkppSolidComponentShPtr solidComp =
	KIT_DYNAMIC_PTR_CAST(CkppSolidComponent, outerObject);
      if (solidComp) {
	outerName = solidComp->name();
      } else {
	outerName = std::string("");
      }
#endif
      /*
	Append object at the end of KineoWorks set of outer objects
	for collision checking
      */
      std::vector<CkcdObjectShPtr> outerList = outerObjects();
      outerList.push_back(outerObject);
      outerObjects(outerList);

      /**
	 If distance computation is requested, build necessary CkcdAnalysis
	 objects
      */
      if (distanceComputation) {
	/*
	  Store object in case inner objects are added a posteriori
	*/
	outerObjForDist_.push_back(outerObject);

	/*
	  Build distance computation objects (CkcdAnalysis)
	*/
	const CkcdObjectShPtr& outerObject=outerObject;
	const std::vector<CkcdObjectShPtr> innerList = innerObjForDist_;
	for (std::vector<CkcdObjectShPtr>::const_iterator it = innerList.begin();
	     it != innerList.end();
	     it++) {
	  const CkcdObjectShPtr& innerObject=*it;

	  /* Instantiate the analysis object */
	  CkcdAnalysisShPtr analysis = CkcdAnalysis::create();
	  analysis->analysisType(CkcdAnalysisType::EXACT_DISTANCE);
	  /*
	    Ignore tolerance for distance computations
	  */
	  analysis->ignoreTolerance(true);

	  /* retrieve the test trees associated with the objects */
	  CkcdTestTreeShPtr leftTree = innerObject->collectTestTrees();
	  CkcdTestTreeShPtr rightTree = outerObject->collectTestTrees();

	  // associate the lists with the analysis object
	  analysis->leftTestTree(leftTree);
	  analysis->rightTestTree(rightTree);

#ifdef HPP_DEBUG
	  solidComp = KIT_DYNAMIC_PTR_CAST(CkppSolidComponent, innerObject);
	  if (solidComp) {
	    innerName = solidComp->name();
	  } else {
	    innerName = std::string("");
	  }
#endif
	  hppDout(info, ":addInnerObject: creating analysis between "
		  << innerName << " and "
		  << outerName);

	  distCompPairs_.push_back(analysis);
	}
      }
    }

    //=========================================================================

    void Body::resetOuterObjects()
    {
      outerObjForDist_.clear();
      distCompPairs_.clear();
    }


    //=========================================================================

    ktStatus
    Body::distAndPairsOfPoints(unsigned int pairId,
			       double& outDistance,
			       CkitPoint3& outPointBody,
			       CkitPoint3& outPointEnv,
			       CkcdObjectShPtr &outObjectBody,
			       CkcdObjectShPtr &outObjectEnv,
			       CkcdAnalysisType::Type type)
    {
      KWS_PRECONDITION(pairId < nbDistPairs());

      CkcdAnalysisShPtr analysis = distCompPairs_[pairId];
      analysis->analysisType(type);

      ktStatus status = analysis->compute();
      if (KD_SUCCEEDED(status)) {
	hppDout(info, ":distAndPairsOfPoints: compute succeeded.");
	unsigned int nbDistances = analysis->countExactDistanceReports();

	if(nbDistances == 0) {
	  //no distance information available, return 0 for instance;
	  hppDout(info, ":distAndPairsOfPoints: no distance report.");
	  outDistance = 0;

	  outObjectBody.reset();
	  outObjectEnv.reset();

	  return KD_OK;
	}
	else{

	  CkcdExactDistanceReportShPtr distanceReport;
	  CkitPoint3 leftPoint, rightPoint;

	  //distances are ordered from lowest value, to highest value.
	  distanceReport = analysis->exactDistanceReport(0);

	  //exact distance between two lists is always stored at the first
	  //rank of Distance reports.
	  outDistance = distanceReport->distance();

	  if(distanceReport->countPairs()>0) {
	    CkitMat4 firstPos, secondPos;
	    // get points in the frame of the object
	    distanceReport->getPoints(0,leftPoint,rightPoint) ;
	    // get geometry
	    outObjectBody = distanceReport->pair(0).first->geometry();
	    outObjectEnv = distanceReport->pair(0).second->geometry();
	    outObjectBody->getAbsolutePosition(firstPos);
	    outObjectEnv->getAbsolutePosition(secondPos);
	    //from these geometry points we can get points in absolute
	    //frame(world) or relative frame(geometry).
	    //here we want points in the absolute frame.
	    outPointBody= firstPos * leftPoint;
	    outPointEnv = secondPos * rightPoint;
	  }

	  // get object
	  outObjectBody = distanceReport->leftCollisionEntity();
	  outObjectEnv = distanceReport->rightCollisionEntity();

	  return KD_OK;
	}

      } else {
	hppDout(error, "Compute failed.");
	return KD_ERROR;
      }
      return KD_OK;
    }
  } // namespace model
} // namespace hpp
