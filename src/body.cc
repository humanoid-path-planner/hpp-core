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

#include "hpp/model/body.hh"

//=============================================================================

BodyShPtr Body::create(const std::string& inName)
{
  Body* hppBody = new Body(inName);
  BodyShPtr hppBodyShPtr(hppBody);
  BodyWkPtr hppBodyWkPtr = hppBodyShPtr;

  if (hppBody->init(hppBodyWkPtr) != KD_OK) {
    ODEBUG1(" error in create() ");
    hppBodyShPtr.reset();
  }
  return hppBodyShPtr;
}

//=============================================================================

ktStatus Body::init(const BodyWkPtr inBodyWkPtr)
{
  attWeakPtr = inBodyWkPtr;
  return CkwsKCDBody::init(inBodyWkPtr);
}

//=============================================================================

bool
Body::addInnerObject(const CkppSolidComponentRefShPtr& inSolidComponentRef,
			 const CkitMat4& inPosition,
			 bool distanceComputation)
{
  CkppSolidComponentShPtr solidComponent =
    inSolidComponentRef->referencedSolidComponent();

#if DEBUG >= 2
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
    solidComponent->setAbsolutePosition(inPosition);
    bodyKppJoint->addSolidComponentRef(inSolidComponentRef);
  }
  else {
    ODEBUG1
      ("Body::addSolidComponent: the body is not attached to any joint");
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
      ODEBUG2(":addInnerObject: adding " << solidComponent->name()
	      << " to list of objects for distance computation.");
      attInnerObjForDist.push_back(innerObject);
      /*
	Build Exact distance computation analyses for this object
      */
      const std::vector<CkcdObjectShPtr>& outerList = attOuterObjForDist;
      for (std::vector<CkcdObjectShPtr>::const_iterator it = outerList.begin();
	   it != outerList.end(); it++) {
	const CkcdObjectShPtr& outerObject = *it;

#if DEBUG >=2
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

	ODEBUG2(":addInnerObject: creating analysis between "
		<< innerName << " and "
		<< outerName);
	attDistCompPairs.push_back(analysis);
      }
    }
    else {
      ODEBUG1("addInnerObject: cannot cast solid component into CkcdObject.");
    }
  }
  return true;
}

//=============================================================================

void Body::addOuterObject(const CkcdObjectShPtr& inOuterObject,
			      bool distanceComputation)

{
#if DEBUG >= 2
  std::string innerName;
  std::string outerName;
  CkppSolidComponentShPtr solidComp =
    KIT_DYNAMIC_PTR_CAST(CkppSolidComponent, inOuterObject);
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
  outerList.push_back(inOuterObject);
  outerObjects(outerList);

  /**
     If distance computation is requested, build necessary CkcdAnalysis
     objects
  */
  if (distanceComputation) {
    /*
      Store object in case inner objects are added a posteriori
    */
    attOuterObjForDist.push_back(inOuterObject);

    /*
      Build distance computation objects (CkcdAnalysis)
    */
    const CkcdObjectShPtr& outerObject=inOuterObject;
    const std::vector<CkcdObjectShPtr> innerList = attInnerObjForDist;
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

#if DEBUG >= 2
      solidComp = KIT_DYNAMIC_PTR_CAST(CkppSolidComponent, innerObject);
      if (solidComp) {
	innerName = solidComp->name();
      } else {
	innerName = std::string("");
      }
#endif
      ODEBUG2(":addInnerObject: creating analysis between "
	      << innerName << " and "
	      << outerName);

      attDistCompPairs.push_back(analysis);
    }
  }
}

//=============================================================================

void Body::resetOuterObjects()
{
  attOuterObjForDist.clear();
  attDistCompPairs.clear();
}


//=============================================================================

ktStatus
Body::distAndPairsOfPoints(unsigned int inPairId,
			       double& outDistance,
			       CkitPoint3& outPointBody,
			       CkitPoint3& outPointEnv,
			       CkcdObjectShPtr &outObjectBody,
			       CkcdObjectShPtr &outObjectEnv,
			       CkcdAnalysisType::Type inType)
{
  KWS_PRECONDITION(inPairId < nbDistPairs());

  CkcdAnalysisShPtr analysis = attDistCompPairs[inPairId];
  analysis->analysisType(inType);

  ktStatus status = analysis->compute();
  if (KD_SUCCEEDED(status)) {
    ODEBUG2(":distAndPairsOfPoints: compute succeeded.");
    unsigned int nbDistances = analysis->countExactDistanceReports();

    if(nbDistances == 0) {
      //no distance information available, return 0 for instance;
      ODEBUG2(":distAndPairsOfPoints: no distance report.");
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
    ODEBUG1(":distAndPairsOfPoints: compute failed.");
    return KD_ERROR;
  }
  return KD_OK;
}
