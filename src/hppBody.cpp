
/*

  Copyright 2007 LAAS-CNRS
  Author: Florent Lamiraux (LAAS-CNRS)

*/

#include <iostream>

#include "KineoModel/kppSolidComponentRef.h"
#include "KineoModel/kppJointComponent.h"
#include "KineoKCDModel/kppKCDPolyhedron.h"
#include "KineoKCDModel/kppKCDAssembly.h"
#include "kcd2/kcdExactDistanceReport.h"
#include "kcd2/kcdAnalysis.h"
#include "kcd2/kcdGeometrySubElement.h"
#include "KineoWorks2/kwsJoint.h"

#include "hppModel/hppBody.h"

#if DEBUG==2
#undef NDEBUG
#define ODEBUG2(x) std::cout << "ChppBody:" << x << std::endl
#define ODEBUG1(x) std::cerr << "ChppBody:" << x << std::endl
#elif DEBUG==1
#undef NDEBUG
#define ODEBUG2(x)
#define ODEBUG1(x) std::cerr << "ChppBody:" << x << std::endl
#else
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif

//=============================================================================

ChppBodyShPtr ChppBody::create(const std::string& inName)
{
  ChppBody* hppBody = new ChppBody(inName);
  ChppBodyShPtr hppBodyShPtr(hppBody);
  ChppBodyWkPtr hppBodyWkPtr = hppBodyShPtr;
  
  if (hppBody->init(hppBodyWkPtr) != KD_OK) {
    ODEBUG1(" error in create() ");
    hppBodyShPtr.reset();
  }
  return hppBodyShPtr;
}

//=============================================================================

ktStatus ChppBody::init(const ChppBodyWkPtr inBodyWkPtr)
{
  attWeakPtr = inBodyWkPtr;
  return CkwsKCDBody::init(inBodyWkPtr);
}

//=============================================================================

bool 
ChppBody::addInnerObject(const CkppSolidComponentRefShPtr& inSolidComponentRef, 
			 const CkitMat4& inPosition, 
			 bool inDistanceComputation)
{
  CkppSolidComponentShPtr solidComponent = inSolidComponentRef->referencedSolidComponent();
  
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
    ODEBUG1("ChppBody::addSolidComponent: the body is not attached to any joint");
    return false;
  }
  
  /*
    If requested, add the object in the list of objects the distance to which 
    needs to be computed and build the corresponding analyses.
  */
  if (inDistanceComputation) {
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

void ChppBody::addOuterObject(const CkcdObjectShPtr& inOuterObject, 
			      bool inDistanceComputation)

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
  if (inDistanceComputation) {
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

void ChppBody::resetOuterObjects()
{
  attOuterObjForDist.clear();
  attDistCompPairs.clear();
}


//=============================================================================

ktStatus 
ChppBody::distAndPairsOfPoints(unsigned int inPairId,
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
      
      //exact distance between two lists is always stored at the first rank of Distance reports.
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
	//from these geometry points we can get points in absolute frame(world) or relative frame(geometry).
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
