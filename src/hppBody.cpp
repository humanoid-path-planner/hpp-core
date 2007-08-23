/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)

*/

#include <iostream>
#include "hppBody.h"

#include "KineoKCDModel/kppKCDPolyhedron.h"

//=============================================================================

ChppBodyShPtr ChppBody::create(std::string inName)
{
  ChppBody* hppBody = new ChppBody(inName);
  ChppBodyShPtr hppBodyShPtr(hppBody);
  ChppBodyWkPtr hppBodyWkPtr = hppBodyShPtr;
  
  if (hppBody->init(hppBodyWkPtr) != KD_OK) {
    std::cerr<<"error in ChppBody::create() "<<std::endl;
    hppBodyShPtr.reset();
  }
  return hppBodyShPtr;
}

//=============================================================================

ktStatus ChppBody::init(const ChppBodyWkPtr bodyWkPtr)
{

  m_exact_analyzer = CkcdAnalysis::create();

  // m_exact_analyzer->setAnalysisType(CkcdAnalysis::EXHAUSTIVE_BOOLEAN_COLLISION);
  m_exact_analyzer->analysisType(CkcdAnalysisType::EXACT_DISTANCE);

  return CkwsKCDBody::init(bodyWkPtr);
}

//=============================================================================

std::string ChppBody::name()
{ 
  return bodyName;
}

//=============================================================================

void ChppBody::setInnerObjects (const std::vector< CkcdObjectShPtr > &i_innerObjects, 
				const std::vector< CkitMat4 > &matList)
{
  innerObjects(i_innerObjects, matList);
    
  inner = i_innerObjects;
  //debug
  //std::cout<<"ChppBody::setInnerObjects() called "<<std::endl;
  
  // retrieve the test trees associated with the objects and the ignored object list
  CkcdTestTreeShPtr tree1 = CkcdTestTree::collectTestTrees(i_innerObjects, std::vector< CkcdObjectShPtr >());

  m_exact_analyzer->leftTestTree(tree1);
}

//=============================================================================

void ChppBody::setInnerObjects (const std::vector< CkcdObjectShPtr > &i_innerObjects)
{
  innerObjects(i_innerObjects);
    
  inner = i_innerObjects;
  //debug
  //std::cout<<"ChppBody::setInnerObjects() called "<<std::endl;
  
  // retrieve the test trees associated with the objects and the ignored object list
  CkcdTestTreeShPtr tree1 = CkcdTestTree::collectTestTrees(i_innerObjects, std::vector< CkcdObjectShPtr >());

  m_exact_analyzer->leftTestTree(tree1);
}

//=============================================================================

void ChppBody::setOuterObjects (const std::vector< CkcdObjectShPtr > &i_outerObjects)
{
  outerObjects(i_outerObjects);
  outer = i_outerObjects;

  //debug
  //std::cout<<"ChppBody::setOuterObjects() called "<<std::endl;

  // retrieve the test trees associated with the objects and the ignored object list
  CkcdTestTreeShPtr tree2 = CkcdTestTree::collectTestTrees(i_outerObjects, std::vector< CkcdObjectShPtr >());

  m_exact_analyzer->rightTestTree(tree2);
}

//=============================================================================

void ChppBody::getInnerObjects (std::vector< CkcdObjectShPtr > & list)
{
  list = inner;
}

//=============================================================================

void ChppBody::getOuterObjects (std::vector< CkcdObjectShPtr > & list)
{
  list = outer;
}


//=============================================================================

ktStatus ChppBody::getExactDistance(double &dist, CkitPoint3& o_point1, CkitPoint3& o_point2,
				    CkcdObjectShPtr &object1, CkcdObjectShPtr &object2)
{
  m_exact_analyzer->analysisType(CkcdAnalysisType::EXACT_DISTANCE);

  m_exact_analyzer->compute();
  unsigned int nbDistances = m_exact_analyzer->countExactDistanceReports();
      
  if(nbDistances == 0) {
    //no distance information available, return 0 for instance;
    dist = 0;

    object1.reset(); 
    object2.reset();

    return KD_ERROR;
  } 
  else{

    CkcdExactDistanceReportShPtr distanceReport;
    CkitPoint3 leftPoint, rightPoint;

    //distances are ordered from lowest value, to highest value. 
    distanceReport = m_exact_analyzer->exactDistanceReport(0); 

    //exact distance between two lists is always stored at the first rank of Distance reports.
    dist = distanceReport->distance();

    if(distanceReport->countPairs()>0)
      {
	CkitMat4 firstPos, secondPos;
	// get points in the frame of the object
	distanceReport->getPoints(0,leftPoint,rightPoint) ;
        // get geometry
	object1 = distanceReport->pair(0).first->geometry();
	object2 = distanceReport->pair(0).second->geometry();
	object1->getAbsolutePosition(firstPos);
	object2->getAbsolutePosition(secondPos);
        //from these geometry points we can get points in absolute frame(world) or relative frame(geometry).
	//here we want points in the absolute frame.
	o_point1= firstPos * leftPoint;
	o_point2 = secondPos * rightPoint;
      }
 
    // get object
    object1 = distanceReport->leftCollisionEntity();
    object2 = distanceReport->rightCollisionEntity();
   
    return KD_OK;
  }

}

//=============================================================================

ktStatus ChppBody::getEstimatedDistance(double &dist, 
					CkcdObjectShPtr &object1, CkcdObjectShPtr &object2)
{
  m_exact_analyzer->analysisType(CkcdAnalysisType::ESTIMATED_DISTANCE);
  m_exact_analyzer->compute();

  unsigned int nbDistances = m_exact_analyzer->countEstimatedDistanceReports();
      
  if(nbDistances == 0) {
    //no distance information available, return 0 for instance;
    dist = 0;

    object1.reset(); 
    object2.reset();

    return KD_ERROR;
  } 
  else{

    CkcdEstimatedDistanceReportShPtr distanceReport;
  

    //distances are ordered from lowest value, to highest value. 
    distanceReport = m_exact_analyzer->estimatedDistanceReport(0); 

    //exact distance between two lists is always stored at the first rank of Distance reports.
    dist = distanceReport->distance();

    // no points for estimated distance
    
    // get object
    object1 = distanceReport->leftCollisionEntity();
    object2 = distanceReport->rightCollisionEntity();
  

    return KD_OK;
  }

}

//=============================================================================

bool ChppBody::getCollisionVec(unsigned int &nbCollisions,
			       std::vector<CkcdObjectShPtr> &objectVec1, 
			       std::vector<CkcdObjectShPtr> &objectVec2)
{
  //here we consider that collision lists (CkcdCollisionList) have been given to the analysis

  m_exact_analyzer->analysisType(CkcdAnalysisType::EXHAUSTIVE_BOOLEAN_COLLISION);
  // m_exact_analyzer->analysisType(CkcdAnalysis::FAST_BOOLEAN_COLLISION);

  m_exact_analyzer->compute();
  nbCollisions = m_exact_analyzer->countCollisionReports();

  objectVec1.clear();
  objectVec2.clear();

  if(nbCollisions > 0) {
    //there is at least one collision, get more information on that collision
    CkcdCollisionReportShPtr collisionReport;
    CkcdObjectShPtr object1, object2;

    for(unsigned int i=0; i<nbCollisions; i++){
      
      collisionReport = m_exact_analyzer->collisionReport(i);
        
      //retrieve colliding objects
      object1 = collisionReport->pair(i).first->geometry() ;
      object2 = collisionReport->pair(i).second->geometry() ;
      //here we could obtain more information, like which triangles are colliding.
      //See CkcdPolyhedronTriangle for more information

      objectVec1.push_back(object1);
      objectVec2.push_back(object2);
    }
    return true;
  }

  return false;

}

//=============================================================================

bool ChppBody::getCollision(unsigned int &nbCollisions,
			    CkcdObjectShPtr &object1, CkcdObjectShPtr &object2)
{
  //here we consider that collision lists (CkcdCollisionList) have been given to the analysis

  // m_exact_analyzer->analysisType(CkcdAnalysis::EXHAUSTIVE_BOOLEAN_COLLISION);
  m_exact_analyzer->analysisType(CkcdAnalysisType::FAST_BOOLEAN_COLLISION);
  m_exact_analyzer->compute();
  nbCollisions = m_exact_analyzer->countCollisionReports();

  if(nbCollisions > 0) {
    //there is at least one collision, get more information on that collision
    CkcdCollisionReportShPtr collisionReport;

    collisionReport = m_exact_analyzer->collisionReport(0);

    //retrieve colliding objects
    object1 = collisionReport->pair(0).first->geometry() ;
    object2 = collisionReport->pair(0).second->geometry() ;
    //here we could obtain more information, like which triangles are colliding.
    //See CkcdPolyhedronTriangle for more information

    return true;
  }

  else{
    object1.reset();
    object2.reset();

    return false;
  }
}


//=============================================================================

bool ChppBody::printCollisionStatus(const bool& detailInfoFlag )
{
  unsigned int nbCollisions;
  std::vector<CkcdObjectShPtr> objectVec1, objectVec2;

  if(getCollisionVec(nbCollisions, objectVec1, objectVec2) == true){
    std::cout<<" ------- "<<nbCollisions<<" collision(s) detected.-------- " <<std::endl;
    for(unsigned int i=0; i<nbCollisions; i++){

      CkppPolyhedronShPtr kppPolyhedron1 = KIT_DYNAMIC_PTR_CAST(CkppPolyhedron, objectVec1[i]);
      CkppPolyhedronShPtr kppPolyhedron2 = KIT_DYNAMIC_PTR_CAST(CkppPolyhedron, objectVec2[i]);
      
      std::cout<<kppPolyhedron1->name()<<" and "<<kppPolyhedron2->name()<<std::endl;
    }
    std::cout<<" --------------------------"<<std::endl;
    return true;
  }

  if(detailInfoFlag){

    std::cout<<std::endl<<" no collision detected. Closest objects: ";
    double dist;
    CkitPoint3 o_point1,  o_point2;
    CkcdObjectShPtr object1, object2;
      
    if(getExactDistance(dist, o_point1, o_point2, object1, object2) == KD_OK){

      CkppPolyhedronShPtr kppPolyhedron1 = KIT_DYNAMIC_PTR_CAST(CkppPolyhedron, object1);
      CkppPolyhedronShPtr kppPolyhedron2 = KIT_DYNAMIC_PTR_CAST(CkppPolyhedron, object2);

      std::cout<<std::endl<<" kppPolyhedron "<<kppPolyhedron1->name()<<" and "<<kppPolyhedron2->name()
	       <<", distance "<<dist<<std::endl;
      std::cout<<" at ["<<o_point1[0]<<"] ["<<o_point1[1]<<"] ["<<o_point1[0]<<"] and ["
	       <<o_point2[0]<<"] ["<<o_point2[1]<<"] ["<<o_point2[0]<<"]"<<std::endl;
      
    }
    else{
      std::cout<<"no outerlist registered for getExactDistance"<<std::endl;
    }
  }
  return false;
}

//=============================================================================

void ChppBody::printCollisionStatusFast()
{
  unsigned int nbCollisions;
  CkcdObjectShPtr object1, object2;

  if(getCollision(nbCollisions, object1, object2) == true){
    // collision
    std::cout<<std::endl<<" ------- "<<nbCollisions<<" collision(s) detected.-------- " <<std::endl;

    getCollision(nbCollisions, object1, object2);

    CkppPolyhedronShPtr kppPolyhedron1 = KIT_DYNAMIC_PTR_CAST(CkppPolyhedron, object1);
    CkppPolyhedronShPtr kppPolyhedron2 = KIT_DYNAMIC_PTR_CAST(CkppPolyhedron, object2);

    std::cout<<"here"<<std::endl;
    std::cout<<kppPolyhedron1->name();
    std::cout<<" and "<<kppPolyhedron2->name()<<std::endl;
    std::cout<<" --------------------------"<<std::endl;
  }
  else{
    double dist;
    CkcdObjectShPtr object1, object2;
      
    if(getEstimatedDistance(dist, object1, object2) == KD_OK){

      std::cout<<std::endl<<" no collision detected. Closest objects: ";

      CkppPolyhedronShPtr kppPolyhedron1 = KIT_DYNAMIC_PTR_CAST(CkppPolyhedron, object1);
      CkppPolyhedronShPtr kppPolyhedron2 = KIT_DYNAMIC_PTR_CAST(CkppPolyhedron, object2);

      std::cout<<std::endl<<" kppPolyhedron "<<kppPolyhedron1->name()<<" and "<<kppPolyhedron2->name()
	       <<", distance "<<dist<<std::endl;
      
    }
  }
}


#if 0
//=============================================================================

void ChppBody::mass(double m)
{
  _mass = m;
}

//=============================================================================

double ChppBody::mass()
{
  return _mass;
}

//=============================================================================

void ChppBody::intertia(std::vector<double> i)
{
  _inertia = i;
}
 
//=============================================================================

std::vector<double> ChppBody::inertia()
{
  return _inertia; 
}

//=============================================================================

void ChppBody::relComVec(double x, double y, double z)
{
  _relComVec = CkitVect3(x, y, z);
}

//=============================================================================

void ChppBody::relComVec(const CkitVect3& vec)
{
  _relComVec = CkitVect3(vec);
}

//=============================================================================

const CkitVect3& ChppBody::relComVec() const
{
  return _relComVec;
}


//=============================================================================

ktStatus ChppBody::currentComPos(CkitPoint3 &pos)
{
  CkwsJointShPtr kwsJoint;
  if(!(kwsJoint = joint())){
    std::cerr<<" in ChppBody::currentComPos(): for "<<name()<<", joint not attached. "
	     <<std::endl;
    return KD_ERROR;
  }

  // it should be "point" so that translation is taken into account.
  CkitPoint3 relCom(_relComVec);
  CkitMat4 currentPos = kwsJoint->currentPosition();
  pos = currentPos * relCom;

  return KD_OK;
}

#endif
