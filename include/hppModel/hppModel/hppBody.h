/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)

*/

#ifndef HPPBODY_H_
#define HPPBODY_H_

/*************************************
INCLUDE
**************************************/

#include "KineoWorks2/kwsInterface.h"
#include "KineoUtility/kitDefine.h"
#include "KineoUtility/kitInterface.h"
#include "kcd2/kcdInterface.h"
#include "kwsKcd2/kwsKCDBody.h"
#include "KineoModel/kppSolidComponentRef.h"

KIT_PREDEF_CLASS(ChppBody);


/*************************************
CLASS
**************************************/
/**
 \brief This class represents bodies (solid components attached to a joint). It derives from 
 KineoWorks CkwsKCDBody class. In order to store more information, we have derived CkwsKCDBody class. 
 A name is given at the creation of the body (ChppBody::create(std::string inName)). 
 The constructor is protected and method create returns a shared pointer to the device.
\sa Smart pointers documentation: http://www.boost.org/libs/smart_ptr/smart_ptr.htm 
*/

class ChppBody : public CkwsKCDBody
{
public:
  std::string name();
  static ChppBodyShPtr create(std::string inName);

  
  void 	setInnerObjects (const std::vector< CkcdObjectShPtr > &i_innerObjects);
  void 	setInnerObjects (const std::vector< CkcdObjectShPtr > &i_innerObjects,  
				const std::vector< CkitMat4 > &matList);

  void 	setOuterObjects (const std::vector< CkcdObjectShPtr > &i_outerObjects);

  /**
     \brief Add geometry to the body
     
     \param inSolidComponentRef Reference to the solid component to add.
     \return true if success, false otherwise.

     The input solid component is dynamically cast into
     \li a CkppKCDPolyhedron or 
     \li a CkppKCDAssembly
     The object is then added to the inner object list of the body.
     The collision analyser attExactAnalyzer is also updated.

     \note The body must be attached to a joint.
  */
  bool addSolidComponent(const CkppSolidComponentRefShPtr& inSolidComponentRef);

  ktStatus getExactDistance(double &dist, CkitPoint3& o_point1, CkitPoint3& o_point2,
			    CkcdObjectShPtr &object1, CkcdObjectShPtr &object2);
  ktStatus getEstimatedDistance(double &dist, 
				CkcdObjectShPtr &object1, CkcdObjectShPtr &object2);
   
  bool getCollisionVec(unsigned int &nbCollisions, std::vector<CkcdObjectShPtr> &objectVec1, 
		       std::vector<CkcdObjectShPtr> &objectVec2);
  bool getCollision(unsigned int &nbCollisions,
		    CkcdObjectShPtr &object1, CkcdObjectShPtr &object2);

  bool printCollisionStatus(const bool& detailInfoFlag = false);
  void printCollisionStatusFast();
 
protected:

  ChppBody(std::string inName) : bodyName(inName) {};
  ktStatus init(const ChppBodyWkPtr bodyWkPtr);
  
private:

  std::string bodyName;

  CkcdAnalysisShPtr m_exact_analyzer;
  std::vector< CkcdObjectShPtr > inner, outer;
};


#endif /*HPPBODY_H_*/
