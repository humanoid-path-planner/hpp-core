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

#include "hppPolyhedron.h"


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

  void getInnerObjects (std::vector< CkcdObjectShPtr > & list);
  void getOuterObjects (std::vector< CkcdObjectShPtr > & list);


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
 
  /**
     \brief Functions for physical properties : Setting and getting mass.
  */
  void mass(double m);
  double mass();

  /**
   \brief Setting and getting moments of inertia.
  */
  void intertia(std::vector<double> i);
  std::vector<double> inertia();

  /** 
    \brief Setting and getting relative CoM vector.
  */
  void relComVec(double x, double y, double z);
  void relComVec(const CkitVect3& vec);
  const CkitVect3& relComVec() const;

  /**
   \brief Calculate absolute position of CoM from Joint information
  */
  ktStatus currentComPos(CkitPoint3 &pos);

protected:

  ChppBody(std::string inName) : bodyName(inName) {};
  ktStatus init(const ChppBodyWkPtr bodyWkPtr);
  
private:

  std::string bodyName;

  CkcdAnalysisShPtr m_exact_analyzer;
  std::vector< CkcdObjectShPtr > inner, outer;

  /// physical properties
  double _mass;
  std::vector<double> _inertia;
  CkitVect3 _relComVec;
};


#endif /*HPPBODY_H_*/
