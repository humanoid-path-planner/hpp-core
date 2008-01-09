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
  /**
     \brief Creation of a body
     \param inName Name of the new body.
     \return A shared pointer to a new body.
  */
  static ChppBodyShPtr create(std::string inName);

  /**
     \brief Get name of object.
  */
  const std::string& name() {return attName;};

  /**
     \name Collision lists
     @{
  */
  /**
     \brief Attach objects to the body.
     \param inInnerObjects list of objects to attach to the body

     Previous objects if any are detached. 

     Objects are put in the left test tree of attExactAnalyzer for exact distance computation.
  */

  void 	setInnerObjects (const std::vector< CkcdObjectShPtr > &inInnerObjects);

  /**
     \brief Attach objects to the body in specified position
     \param inInnerObjects list of objects to attach to the body
     \param inPlacementVector Vector of homogeneous matrix specifying the position of each object in inInnerObjects.

     Previous objects if any are detached. 

     Objects are put in the left test tree of attExactAnalyzer for exact distance computation.
  */

  void 	setInnerObjects (const std::vector< CkcdObjectShPtr > &inInnerObjects, 
			 const std::vector< CkitMat4 > &inPlacementVector);


  /**
     \brief Defines the list of objects to be tested for collision with this body.
     \param inOuterObjects list of objects to be tested for collision for this body

     Previous objects if any are removed. 

     Objects are put in the right test tree of attExactAnalyzer for exact distance computation.
  */

  void 	setOuterObjects (const std::vector< CkcdObjectShPtr > &inOuterObjects);

  /**
     @}
  */
  /**
     \brief Add geometry to the body
     
     \param inSolidComponentRef Reference to the solid component to add.
     \param inPosition Position of the object before attaching it to the body (default value=Identity).
     \return true if success, false otherwise.

     The input solid component is dynamically cast into
     \li a CkppKCDPolyhedron or 
     \li a CkppKCDAssembly
     The object is then added to the inner object list of the body.
     The collision analyser attExactAnalyzer is also updated.

     \note The body must be attached to a joint.
  */
  bool addSolidComponent(const CkppSolidComponentRefShPtr& inSolidComponentRef, const CkitMat4& inPosition=CkitMat4());

  /**
     \name Collision and distance computation
     @{
  */

  /**
     \brief Compute exact distance and closest points between body and set of outer objects.

     \retval outDistance Distance between body and outer objects
     \retval outPointBody Closest point on body
     \retval outPointEnv Closest point in outer object set
     \retval outObjectBody Closest object on body
     \retval outObjectEnv Closest object in outer object list
  */
  ktStatus getExactDistance(double& outDistance, CkitPoint3& outPointBody, CkitPoint3& outPointEnv,
			    CkcdObjectShPtr &outObjectBody, CkcdObjectShPtr &outObjectEnv);


  /**
     \brief Compute a lower bound of distance between body and set of outer objects.

     \retval outDistance Distance between body and outer objects
     \retval outPointBody Closest point on body
     \retval outPointEnv Closest point in outer object set
     \retval outObjectBody Closest object on body
     \retval outObjectEnv Closest object in outer object list
  */
  ktStatus getEstimatedDistance(double &outDistance, 
				CkcdObjectShPtr &outObjectBody, CkcdObjectShPtr &outObjectEnv);
   
  
  /**
     \brief Compute the set of inner and outer objects that are in collision with each other.

     \retval outNbCollision Number of pairs of objects in collision.
     \retval outObjectBodyVector Vector of objects in collision of the body.
     \retval outObjectEnvVector Vector of objects in collision in the outer object list.

     \return Whether there is a collision.
  */
  bool getCollisionVec(unsigned int &outNbCollision, std::vector<CkcdObjectShPtr>& outObjectBodyVector, 
		       std::vector<CkcdObjectShPtr>& outObjectEnvVector);

  /**
     \brief Compute collision and return the two first objects found in collision.

     \retval outNbCollision Number of pairs of object in collision found.
     \retval outObjectBody First object of the body found in collision.
     \retval outObjectEnv First object in the body outer list found in collision.

     \return Whether there is a collision.
  */
  bool getCollision(unsigned int& outNbCollision,
		    CkcdObjectShPtr &outObjectBody, CkcdObjectShPtr &outObjectEnv);

  bool printCollisionStatus(const bool& detailInfoFlag = false);
  void printCollisionStatusFast();

  /**
     \brief Compute the minimum distance to the obstacle

     \return the minimum distance to the obstacle
  */
  double getMinDistance();

  /**
     @}
  */
protected:

  /**
     \brief Constructor by name.
  */
  ChppBody(std::string inName) : attName(inName) {};

  /**
     \brief Initialization of body
  */
  ktStatus init(const ChppBodyWkPtr bodyWkPtr);
  
private:

  /**
     \brief Name of the body.
  */
  std::string attName;

  /**
     \brief Collision analyser for this body
  */
  CkcdAnalysisShPtr attExactAnalyzer;
};


#endif /*HPPBODY_H_*/
