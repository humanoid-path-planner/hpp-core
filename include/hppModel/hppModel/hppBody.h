/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)

*/

#ifndef HPPBODY_H_
#define HPPBODY_H_

/*************************************
INCLUDE
**************************************/

#include "KineoUtility/kitDefine.h"
#include "kcd2/kcdAnalysisType.h"
#include "kwsKcd2/kwsKCDBody.h"

KIT_PREDEF_CLASS(ChppBody);
KIT_PREDEF_CLASS(CkcdObject);
KIT_PREDEF_CLASS(CkppSolidComponentRef);

class CkitMat4;

/*************************************
CLASS
**************************************/
/**
 \brief This class represents bodies (geometric objects attached to a joint).

 It derives from KineoWorks CkwsKCDBody class and from an implementation of 
 CjrlJoint.

 Objects attached to a body (called inner objects) are used for collision
 checking with selected objects of the environment (called outer objects).

 To attach an object to the body, call addInnerObject(). To select an object
 for collision checking with the body, call addOuterObject().

 Distances between pairs of inner objects and outer objects can also
 be computed. Setting <code>inDistanceComputation</code> to true in
 addInnerObject() or addOuterObject() specifies that distances should
 be computed for these objects. Each pair of such specified (inner,
 outer) objects gives rise to one distance computation when calling
 distAndPairsOfPoints(). The number of such pairs can be retrieved by
 calling nbDistPairs(). distAndPairsOfPoints() also returns distances
 and pairs of closest points for each computed pair.

 The constructor is protected and method create returns a shared
 pointer to the device.

 \sa Smart pointers documentation:
 http://www.boost.org/libs/smart_ptr/smart_ptr.htm
*/

class ChppBody : public CkwsKCDBody
{
public:
  /**
     \brief Creation of a body
     \param inName Name of the new body.
     \return A shared pointer to a new body.
  */
  static ChppBodyShPtr create(const std::string& inName);

  /**
     \brief Get name of object.
  */
  const std::string& name() {return attName;};

  /**
     \name Define inner and outer objects
     @{
  */
  /**
     \brief Add a geometric object to the body

     \param inSolidComponentRef Reference to the solid component to add.
     \param inPosition Position of the object before attaching it to the body
     (default value=Identity).
     \param inDistanceComputation whether this object should be put in the
     distance computation analysis.

     \return true if success, false otherwise.

     The object is added to the inner object list of the body.

     \note The body must be attached to a joint.
  */
  bool addInnerObject(const CkppSolidComponentRefShPtr& inSolidComponentRef,
		      const CkitMat4& inPosition=CkitMat4(),
		      bool inDistanceComputation=false);

  /**
     \brief Add an object for collision testing with the body

     \param inOuterObject new object
     \param inDistanceComputation whether distance analyses should be added for
     this object.
  */

  void	addOuterObject(const CkcdObjectShPtr& inOuterObject,
		       bool inDistanceComputation=true);

  /**
     \brief Reset the list of outer objects
  */
  void resetOuterObjects();

  /**
     @}
  */

  /**
     \name Distance computation
     @{
  */


  /**
     \brief Get the number of pairs of object for which distance is computed
  */
  inline unsigned int nbDistPairs() { return attDistCompPairs.size(); };

  /**
     \brief Compute exact distance and closest points between body and set of outer objects.

     \param inPairId id of the pair of objects
     \param inType Type of distance computation
     (either CkcdAnalysisType::EXACT_DISTANCE or
     CkcdAnalysisType::ESTIMATED_DISTANCE)

     \retval outDistance Distance between body and outer objects
     \retval outPointBody Closest point on body (in global reference frame)
     \retval outPointEnv Closest point in outer object set (in global reference frame)
     \retval outObjectBody Closest object on body
     \retval outObjectEnv Closest object in outer object list
  */
  ktStatus distAndPairsOfPoints(unsigned int inPairId,
				double& outDistance,
				CkitPoint3& outPointBody,
				CkitPoint3& outPointEnv,
				CkcdObjectShPtr &outObjectBody,
				CkcdObjectShPtr &outObjectEnv,
				CkcdAnalysisType::Type inType=
				CkcdAnalysisType::EXACT_DISTANCE);


  /**
     @}
  */

protected:

  /**
     \brief Constructor by name.
  */
  ChppBody(const std::string& inName): attName(inName) {};

  /**
     \brief Initialization of body

     \param inBodyWkPtr weak pointer to itself
  */
  ktStatus init(const ChppBodyWkPtr inBodyWkPtr);

private:

  /**
     \brief Name of the body.
  */
  std::string attName;

  /**
     \brief Set of inner objects for which distance computation is performed
  */
  std::vector<CkcdObjectShPtr> attInnerObjForDist;

  /**
     \brief Set of outer objects for which distance computation is performed
  */
  std::vector<CkcdObjectShPtr> attOuterObjForDist;

  /**
     \brief Collision analyses for this body

     Each pair (inner object, outer object) potentially defines an exact
     distance analysis. Only inner objects specified in attDistanceObjects
     define analyses.
  */
  std::vector<CkcdAnalysisShPtr> attDistCompPairs;

  /**
     \brief Weak pointer to itself
  */
  ChppBodyWkPtr attWeakPtr;
};


#endif /*HPPBODY_H_*/
