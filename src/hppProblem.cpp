/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)
  and Mathieu Poirier (LAAS-CNRS)
*/


/*****************************************
 INCLUDES
*******************************************/

#include <iostream>

#include "hppCore/hppProblem.h"
#include "hppModel/hppBody.h"

#include "KineoWorks2/kwsValidatorDPCollision.h"

const CkitNotification::TType  ChppProblem::ID_HPP_ADD_PATH ( CkitNotification::makeID() );

const std::string ChppProblem::PATH_KEY ( "path" );
const std::string ChppProblem::PATH_ID_KEY ( "path_id" );
const std::string ChppProblem::DEVICE_KEY ( "device" );


#if DEBUG==3
#define ODEBUG3(x) std::cout << "ChppProblem:" << x << std::endl
#define ODEBUG2(x) std::cout << "ChppProblem:" << x << std::endl
#define ODEBUG1(x) std::cerr << "ChppProblem:" << x << std::endl
#elif DEBUG==2
#define ODEBUG3(x)
#define ODEBUG2(x) std::cout << "ChppProblem:" << x << std::endl
#define ODEBUG1(x) std::cerr << "ChppProblem:" << x << std::endl
#elif DEBUG==1
#define ODEBUG3(x)
#define ODEBUG2(x)
#define ODEBUG1(x) std::cerr << "ChppProblem:" << x << std::endl
#else
#define ODEBUG3(x)
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif

/*****************************************
 PUBLIC METHODS
*******************************************/

// ==========================================================================

ChppProblem::ChppProblem ( CkppDeviceComponentShPtr inRobot, double inPenetration ) :
  attNotificator ( CkitNotificator::defaultNotificator() ),
  attRobot ( inRobot ),
  attPenetration(inPenetration)
{
  /*
    Set the input penetration in the direct path collision validator of the robot
  */
  setPenetration();
  initMapOuter();
}

// ==========================================================================

ChppProblem::ChppProblem (CkppDeviceComponentShPtr inRobot,
                          const std::vector<CkcdObjectShPtr>& inObstacleList, 
			  double inPenetration) :
  attNotificator ( CkitNotificator::defaultNotificator() ),
  attRobot ( inRobot ),
  attPenetration(inPenetration)
{
  /*
    Set the input penetration in the direct path collision validator of the robot
  */
  setPenetration();
  initMapOuter();
  obstacleList ( inObstacleList );
}

// ==========================================================================

ChppProblem::ChppProblem(const ChppProblem& inProblem) :
  attNotificator(inProblem.attNotificator),
  attRobot(inProblem.attRobot),
  attInitConf(inProblem.attInitConf),
  attGoalConf(inProblem.attGoalConf),
  attRoadmap(inProblem.attRoadmap),
  attRoadmapBuilder(inProblem.attRoadmapBuilder),
  attPathOptimizer(inProblem.attPathOptimizer),
  attConfigExtractor(inProblem.attConfigExtractor),
  attObstacleList(inProblem.attObstacleList),
  attPathVector(inProblem.attPathVector),
  attMapOuter(inProblem.attMapOuter),
  attPenetration(inProblem.attPenetration)
{
}

// ==========================================================================

void ChppProblem::initMapOuter()
{
  CkwsDevice::TBodyVector bodyVector;
  attRobot->getBodyVector ( bodyVector );

  // Loop over bodies of robot.
  for ( CkwsDevice::TBodyIterator bodyIter = bodyVector.begin(); bodyIter < bodyVector.end(); bodyIter++ )
    {
      // Try to cast body into CkwsKCDBody
      CkwsKCDBodyShPtr kcdBody;
      if ( kcdBody = boost::dynamic_pointer_cast<CkwsKCDBody> ( *bodyIter ) )
	{
	  // Copy list of outer objects
	  attMapOuter[kcdBody]= kcdBody->outerObjects();
	}
    }
}

// ==========================================================================

CkppDeviceComponentShPtr ChppProblem::getRobot() const
{
  return attRobot;
}

// ==========================================================================

CkwsConfigShPtr ChppProblem::initConfig() const
{
  return attInitConf;
}

// ==========================================================================

ktStatus ChppProblem::initConfig ( CkwsConfigShPtr inConfig )
{
  if (inConfig->device() != attRobot) {
    ODEBUG1(":goalConfig: configuration device does not match problem device.");
    return KD_ERROR;
  }
  attInitConf = inConfig;
  return KD_OK;
}

// ==========================================================================

CkwsConfigShPtr ChppProblem::goalConfig() const
{
  return attGoalConf;
}

// ==========================================================================

ktStatus ChppProblem::goalConfig ( CkwsConfigShPtr inConfig )
{
  if (inConfig->device() != attRobot) {
    ODEBUG1(":goalConfig: configuration device does not match problem device.");
    return KD_ERROR;
  }
  attGoalConf = inConfig;
  return KD_OK;
}

// ==========================================================================

ktStatus 
ChppProblem::obstacleList(const std::vector<CkcdObjectShPtr>& inCollisionList)
{
  for (std::vector<CkcdObjectShPtr>::const_iterator it = 
	 inCollisionList.begin();
       it < inCollisionList.end();
       it++) {
    addObstacle(*it, true);
  }
  return KD_OK;
}

// ==========================================================================

const std::vector<CkcdObjectShPtr>& ChppProblem::obstacleList()
{
  return attObstacleList;
}

// ==========================================================================

ktStatus ChppProblem::addObstacle(const CkcdObjectShPtr& inObject, 
				  bool inDistanceComputation)
{
  // Add object in local list
  attObstacleList.push_back(inObject);

  // Get robot vector of bodies.
  CkwsDevice::TBodyVector bodyVector;
  attRobot->getBodyVector ( bodyVector );

  // Loop over bodies of robot.
  for ( CkwsDevice::TBodyIterator bodyIter = bodyVector.begin(); 
	bodyIter < bodyVector.end(); 
	bodyIter++ ) {

    if (ChppBodyShPtr hppBody = KIT_DYNAMIC_PTR_CAST(ChppBody, *bodyIter)) {
      ODEBUG2(":addOuterObject: ChppBody type.");
      hppBody->addOuterObject(inObject, inDistanceComputation);
    }
    else if (CkwsKCDBodyShPtr kcdBody = 
	     KIT_DYNAMIC_PTR_CAST(CkwsKCDBody, *bodyIter)) {
      ODEBUG2(":addOuterObject: CkwsKCDBody type.");
      /*
	Append object at the end of KineoWorks set of outer objects
	for collision checking
      */
      std::vector<CkcdObjectShPtr> outerList = kcdBody->outerObjects();
      outerList.push_back(inObject);
      kcdBody->outerObjects(outerList);
    }
  }
  return KD_OK;
}

// ==========================================================================

void ChppProblem::steeringMethod ( const CkwsSteeringMethodShPtr &inSteeringMethod )
{
  attRobot->steeringMethod ( inSteeringMethod );
}

// ==========================================================================

CkwsSteeringMethodShPtr ChppProblem::steeringMethod() const
{
  return attRobot->steeringMethod();
}


// ==========================================================================

void ChppProblem::addPath ( CkwsPathShPtr kwsPath )
{
  attPathVector.push_back ( kwsPath );
  std::string robotName = attRobot->name();

  // attNotificator->sendNotification(ID_HPP_ADD_PATH, this);
  CkitNotificationShPtr notification
    = CkitNotification::createWithPtr<ChppProblem> ( ChppProblem::ID_HPP_ADD_PATH, this );
  // set attribute to retreave
  notification->shPtrValue<CkwsPath> ( PATH_KEY, kwsPath );
  notification->shPtrValue<CkppDeviceComponent> ( DEVICE_KEY, attRobot );
  // set path number
  notification->unsignedIntValue ( PATH_ID_KEY, attPathVector.size() );
  attNotificator->notify ( notification );


}

// ==========================================================================

CkwsPathShPtr ChppProblem::getIthPath ( unsigned int pathId ) const
{
  CkwsPathShPtr resultPath;

  if ( pathId < attPathVector.size() )
    {
      resultPath = attPathVector[pathId];
    }
  return resultPath;
}

// ==========================================================================

unsigned int ChppProblem::getNbPaths() const
{
  return attPathVector.size();
}


// ==========================================================================

void ChppProblem::roadmapBuilder ( CkwsRoadmapBuilderShPtr inRoadmapBuilder )
{

  attRoadmapBuilder = inRoadmapBuilder;
}

// ==========================================================================

CkwsRoadmapBuilderShPtr ChppProblem::roadmapBuilder()
{

  return attRoadmapBuilder ;
}

// ==========================================================================

CkwsRoadmapShPtr ChppProblem::roadmap() const
{

  return attRoadmap ;
}

// ==========================================================================

void ChppProblem::roadmap ( CkwsRoadmapShPtr inRoadmap )
{

  attRoadmap = inRoadmap ;
}

// ==========================================================================

void ChppProblem::pathOptimizer ( CkwsPathOptimizerShPtr inOptimizer )
{

  attPathOptimizer = inOptimizer ;
}

// ==========================================================================

CkwsPathOptimizerShPtr ChppProblem::pathOptimizer()
{
  return attPathOptimizer ;
}

// ==========================================================================

void ChppProblem::configExtractor(const CkwsConfigExtractorShPtr& inConfigExtractor)
{
  attConfigExtractor = inConfigExtractor;
}

// ==========================================================================

const CkwsConfigExtractorShPtr& ChppProblem::configExtractor()
{
  return attConfigExtractor;
}

// ==========================================================================

void ChppProblem::penetration(double inPenetration)
{
  attPenetration = inPenetration;
  setPenetration();
}

// ==========================================================================

double ChppProblem::penetration() const
{ 
  return attPenetration;
}

void ChppProblem::setPenetration()
{
  CkwsDirectPathValidatorSetConstShPtr dpValidators = 
    attRobot->directPathValidators();

  /*
    Retrieve collision validator if any and set penetration as penetration 
    distance of roadmap builder.
  */
  CkwsValidatorDPCollisionShPtr collisionValidator = 
    dpValidators->retrieve<CkwsValidatorDPCollision>();
  if (collisionValidator) {
    collisionValidator->penetration(attPenetration);
  }
}
