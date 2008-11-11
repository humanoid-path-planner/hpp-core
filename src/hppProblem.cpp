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

ktStatus ChppProblem::obstacleList ( const std::vector<CkcdObjectShPtr>& inCollisionList )
{
	// Copy collision list in problem.
	attObstacleList = inCollisionList;

	// For each obstacle,
	//   - insert the obstacle in outer object of each body of the robot.

	// Loop over object in the collision list.
	// Get robot vector of bodies.
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
			std::vector<CkcdObjectShPtr> collisionList = attMapOuter[kcdBody];
			unsigned int nObjects = attObstacleList.size();
			for ( unsigned int iObject=0; iObject<nObjects; iObject++ )
			{
				CkcdObjectShPtr kcdObject = attObstacleList[iObject];
				// Add object to list of outer objects
				collisionList.push_back ( kcdObject );
			}
			// Set updated list as new list of outer objects
			kcdBody->outerObjects ( collisionList );
		}
		else
		{
			std::cout << "CcppProblem::obstacleList: body is not KCD body. Obstacle not inserted." << std::endl;
		}
	}

//----------------------------------
	/*
		// Copy collision list in problem.
		attObstacleList = inCollisionList;

		// For each obstacle,
		//   - insert the obstacle in outer object of each body of the robot.

		// Loop over object in the collision list.
		// Get robot vector of bodies.
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
				std::vector<CkcdObjectShPtr> collisionList = kcdBody->outerObjects();
				unsigned int nObjects = attObstacleList.size();
				for ( unsigned int iObject=0; iObject<nObjects; iObject++ )
				{
					CkcdObjectShPtr kcdObject = attObstacleList[iObject];
					// Add object to list of outer objects
					collisionList.push_back ( kcdObject );
				}
				// Set updated list as new list of outer objects
				kcdBody->outerObjects ( collisionList );
			}
			else
			{
				std::cout << "CcppProblem::obstacleList: body is not KCD body. Obstacle not inserted." << std::endl;
			}
		}
	*/
//----------------------------------
	return KD_OK;
}

// ==========================================================================

const std::vector<CkcdObjectShPtr>& ChppProblem::obstacleList()
{
	return attObstacleList;
}

// ==========================================================================

ktStatus ChppProblem::addObstacle ( const CkcdObjectShPtr& inObject )
{
	// Get robot vector of bodies.
	CkwsDevice::TBodyVector bodyVector;
	attRobot->getBodyVector ( bodyVector );

	// Loop over bodies of robot.
	for ( CkwsDevice::TBodyIterator bodyIter = bodyVector.begin(); bodyIter < bodyVector.end(); bodyIter++ )
	{
		// Try to cast body into CkwsKCDBody
		CkwsKCDBodyShPtr kcdBody;
		ChppBodyShPtr hppBody;
		if ( kcdBody = boost::dynamic_pointer_cast<CkwsKCDBody> ( *bodyIter ) )
		{
			std::vector< CkcdObjectShPtr > collisionList = kcdBody->outerObjects();
			collisionList.push_back ( inObject );

			if ( hppBody = boost::dynamic_pointer_cast<ChppBody> ( kcdBody ) )
			{
				hppBody->setOuterObjects ( collisionList );
			}
			else
				kcdBody->outerObjects ( collisionList );

		}
		else
		{
			std::cout << "ChppProblem::addObstacle: body is not KCD body. Obstacle is not inserted." << std::endl;
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

void ChppProblem::configExtractor(const CkwsConfigExtractorShPtr& inConfigExtractor)
{
  attConfigExtractor = inConfigExtractor;
}

const CkwsConfigExtractorShPtr& ChppProblem::configExtractor()
{
  return attConfigExtractor;
}

void ChppProblem::penetration(double inPenetration)
{
  attPenetration = inPenetration;
}

double ChppProblem::penetration() const
{ 
  return attPenetration;
}
