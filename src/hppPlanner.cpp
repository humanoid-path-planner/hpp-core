/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
 
  Developed by Florent Lamiraux (LAAS-CNRS)
  and Mathieu Poirier (LAAS-CNRS)
*/


// TODO : nettoyer les reste de la classe hppProblem


/*****************************************
 INCLUDES
*******************************************/

#include <iostream>
#include <fstream>

#include "KineoModel/kppDeviceComponent.h"

#include "hppCore/hppPlanner.h"
#include "hppCore/hppProblem.h"
#include "hppModel/hppBody.h"

#include "KineoUtility/kitNotificator.h"

const CkitNotification::TType ChppPlanner::ID_HPP_ADD_ROBOT(CkitNotification::makeID());
const CkitNotification::TType ChppPlanner::ID_HPP_SET_CURRENT_CONFIG(CkitNotification::makeID());
const CkitNotification::TType ChppPlanner::ID_HPP_REMOVE_OBSTACLES(CkitNotification::makeID());
const CkitNotification::TType ChppPlanner::ID_HPP_SET_OBSTACLE_LIST(CkitNotification::makeID());
const CkitNotification::TType ChppPlanner::ID_HPP_ADD_OBSTACLE(CkitNotification::makeID());

const std::string ChppPlanner::ROBOT_KEY("robot");
const std::string ChppPlanner::OBSTACLE_KEY("obstacle");
const std::string ChppPlanner::CONFIG_KEY("config");

/*****************************************
 PUBLIC METHODS
*******************************************/

// ==========================================================================

ChppPlanner::ChppPlanner()
{
  attNotificator = CkitNotificator::defaultNotificator(); 
  mObstacleList.clear();
}


// ==========================================================================


ChppPlanner::~ChppPlanner()
{
  // delete hppNotificator;
}

// ==========================================================================

ktStatus ChppPlanner::addHppProblem(CkppDeviceComponentShPtr robot)
{
  ChppProblem hppProblem(robot, mObstacleList);

  cout<<"adding a problem in vector"<<endl;
  // Add robot in vector .
  hppProblemVector.push_back(hppProblem);

 
  CkitNotificationShPtr notification 
    = CkitNotification::createWithPtr<ChppPlanner>(ChppPlanner::ID_HPP_ADD_ROBOT, this);
  // set attribute if necessary
  notification->shPtrValue<CkppDeviceComponent>(ROBOT_KEY, robot);
  attNotificator->notify(notification);


  return KD_OK;
}


// ==========================================================================

ktStatus ChppPlanner::addHppProblemAtBeginning(CkppDeviceComponentShPtr robot)
{
  ChppProblem hppProblem(robot, mObstacleList);

  cout<<"adding a problem at beginning of vector"<<endl;
  // Add robot in vector .
  hppProblemVector.push_front(hppProblem);

  CkitNotificationShPtr notification  = CkitNotification::createWithPtr<ChppPlanner>(ChppPlanner::ID_HPP_ADD_ROBOT, this);
  // set attribute if necessary
  notification->shPtrValue<CkppDeviceComponent>(ROBOT_KEY, robot);
  attNotificator->notify(notification);


  return KD_OK;
}


// ==========================================================================

const CkppDeviceComponentShPtr ChppPlanner::robotIthProblem(unsigned int rank) const
{
  CkppDeviceComponentShPtr nullShPtr;

  if (rank < getNbHppProblems()) {
    return hppProblemVector[rank].getRobot();
  }
  return nullShPtr;
}

// ==========================================================================

CkwsConfigShPtr ChppPlanner::robotCurrentConfIthProblem(unsigned int rank) const
{
  ktStatus status = KD_ERROR;
  CkwsConfigShPtr outConfig;

  if (rank < getNbHppProblems()) {
    const CkppDeviceComponentShPtr robot = robotIthProblem(rank);
    CkwsConfig config(robot);
    status = robot->getCurrentConfig(config);
    if (status == KD_OK) {
      outConfig = CkwsConfig::create(config);
    }
  }
  return outConfig;
}

// ==========================================================================

ktStatus ChppPlanner::robotCurrentConfIthProblem(unsigned int rank,
						 const CkwsConfigShPtr& config)
{
  ktStatus status = KD_ERROR;

  if (rank < getNbHppProblems()) {
    // status = hppProblemVector[rank].getRobot()->setCurrentConfig(config);
    // upadate it to use kppDeviceComponent
    status = hppProblemVector[rank].getRobot()->applyCurrentConfig(config);
  }

  /* send notification */
  // I think there is no need to notify because CkppDeviceComponent::applyCurrentConfig()
  // will reflect the configuration change. To be tested.
  if (status == KD_OK) {
    CkitNotificationShPtr notification 
      = CkitNotification::createWithPtr<ChppPlanner>(ChppPlanner::ID_HPP_SET_CURRENT_CONFIG, this);
    notification->shPtrValue<CkwsConfig>(CONFIG_KEY, config);
    attNotificator->notify(notification);

    // status = hppNotificator->sendNotification(ID_HPP_SET_CURRENT_CONFIG);
  }
	
  return status;
}

// ==========================================================================

ktStatus ChppPlanner::robotCurrentConfIthProblem(unsigned int rank,
						 const CkwsConfig& config)
{
  ktStatus status = KD_ERROR;

  if (rank < getNbHppProblems()) {
    status = hppProblemVector[rank].getRobot()->setCurrentConfig(config);
  }


  // If success, send notification
  /* send notification */
  if (status == KD_OK) {
    // status = hppNotificator->sendNotification(ID_HPP_SET_CURRENT_CONFIG);
    CkitNotificationShPtr notification 
      = CkitNotification::createWithPtr<ChppPlanner>(ChppPlanner::ID_HPP_SET_CURRENT_CONFIG, this);
    attNotificator->notify(notification);
    // set attribute if necessary

  }
	
  return status;
}

// ==========================================================================

CkwsConfigShPtr ChppPlanner::initConfIthProblem(unsigned int rank) const
{
  CkwsConfigShPtr config;

  if (rank < getNbHppProblems()) {
    config = hppProblemVector[rank].initConfig();
  }

  return config;
}

// ==========================================================================

ktStatus ChppPlanner::initConfIthProblem(unsigned int rank,
					 const CkwsConfigShPtr config)
{
  ktStatus status = KD_ERROR;

  if (rank < getNbHppProblems()) {
    hppProblemVector[rank].initConfig(config);
    status = KD_OK;
  }

  return status;
}

// ==========================================================================

CkwsConfigShPtr ChppPlanner::goalConfIthProblem(unsigned int rank) const
{
  CkwsConfigShPtr config;

  if (rank < getNbHppProblems()) {
    config = hppProblemVector[rank].goalConfig();
  }

  return config;
}

// ==========================================================================


ktStatus ChppPlanner::roadmapBuilderIthProblem(unsigned int rank,
					       CkwsRoadmapBuilderShPtr inRoadmapBuilder)
{
  if (rank >= getNbHppProblems()) {
    return KD_ERROR;
  }

  ChppProblem& hppProblem =  hppProblemVector[rank];
  hppProblem.roadmapBuilder(inRoadmapBuilder);

  return KD_OK;
}

// ==========================================================================

CkwsRoadmapBuilderShPtr ChppPlanner::roadmapBuilderIthProblem(unsigned int rank)
{
  CkwsRoadmapBuilderShPtr roadmapBuilder;
  if (rank < getNbHppProblems()) {
    ChppProblem& hppProblem =  hppProblemVector[rank];
    roadmapBuilder = hppProblem.roadmapBuilder();
  }
  return roadmapBuilder;
}

// ==========================================================================

ktStatus ChppPlanner::pathOptimizerIthProblem(unsigned int rank,
					      CkwsPathOptimizerShPtr inPathOptimizer)
{
  if (rank >= getNbHppProblems()) {
    return KD_ERROR;
  }

  ChppProblem& hppProblem =  hppProblemVector[rank];
  hppProblem.pathOptimizer(inPathOptimizer);

  return KD_OK;
}


// ==========================================================================

CkwsPathOptimizerShPtr ChppPlanner::pathOptimizerIthProblem(unsigned int rank)
{
  CkwsPathOptimizerShPtr pathOptimizer;
  if (rank < getNbHppProblems()) {
    ChppProblem& hppProblem =  hppProblemVector[rank];
    pathOptimizer = hppProblem.pathOptimizer();
  }
  return pathOptimizer;
}

// ==========================================================================


ktStatus ChppPlanner::steeringMethodIthProblem(unsigned int rank, CkwsSteeringMethodShPtr inSM)
{

  if (rank >= getNbHppProblems()) {
    return KD_ERROR;
  }

  hppProblemVector[rank].getRobot()->steeringMethod(inSM) ;

  return KD_OK ;
}

// ==========================================================================

CkwsSteeringMethodShPtr ChppPlanner::steeringMethodIthProblem(unsigned int rank)
{

  CkwsSteeringMethodShPtr inSM ;
  if (rank < getNbHppProblems()) {
    inSM = hppProblemVector[rank].getRobot()->steeringMethod() ;
  }
  return inSM ;

}

// ==========================================================================

ktStatus ChppPlanner::goalConfIthProblem(unsigned int rank,
					 const CkwsConfigShPtr config)

{
  ktStatus status = KD_ERROR;

  if (rank < getNbHppProblems()) {
    hppProblemVector[rank].goalConfig(config);
    status = KD_OK;
  }

  return status;
}

// ==========================================================================

ktStatus ChppPlanner::obstacleList(std::vector<CkcdObjectShPtr> collisionList)
{
  // Send notification to destroy current obstacles in Kpp.
 
  CkitNotificationShPtr notification  = CkitNotification::createWithPtr<ChppPlanner>(ChppPlanner::ID_HPP_REMOVE_OBSTACLES, this);
  attNotificator->notify(notification);

  // Set list of obstacles.
  mObstacleList = collisionList;

  // Set the list of obstacles for each robot.
  unsigned int nProblem = getNbHppProblems();
  for (unsigned int iProblem=0; iProblem<nProblem; iProblem++) {
    ChppProblem& problem = hppProblemVector[iProblem];
    problem.obstacleList(collisionList);
  }

  // Send notification for new list of obstacles.
  notification = CkitNotification::createWithPtr<ChppPlanner>(ChppPlanner::ID_HPP_SET_OBSTACLE_LIST, this);
  // set attributes if necessary
  notification->ptrValue< std::vector<CkcdObjectShPtr> >(OBSTACLE_KEY, &mObstacleList);
  attNotificator->notify(notification);
	
  return KD_OK;
}

// ==========================================================================

const std::vector< CkcdObjectShPtr > ChppPlanner::obstacleList()
{
  return mObstacleList;
}

// ==========================================================================

ktStatus ChppPlanner::addObstacle(CkcdObjectShPtr object)
{
  mObstacleList.push_back(object);

  // Set the list of obstacles for each robot.
  unsigned int nProblem = getNbHppProblems();
  for (unsigned int iProblem=0; iProblem<nProblem; iProblem++) {
    ChppProblem& problem = hppProblemVector[iProblem];
    problem.addObstacle(object);
  }

  // Send notification
  CkitNotificationShPtr notification = CkitNotification::createWithPtr<ChppPlanner>(ChppPlanner::ID_HPP_ADD_OBSTACLE, this);
  // set attributes if necessary
  notification->ptrValue< std::vector<CkcdObjectShPtr> >(OBSTACLE_KEY, &mObstacleList);
  attNotificator->notify(notification);

  return KD_OK;
}

// ==========================================================================

ktStatus ChppPlanner::solveOneProblem(unsigned int problemId)
{

  //TODO : rajouter le extraDOf - essayer les precondition des SM ou dP -- FAIT mais pas notifier !!
  if (problemId >= getNbHppProblems()) {
    cerr << "ChppPlanner::solveOneProblem: problem Id="
	 << problemId << "is bigger than vector size="
	 << getNbHppProblems();

    return KD_ERROR;
  }

  ChppProblem& hppProblem = hppProblemVector[problemId];
  CkwsPathShPtr kwsPath;

  CkppDeviceComponentShPtr hppDevice = hppProblem.getRobot();
  if (!hppDevice)
    return KD_ERROR ;

  CkwsConfigShPtr initConfig = hppProblem.initConfig() ;
  if (!initConfig)
    return KD_ERROR ;
  CkwsConfigShPtr goalConfig = hppProblem.goalConfig() ;
  if (!goalConfig)
    return KD_ERROR ;

  // solve the problem with the roadmapBuilder
  if ( initConfig && goalConfig && hppProblem.roadmapBuilder() ) {

    if(KD_OK == hppProblem.roadmapBuilder()->solveProblem( *initConfig , *goalConfig , kwsPath)) {
      cout << "--- Problem solved.----" << endl;
    } else {
      cout << "---- Problem NOT solved.----" << endl ;
      return KD_ERROR;
    }

    // optimizer for the path
    if (hppProblem.pathOptimizer()) {
      hppProblem.pathOptimizer()->optimizePath(kwsPath, hppProblem.roadmapBuilder()->penetration());

      cout << "ChppPlanner::solveOneProblem: path optimized with penetration " 
	   << hppProblem.roadmapBuilder()->penetration()<<endl;

    } else {
      cerr << " no Optimizer Defined " << endl ;
    }

    cout<<" number of direct path: "<<kwsPath->countDirectPaths()<<endl;

    // Add the path to vector of paths of the problem.
    hppProblem.addPath(kwsPath);
  } else {
    cerr << "  ChppPlanner::solveOneProblem: problem ill-defined:initConfig, GoalConfig or RoadmapBuilder not initialised" << endl ;
    return KD_ERROR ;
  }

  return KD_OK ;
}

ktStatus ChppPlanner::optimizePath(unsigned int inProblemId, unsigned int inPathId)
{
  if (inProblemId >= getNbHppProblems()) {
    cerr << "ChppPlanner::optimizePath: problem Id="
	 << inProblemId << " is bigger than vector size="
	 << getNbHppProblems();

    return KD_ERROR;
  }

  ChppProblem& hppProblem = hppProblemVector[inProblemId];

  if (inPathId >= hppProblem.getNbPaths()) {
    cerr << "ChppPlanner::optimizePath: problem Id="
	 << inPathId << " is bigger than number of paths="
	 << hppProblem.getNbPaths();
    return KD_ERROR;
  }
  CkwsPathShPtr kwsPath = hppProblem.getIthPath(inPathId);

  // optimizer for the path
  if (hppProblem.pathOptimizer()) {
    hppProblem.pathOptimizer()->optimizePath(kwsPath, hppProblem.roadmapBuilder()->penetration());
    
    cout << "ChppPlanner::solveOneProblem: path optimized with penetration " 
	 << hppProblem.roadmapBuilder()->penetration()<<endl;
    
  } else {
    cerr << " no Optimizer Defined " << endl ;
  }
  return KD_OK;
}

// ==========================================================================

unsigned int ChppPlanner::getNbPaths(unsigned int inProblemId) const
{
  unsigned int nbPaths = 0;
  if (inProblemId < getNbHppProblems()) {
    nbPaths =  hppProblemVector[inProblemId].getNbPaths();
  } else {
    cerr << "ChppPlanner::getNbPaths : inProblemId = "<< inProblemId << " should be smaller than nb of problems: " << getNbHppProblems() << endl;
  }
  return nbPaths;
}

// ==========================================================================

CkwsPathShPtr ChppPlanner::getPath(unsigned int inProblemId,
                                   unsigned int inPathId) const
{
  CkwsPathShPtr resultPath;

  if (inProblemId < getNbHppProblems()) {
    if (inPathId < hppProblemVector[inProblemId].getNbPaths()) {
      resultPath = hppProblemVector[inProblemId].getIthPath(inPathId);
    }
  }
  return resultPath;
}

// ==========================================================================

ktStatus ChppPlanner::addPath(unsigned int inProblemId, CkwsPathShPtr kwsPath)
{
  if (inProblemId >= hppProblemVector.size()) {
    cerr << "ChppPlanner::addPath : inProblemId bigger than vector size." << endl;
    return KD_ERROR;
  }
  hppProblemVector[inProblemId].addPath(kwsPath);
  return KD_OK;
}

// ==========================================================================

ktStatus ChppPlanner::drawRoadmap(unsigned int inProblemId)
{
  if (inProblemId >= hppProblemVector.size()) {
    cerr << "ChppPlanner::drawRoadmap : inProblemId bigger than vector size." << endl;
    return KD_ERROR;
  }
  ChppProblem& hppProblem = hppProblemVector[inProblemId];

  if (hppProblem.drawRoadmap() == KD_ERROR) {
    return KD_ERROR;
  }
  return KD_OK;
}

// ==========================================================================

ktStatus ChppPlanner::stopdrawingRoadmap(unsigned int inProblemId)
{
  if (inProblemId >= hppProblemVector.size()) {
    cerr << "ChppPlanner::drawRoadmap : inProblemId bigger than vector size." << endl;
    return KD_ERROR;
  }
  ChppProblem& hppProblem = hppProblemVector[inProblemId];
  hppProblem.stopDrawingRoadmap();
  return KD_OK;
}

// ==========================================================================

ChppBodyConstShPtr ChppPlanner::findBodyByName(std::string inBodyName) const
{
  ChppBodyConstShPtr hppBody;
  unsigned int nbProblems = getNbHppProblems();

  // Loop over hppProblem.
  for (unsigned int iProblem=0; iProblem < nbProblems; iProblem++) {
    CkwsDevice::TBodyVector bodyVector;
    const CkppDeviceComponentShPtr hppRobot = robotIthProblem(iProblem);
    hppRobot->getBodyVector(bodyVector);

    // Loop over bodies of the robot.
    for (CkwsDevice::TBodyIterator bodyIter=bodyVector.begin(); bodyIter < bodyVector.end(); bodyIter++) {
      // Cast body into ChppBody
      if (ChppBodyShPtr hppBodyInRobot = boost::dynamic_pointer_cast<ChppBody>(*bodyIter)) {
	if (hppBodyInRobot->name() == inBodyName) {
	  hppBody = hppBodyInRobot;
	  return hppBody;
	}
      } else {
	cerr << "ChppPlanner::findBodyByName : one body of robot in hppProblem "<< iProblem << " is not a ChppBody." << endl;
      }
    }
  }
  return hppBody;
}


ktStatus ChppPlanner::solve()
{
  ktStatus success=KD_OK;
  for (unsigned int iProblem=0; iProblem<getNbHppProblems(); iProblem++) {
    if (solveOneProblem(iProblem) != KD_OK) {
      success = KD_ERROR;
    }
  }
  return success;
}
