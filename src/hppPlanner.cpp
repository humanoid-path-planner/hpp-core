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

#include "kprParserXML/kprParserManager.h"
#include "KineoModel/kppModelTree.h"
#include "KineoModel/kppDeviceNode.h"
#include "KineoModel/kppGeometryNode.h"
#include "KineoModel/kppPathNode.h"
#include "KineoModel/kppPathComponent.h"

const CkitNotification::TType ChppPlanner::ID_HPP_ADD_ROBOT(CkitNotification::makeID());
const CkitNotification::TType ChppPlanner::ID_HPP_SET_CURRENT_CONFIG(CkitNotification::makeID());
const CkitNotification::TType ChppPlanner::ID_HPP_REMOVE_OBSTACLES(CkitNotification::makeID());
const CkitNotification::TType ChppPlanner::ID_HPP_SET_OBSTACLE_LIST(CkitNotification::makeID());
const CkitNotification::TType ChppPlanner::ID_HPP_ADD_OBSTACLE(CkitNotification::makeID());
const CkitNotification::TType ChppPlanner::ID_HPP_REMOVE_ROADMAPBUILDER(CkitNotification::makeID());
const CkitNotification::TType ChppPlanner::ID_HPP_ADD_ROADMAPBUILDER(CkitNotification::makeID());

const std::string ChppPlanner::ROBOT_KEY("robot");
const std::string ChppPlanner::OBSTACLE_KEY("obstacle");
const std::string ChppPlanner::CONFIG_KEY("config");
const std::string ChppPlanner::ROADMAP_KEY("roadmap");

#if DEBUG==2
#define ODEBUG2(x) std::cout << "ChppPlanner:" << x << std::endl
#define ODEBUG1(x) std::cerr << "ChppPlanner:" << x << std::endl
#elif DEBUG==1
#define ODEBUG2(x)
#define ODEBUG1(x) std::cerr << "ChppPlanner:" << x << std::endl
#else
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif

/*****************************************
 PUBLIC METHODS
*******************************************/

// ==========================================================================

ChppPlanner::ChppPlanner()
{
  attNotificator = CkitNotificator::defaultNotificator(); 
  mObstacleList.clear();
  attStopRdmBuilderDelegate = new CkwsPlusStopRdmBuilderDelegate;

}


// ==========================================================================


ChppPlanner::~ChppPlanner()
{
  delete attStopRdmBuilderDelegate;
}

// ==========================================================================

ktStatus ChppPlanner::addHppProblem(CkppDeviceComponentShPtr robot)
{
  ChppProblem hppProblem(robot, mObstacleList);

  ODEBUG2(":addHppProblem: adding a problem in vector");
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

ktStatus ChppPlanner::removeHppProblem(){

  if(hppProblemVector.size()){
    hppProblemVector.pop_back();
    mObstacleList.clear();
    return KD_OK;
  }

  return KD_ERROR;

}

// ==========================================================================

ktStatus ChppPlanner::addHppProblemAtBeginning(CkppDeviceComponentShPtr robot)
{
  ChppProblem hppProblem(robot, mObstacleList);

  ODEBUG2(":addHppProblemAtBeginning: adding a problem");
  // Add robot in vector .
  hppProblemVector.push_front(hppProblem);

  CkitNotificationShPtr notification  = CkitNotification::createWithPtr<ChppPlanner>(ChppPlanner::ID_HPP_ADD_ROBOT, this);
  // set attribute if necessary
  notification->shPtrValue<CkppDeviceComponent>(ROBOT_KEY, robot);
  attNotificator->notify(notification);


  return KD_OK;
}
// ==========================================================================

ktStatus ChppPlanner::removeHppProblemAtBeginning(){

  if(hppProblemVector.size()){
    hppProblemVector.pop_front();
    mObstacleList.clear();
    return KD_OK;
  }

  return KD_ERROR;

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
    status = hppProblemVector[rank].initConfig(config);
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

ktStatus ChppPlanner::goalConfIthProblem(unsigned int rank,
					 const CkwsConfigShPtr config)

{
  ktStatus status = KD_ERROR;

  if (rank < getNbHppProblems()) {
    status = hppProblemVector[rank].goalConfig(config);
  }

  return status;
}

// ==========================================================================


ktStatus ChppPlanner::roadmapBuilderIthProblem(unsigned int rank,
					       CkwsRoadmapBuilderShPtr inRoadmapBuilder,
					       bool inDisplay)
{
  if (rank >= getNbHppProblems()) {
    return KD_ERROR;
  }

  /*
    If a roadmap was already stored, it will be removed. If this roadmap is displayed 
    in the interface, we need first to remove the corresponding data-structure in the 
    interface. This is done by sending a notification.
  */
  CkitNotificationShPtr notification  = 
    CkitNotification::createWithPtr<ChppPlanner>(ChppPlanner::ID_HPP_REMOVE_ROADMAPBUILDER, this);
  notification->unsignedIntValue(ChppPlanner::ROADMAP_KEY, rank);
  attNotificator->notify(notification);

  /*
    Add an interruption delegate to the roadmap builder
  */
#if 0
  inRoadmapBuilder->addDelegate(attStopRdmBuilderDelegate);
#endif
  ChppProblem& hppProblem =  hppProblemVector[rank];
  hppProblem.roadmapBuilder(inRoadmapBuilder);
  
  /* 
     If the new roadmap is displayed in the interface, send a notification to 
     trigger appropriate action.
  */
  if (inDisplay) {
    notification = 
      CkitNotification::createWithPtr<ChppPlanner>(ChppPlanner::ID_HPP_ADD_ROADMAPBUILDER, this);
    notification->unsignedIntValue(ChppPlanner::ROADMAP_KEY, rank);
    attNotificator->notify(notification);
  }

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
    ODEBUG1(":solveOneProblem: problem Id=" << problemId << "is bigger than vector size=" << getNbHppProblems());

    return KD_ERROR;
  }

  ChppProblem& hppProblem = hppProblemVector[problemId];

  CkppDeviceComponentShPtr hppDevice = hppProblem.getRobot();
  if (!hppDevice)
    return KD_ERROR ;

  CkwsPathShPtr	kwsPath = CkwsPath::create(hppDevice);

  CkwsConfigShPtr initConfig = hppProblem.initConfig() ;
  if (!initConfig)
    return KD_ERROR ;
  CkwsConfigShPtr goalConfig = hppProblem.goalConfig() ;
  if (!goalConfig)
    return KD_ERROR ;

  if(!hppProblem.roadmapBuilder()){
    ODEBUG1(":solveOneProblem: problem Id=" << problemId << ": Define a roadmap builder with penetration");
    return KD_ERROR ;
  }

  /*
    Test that configurations are valid
  */
  if (validateConfig(hppDevice, initConfig) != KD_OK) {
    return KD_ERROR;
  }
  if (validateConfig(hppDevice, goalConfig) != KD_OK) {
    return KD_ERROR;
  }

  double penetration = hppProblem.roadmapBuilder()->penetration();

  /*
    Try first a direct path.
  */
  CkwsSteeringMethodShPtr steeringMethod = hppDevice->steeringMethod();

  if ( initConfig && goalConfig && steeringMethod) {
    CkwsDirectPathShPtr directPath = steeringMethod->makeDirectPath(*initConfig, *goalConfig);

    if (directPath) {
      CkwsDirectPathValidatorSetConstShPtr dpValidator = 
	CkwsDirectPathValidatorSet::createCopy(hppDevice->directPathValidators());
      
      CkwsValidatorDPCollisionShPtr collisionValidator = 
	dpValidator->retrieve<CkwsValidatorDPCollision>();
      if (collisionValidator) {
	collisionValidator->penetration(penetration);
      }

      dpValidator->validate(*directPath);
      if (directPath->isValid()) {

	ODEBUG2(":solveOneProblem: Problem solved with direct connection. ");

#if 1
	/* Add direct path to roadmap if not already included */
	if (hppProblem.roadmapBuilder()) {
	  CkwsRoadmapShPtr roadmap = hppProblem.roadmapBuilder()->roadmap();
	  ODEBUG2(":solveOneProblem: number of edges in roadmap before inserting nodes = " << roadmap->countEdges());
	  
	  CkwsNodeShPtr startNode = roadmap->nodeWithConfig(*initConfig);
	  CkwsNodeShPtr goalNode = roadmap->nodeWithConfig(*goalConfig);

	  ODEBUG2(":solveOneProblem: number of edges in roadmap after creating nodes = " << roadmap->countEdges());

	  /* If start and goal node are not in roadmap, add them. */
	  if (!startNode) {
	    startNode = CkwsNode::create(*initConfig);
	    if (roadmap->addNode(startNode) != KD_OK) {
	      ODEBUG1(":solveOneProblem: failed to add start node in roadmap.");
	      startNode.reset();
	    }
	  }
	  if (!goalNode) {
	    goalNode = CkwsNode::create(*goalConfig);
	    if (roadmap->addNode(goalNode) != KD_OK) {
	      ODEBUG1(":solveOneProblem: failed to add goal node in roadmap.");
	      goalNode.reset();
	    }
	  }
	  
	  ODEBUG2(":solveOneProblem: number of edges in roadmap after adding nodes = " << roadmap->countEdges());

	  if (startNode && goalNode) {
	    /* Add edge only if goal node is not accessible from initial node */
	    if (!startNode->hasTransitiveOutNode(goalNode)) {
	      CkwsEdgeShPtr edge=CkwsEdge::create (directPath);
	      
	      if (roadmap->addEdge(startNode, goalNode, edge) == KD_ERROR) {
		ODEBUG2(":solveOneProblem: Failed to add direct path in roadmap.");
	      }
	    }
	    ODEBUG2(":solveOneProblem: number of edges in roadmap after attempting at adding edge= " << roadmap->countEdges());
	  }
	}
#endif
	kwsPath->appendDirectPath(directPath);
	// Add the path to vector of paths of the problem.
	hppProblem.addPath(KIT_DYNAMIC_PTR_CAST(CkwsPath, kwsPath->clone()));

	return KD_OK;
      }
    }
  } else {
    ODEBUG1(":solveOneProblem: problem ill-defined:initConfig, GoalConfig or steering method not defined");
    return KD_ERROR ;
  }


  /*
    solve the problem with the roadmapBuilder
  */
  if (hppProblem.roadmapBuilder()) {

    if(KD_OK == hppProblem.roadmapBuilder()->solveProblem( *initConfig , *goalConfig , kwsPath)) {
      ODEBUG2(":solveOneProblem: --- Problem solved.----");
    } else {
      ODEBUG2(":solveOneProblem: ---- Problem NOT solved.----");
      return KD_ERROR;
    }
    if (!kwsPath) {
      ODEBUG1(":solveOneProblem: no path after successfully solving the problem");
      ODEBUG1(":solveOneProblem: this should not happen.");
      return KD_ERROR;
    }

    if (kwsPath->length()== 0) {
      ODEBUG1(":solveOneProblem: Path length is 0 after successfully solving the problem");
      ODEBUG1(":solveOneProblem: this should not happen.");
      return KD_ERROR;
    }

    /*
      Store path before optimization
    */
    hppProblem.addPath(KIT_DYNAMIC_PTR_CAST(CkwsPath, kwsPath->clone()));

    // optimizer for the path
    if (hppProblem.pathOptimizer()) {
      if (hppProblem.pathOptimizer()->optimizePath(kwsPath, 
						   penetration)
	  == KD_OK) {
	
	ODEBUG2(":solveOneProblem: path optimized with penetration " 
		<< penetration);
      }
      else {
	ODEBUG1(":solveOneProblem: path optimization failed.");
      }
      
    } else {
      ODEBUG1(":solveOneProblem: no Optimizer Defined ");
    }

    if (kwsPath) {
      ODEBUG2(":solveOneProblem: number of direct path: "<<kwsPath->countDirectPaths());
      // Add the path to vector of paths of the problem.
      hppProblem.addPath(KIT_DYNAMIC_PTR_CAST(CkwsPath, kwsPath->clone()));
    }
  } else {
    ODEBUG1(":solveOneProblem: no roadmap builder");
    return KD_ERROR ;
  }

  return KD_OK ;
}

ktStatus ChppPlanner::optimizePath(unsigned int inProblemId, unsigned int inPathId)
{
  if (inProblemId >= getNbHppProblems()) {
    ODEBUG1(":optimizePath: problem Id=" << inProblemId << " is bigger than vector size=" << getNbHppProblems());

    return KD_ERROR;
  }

  ChppProblem& hppProblem = hppProblemVector[inProblemId];
  CkwsRoadmapBuilderShPtr roadmapBuilder = hppProblem.roadmapBuilder();
  if (!roadmapBuilder) {
    ODEBUG1(":optimizePath: roadmap builder should be set to define penetration.");
    return KD_ERROR;
  }
  double penetration = roadmapBuilder->penetration();
    
  if (inPathId >= hppProblem.getNbPaths()) {
    ODEBUG1(":optimizePath: problem Id="
	    << inPathId << " is bigger than number of paths="
	    << hppProblem.getNbPaths());
    return KD_ERROR;
  }
  CkwsPathShPtr kwsPath = hppProblem.getIthPath(inPathId);

  // optimizer for the path
  if (hppProblem.pathOptimizer()) {
    hppProblem.pathOptimizer()->optimizePath(kwsPath, penetration);
    
    ODEBUG2(":optimizePath: path optimized with penetration " 
	    << penetration);
    
  } else {
    ODEBUG1(":optimizePath: no optimizer defined");
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
    ODEBUG1(":getNbPaths : inProblemId = "<< inProblemId << " should be smaller than nb of problems: " << getNbHppProblems());
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
    ODEBUG1(":addPath : inProblemId bigger than vector size.");
    return KD_ERROR;
  }
  hppProblemVector[inProblemId].addPath(kwsPath);
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
	ODEBUG1(":findBodyByName : one body of robot in hppProblem "<< iProblem << " is not a ChppBody.");
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

void ChppPlanner::interruptPathPlanning()
{
  if (attStopRdmBuilderDelegate == NULL) {
    ODEBUG1(":interruptPathPlanning: no stop delegate.");
    return;
  }
  attStopRdmBuilderDelegate->shouldStop(true);
}

ktStatus ChppPlanner::validateConfig(CkppDeviceComponentShPtr inDevice, 
				     const CkwsConfigShPtr& inConfig)
{
  inDevice->configValidators()->validate(*inConfig);
  
  if (inConfig->isValid()) {
    return KD_OK;
  }
  for(unsigned int i=0; i<inConfig->countReports(); ++i) {
    std::string theValidatorName;
    CkwsValidationReportConstShPtr theReport(inConfig->report(i, theValidatorName));
    if(!theReport->isValid()) {
      ODEBUG1(" " << theValidatorName << 
	      " failed at validating the configuration.");
      
      // If this is a CkwsDofReport then we can retrieve more information...
      CkwsReportCfgDofConstShPtr theDofReport;
      theDofReport = KIT_DYNAMIC_PTR_CAST(CkwsReportCfgDof const, theReport);
      if(theDofReport) {
	for(unsigned int j=0; j<theDofReport->countDofs(); ++j) {
	  if(!theDofReport->isDofValid(j)) {
	    ODEBUG1(" Dof #" << j << " is invalid.");
	  }
	}
      }

    }
  }
  return KD_ERROR;
}

ktStatus ChppPlanner::parseFile(string inFileName)
{

  CkppComponentShPtr modelTreeComponent;
  CkprParserManagerShPtr parser = CkprParserManager::defaultManager();
  
  if(inFileName.length() == 0) { 
    ODEBUG1(":parseFile: no file name"); 
    return KD_ERROR;
  }
  
  if(KD_ERROR == parser->loadComponentFromFile(inFileName, modelTreeComponent)) { 
    ODEBUG1(":parseFile: unable to load file " << inFileName); 
    ODEBUG2(":parseFile: " << parser->lastError().errorMessage());
    return KD_ERROR; 
  }

  CkppModelTreeShPtr modelTree = KIT_DYNAMIC_PTR_CAST(CkppModelTree,modelTreeComponent);

  CkppSolidComponentShPtr solidComponent = KIT_DYNAMIC_PTR_CAST(CkppSolidComponent,modelTreeComponent);

  if(modelTree) { 
    ODEBUG1(":parseFile: found modelTree"); 


    map<CkppDeviceComponentShPtr,unsigned int> devicesIndex;
    
    set<CkcdObjectShPtr> devicesBodies;
    
    unsigned int currentRank=0;
    
    if(!modelTree->deviceNode()) cout<<"No devices"<<endl;
    else{ 
      for(unsigned int i = 0; i< modelTree->deviceNode()->countChildComponents(); i++){
	
	CkppDeviceComponentShPtr deviceComponent = KIT_DYNAMIC_PTR_CAST(CkppDeviceComponent,modelTree->deviceNode()->childComponent(i));
	
	if(deviceComponent){
	  
	  addHppProblem(deviceComponent);
	  
	  devicesIndex.insert(pair<CkppDeviceComponentShPtr,unsigned int>(deviceComponent,currentRank));
	  
	  vector< CkppSolidComponentRefShPtr > solidComponentRefVector;
	  
	  deviceComponent->getSolidComponentRefVector (solidComponentRefVector);
	  
	  for(vector< CkppSolidComponentRefShPtr >::iterator it=solidComponentRefVector.begin();it!=solidComponentRefVector.end();it++)
	    {
	      CkcdObjectShPtr kcdObject = KIT_DYNAMIC_PTR_CAST(CkcdObject,(*it)->referencedSolidComponent() );
	      if(kcdObject) devicesBodies.insert(kcdObject);
	      else cout<<"Cannot cast component to kcdObject"<<endl;
	    }
	  
	  currentRank++;
	  
	}
	
      }
    }
    
    
    
    if(!modelTree->geometryNode()) cout<<"No geometries"<<endl;
    else{
      cout<<"geometries"<<endl;
      for(unsigned int i = 0; i< modelTree->geometryNode()->countChildComponents(); i++){
	
	cout<<i<<" "<<endl;
	
	CkcdObjectShPtr kcdObject = KIT_DYNAMIC_PTR_CAST(CkcdObject, modelTree->geometryNode()->childComponent(i));
	if(kcdObject) addObstacle(kcdObject);
	else cout<<"Cannot cast component to kcdObject"<<endl;
      }
    }
    

    
  
    if(!modelTree->pathNode()) cout<<"No paths"<<endl;
    else{
      
      for(unsigned int i = 0; i< modelTree->pathNode()->countChildComponents(); i++){
	CkppPathComponentShPtr pathComponent = KIT_DYNAMIC_PTR_CAST(CkppPathComponent,modelTree->pathNode()->childComponent(i));
	
	map<CkppDeviceComponentShPtr,unsigned int>::iterator it=devicesIndex.find(pathComponent->deviceComponent());
	
	if(it==devicesIndex.end()){ cout<<"no device matching path"<<endl; return KD_ERROR;}
	
	unsigned int rank=it->second;
	
	addPath(rank,pathComponent->kwsPath());
	
      }
      
    }
  }
  

  else if(solidComponent)
    {
      ODEBUG1(":parseFile: found solid component");
      CkcdObjectShPtr kcdObject = KIT_DYNAMIC_PTR_CAST(CkcdObject,solidComponent);
      if(!kcdObject)
	{
	  ODEBUG1(":parseFile: cannot cast solidComponent to kcdObject");
	  return KD_ERROR;
	}
      addObstacle(kcdObject);
    }

  else
    {
      ODEBUG1(":parseFile: cannot cast file neither to CkppSolidComponent nor to CkppModelTree ");
      return KD_ERROR;
    }
  
  ODEBUG1(":parseFile: everything is fine ");
  return KD_OK ;

}

