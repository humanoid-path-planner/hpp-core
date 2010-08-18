/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)
  and Mathieu Poirier (LAAS-CNRS)
*/


/*****************************************
 INCLUDES
*******************************************/

#include <iostream>

#include "hpp/core/problem.hh"
#include "hppModel/hppBody.h"

#include "KineoWorks2/kwsConfigExtractor.h"
#include "KineoWorks2/kwsValidatorDPCollision.h"
#include "KineoWorks2/kwsSteeringMethod.h"
#include "KineoWorks2/kwsRoadmap.h"
#include "KineoWorks2/kwsNode.h"
#include "KineoWorks2/kwsEdge.h"
#include "KineoWorks2/kwsReportCfgDof.h"

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
  attPenetration(inPenetration),
  attAlwaysOptimize(false)
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
  attPenetration(inPenetration),
  attAlwaysOptimize(false)
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
  attPenetration(inProblem.attPenetration),
  attAlwaysOptimize(inProblem.attAlwaysOptimize)
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

ktStatus ChppProblem::checkProblem() const
{
  if (!getRobot()) {
    ODEBUG1(":checkProblem: no device in problem.");
    return KD_ERROR;
  }

  if (!initConfig()) {
    ODEBUG1(":checkProblem: no init config in problem.");
    return KD_ERROR;
  }
  if (!goalConfig()) {
    ODEBUG1(":checkProblem: no goal config in problem.");
    return KD_ERROR;
  }

  if(!roadmapBuilder()){
    ODEBUG1(":checkProblem: define a roadmap builder with penetration");
    return KD_ERROR;
  }

  if (!steeringMethod()) {
    ODEBUG1(":checkProblem: define a steering method.");
    return KD_ERROR;
  }


  /*
    Test that goal configuration is valid
  */
  if (validateConfig(getRobot(), goalConfig()) != KD_OK) {
    ODEBUG1("checkProblem:: goal configuration not valid.");
    return KD_ERROR;
  }
  return KD_OK;
}

// ==========================================================================

ktStatus ChppProblem::validateInitConfig(CkwsConfigShPtr& inOutInitConfig,
					 CkwsPathShPtr& inOutPath) const
{
  if (validateConfig(getRobot(), initConfig()) == KD_OK) {
    ODEBUG2(":validateInitConfig: initial configuration is valid.");
    return KD_OK;
  }
  /*
    If initial configuration is not valid and configuration extractor
    has been set, try to extract a valid configuration in the neighborhood
    of the initial configuration.
  */
  if (CkwsConfigExtractorShPtr confExtractor = attConfigExtractor) {
    ODEBUG3(":validateInitConfig: confExtractor->minRadius = " 
	    << confExtractor->minRadius());
    ODEBUG3
      (":validateInitConfig: confExtractor->maxRadius = " 
       << confExtractor->maxRadius());
    ODEBUG3
      (":validateInitConfig: confExtractor->scaleFactor = " 
       << confExtractor->scaleFactor());
    CkwsPathShPtr initConfigPath = 
      CkwsPath::createWithConfig(*inOutInitConfig);

    ODEBUG3
      (":validateInitConfig: number of configurations in initConfigPath = "
       << initConfigPath->countConfigurations());

#if DEBUG >= 3
    for (unsigned int i=0; i<initConfigPath->countConfigurations(); i++ ) {
      CkwsConfig config(getRobot());
      initConfigPath->getConfiguration(i, config);
      ODEBUG3(":validateInitConfig: configuration # " << i
	      << " = " << config);
    }
#endif

    if (confExtractor->plan(initConfigPath, CkwsPathPlanner::STABLE_START,
			    inOutPath) == KD_OK) {
      ODEBUG2
	(":validateInitConfig: number of configurations in extracted path = "
	 << inOutPath->countConfigurations());

#if DEBUG >= 3
      for (unsigned int i=0; i<inOutPath->countConfigurations(); i++ ) {
	CkwsConfig config(getRobot());
	inOutPath->getConfiguration(i, config);
	ODEBUG3(":validateInitConfig: configuration # " << i
		<< " = " << config);
      }
#endif

      /*
	Replace inOutInitConfig by end of extraction path for path planning problem
      */
      inOutInitConfig = inOutPath->configAtEnd();
      if (!inOutInitConfig) {
	ODEBUG1
	  (":validateInitConfig: no configuration at end of extraction path.");
	return KD_ERROR;
      }
    }
    else {
      ODEBUG1
	(":validateInitConfig: failed to extract initial configuration.");
      return KD_ERROR;
    }
  }
  else {
    ODEBUG2
      (":validateInitConfig: initial configuration not valid.");
    return KD_ERROR;
  }
  return KD_OK;
}

// ==========================================================================

ktStatus ChppProblem::planPath(const CkwsConfigConstShPtr inInitConfig,
			       const CkwsConfigConstShPtr inGoalConfig,
			       const CkwsPathShPtr& inOutPath)
{
  /*
    Try first a direct path.
  */
  CkwsSteeringMethodShPtr steeringMethod = getRobot()->steeringMethod();

  CkwsDirectPathShPtr directPath = 
    steeringMethod->makeDirectPath(*inInitConfig, *inGoalConfig);

  if (directPath) {
    /*
      Retrieve validators of device
    */
    CkwsDirectPathValidatorSetConstShPtr dpValidators =
      getRobot()->directPathValidators();

    dpValidators->validate(*directPath);
    if (directPath->isValid()) {

      ODEBUG2(":planPath: Problem solved with direct connection. ");

      /* Add direct path to roadmap if not already included */

      CkwsRoadmapShPtr roadmap = roadmapBuilder()->roadmap();
      ODEBUG2
	(":planPath: number of edges in roadmap before inserting nodes = " 
	 << roadmap->countEdges());

      CkwsNodeShPtr startNode = roadmap->nodeWithConfig(*inInitConfig);
      CkwsNodeShPtr goalNode = roadmap->nodeWithConfig(*inGoalConfig);

      ODEBUG2(":planPath: number of edges in roadmap after creating nodes = " 
	      << roadmap->countEdges());

      /* If start and goal node are not in roadmap, add them. */
      if (!startNode) {
	startNode = CkwsNode::create(*inInitConfig);
	if (roadmap->addNode(startNode) != KD_OK) {
	  ODEBUG1(":planPath: failed to add start node in roadmap.");
	  startNode.reset();
	}
      }
      if (!goalNode) {
	goalNode = CkwsNode::create(*inGoalConfig);
	if (roadmap->addNode(goalNode) != KD_OK) {
	  ODEBUG1(":planPath: failed to add goal node in roadmap.");
	  goalNode.reset();
	}
      }
      
      ODEBUG2(":planPath: number of edges in roadmap after adding nodes = " 
	      << roadmap->countEdges());
      
      if (startNode && goalNode) {
	/* Add edge only if goal node is not accessible from initial node */
	if (!startNode->hasTransitiveOutNode(goalNode)) {
	  CkwsEdgeShPtr edge=CkwsEdge::create (directPath);
	  
	  if (roadmap->addEdge(startNode, goalNode, edge) == KD_ERROR) {
	    ODEBUG2(":planPath: Failed to add direct path in roadmap.");
	  }
	}
	ODEBUG2
	  (":planPath: number edges in roadmap after attempt at adding edge= " 
	   << roadmap->countEdges());
      }
      inOutPath->appendDirectPath(directPath);
      // Add the path to vector of paths of the problem.
      addPath(KIT_DYNAMIC_PTR_CAST(CkwsPath, inOutPath->clone()));
      return KD_OK;
    }
  } /* if (directPath) */

  /*
    solve the problem with the roadmapBuilder
  */
  CkwsPathShPtr kwsPath = CkwsPath::create(getRobot());

  if(KD_OK == roadmapBuilder()->solveProblem(*inInitConfig , 
					     *inGoalConfig , kwsPath)) {
    ODEBUG2(":planPath: --- Problem solved.----");
  } else {
    ODEBUG1(":planPath: ---- Problem NOT solved.----");
    return KD_ERROR;
  }
  if (!kwsPath) {
    ODEBUG1(":planPath: no path after successfully solving the problem");
    ODEBUG1(":planPath: this should not happen.");
    return KD_ERROR;
  }

  if (kwsPath->length()== 0) {
    ODEBUG1
      (":planPath: Path length is 0 after successfully solving the problem");
    ODEBUG1(":planPath: this should not happen.");
    return KD_ERROR;
  }

  /*
    Store path before optimization
  */
  if (inOutPath->appendPath(kwsPath) != KD_OK) {
    ODEBUG1
      (":planPath: failed at appending solution path to extraction path.");
    return KD_ERROR;
  }
  addPath(CkwsPath::createCopy(inOutPath));
  return KD_OK;
}

// ==========================================================================

ktStatus ChppProblem::solve()
{
  if (checkProblem() != KD_OK) {
    ODEBUG1(":solve: problem formulation is not correct.");
    return KD_ERROR;
  }

  CkwsConfigShPtr initConf = initConfig();
  CkwsConfigShPtr goalConf = goalConfig();
  /*
    Test that initial configuration is valid
  */
  CkwsPathShPtr solutionPath = CkwsPath::create(getRobot());

  if (validateInitConfig(initConf, solutionPath) != KD_OK) {
    return KD_ERROR;
  }

  if (planPath(initConf, goalConf, solutionPath) != KD_OK) {
    ODEBUG1(":solve: failed to plan a path between init and goal config.");
    return KD_ERROR;
  }

  /*
    Optimize if 
      - there is a path optimizer and
      - the path has more than one direct path or the user has requested 
      that optimization is always performed.
  */
  bool shouldOptimize = 
    pathOptimizer() && (attAlwaysOptimize || 
			solutionPath->countDirectPaths() > 1);

  // optimizer for the path
  if (shouldOptimize) {
    if (pathOptimizer()->optimizePath(solutionPath, attPenetration)
	== KD_OK) {

      ODEBUG2(":solve: path optimized with penetration "
	      << attPenetration);
    }
    else {
      ODEBUG1(":solve: path optimization failed.");
      return KD_ERROR;
    }

  } else {
    ODEBUG2(":solve: no optimization performed ");
    return KD_OK;
  }

  if (solutionPath) {
    ODEBUG2(":solve: number of direct path: "
	    << solutionPath->countDirectPaths());
    // Add the path to vector of paths of the problem.
    addPath(solutionPath);
  }

  return KD_OK ;
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

CkwsRoadmapBuilderShPtr ChppProblem::roadmapBuilder() const
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

// ==========================================================================

ktStatus
ChppProblem::validateConfig(CkppDeviceComponentShPtr inDevice,
			    const CkwsConfigConstShPtr& inConfig) const
{
  inDevice->configValidators()->validate(*inConfig);

  if (inConfig->isValid()) {
    return KD_OK;
  }
  for(unsigned int i=0; i<inConfig->countReports(); ++i) {
    std::string theValidatorName;
    CkwsValidationReportConstShPtr theReport(inConfig->report(i, theValidatorName));
    if(!theReport->isValid()) {
      ODEBUG2(" " << theValidatorName <<
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
