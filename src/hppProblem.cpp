/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)
           and Mathieu Poirier (LAAS-CNRS)
*/


/*****************************************
 INCLUDES
*******************************************/

#include <iostream>
#include "hppProblem.h"

#include "hppBody.h"

const CkitNotification::TType  ChppProblem::ID_HPP_ADD_PATH(CkitNotification::makeID());

const std::string ChppProblem::PATH_KEY("path");
const std::string ChppProblem::PATH_ID_KEY("path_id");
const std::string ChppProblem::DEVICE_KEY("device");

/*! \addtogroup hpp
 *@{
 */

/*****************************************
 PUBLIC METHODS
*******************************************/

// ==========================================================================

ChppProblem::ChppProblem(ChppDeviceShPtr inRobot)
{
  attNotificator = CkitNotificator::defaultNotificator(); 
  attRobot = inRobot;
}

// ==========================================================================

ChppDeviceShPtr ChppProblem::getRobot() const
{
  return attRobot;
}

// ==========================================================================

CkwsConfigShPtr ChppProblem::initConfig() const
{
  return attInitConf;
}

// ==========================================================================

void ChppProblem::initConfig(CkwsConfigShPtr inConfig)
{
  attInitConf = inConfig;
}

// ==========================================================================

CkwsConfigShPtr ChppProblem::goalConfig() const
{
  return attGoalConf;
}

// ==========================================================================

void ChppProblem::goalConfig(CkwsConfigShPtr inConfig)
{
  attGoalConf = inConfig;
}

// ==========================================================================

ktStatus ChppProblem::obstacleList(const std::vector<CkcdObjectShPtr>& inCollisionList)
{
  // Copy collision list in problem.
  attObstacleList = inCollisionList;
  
  // For each obstacle, 
  //   - insert the obstacle in outer object of each body of the robot.
  
  // Loop over object in the collision list.
  // Get robot vector of bodies.
  CkwsDevice::TBodyVector bodyVector;
  attRobot->getBodyVector(bodyVector);
  
  // Loop over bodies of robot.
  for (CkwsDevice::TBodyIterator bodyIter = bodyVector.begin(); bodyIter < bodyVector.end(); bodyIter++) {
    // Try to cast body into CkwsKCDBody
    CkwsKCDBodyShPtr kcdBody;
    if (kcdBody = boost::dynamic_pointer_cast<CkwsKCDBody>(*bodyIter)) {
      // Copy list of outer objects
      std::vector<CkcdObjectShPtr> collisionList = kcdBody->outerObjects();
      unsigned int nObjects = attObstacleList.size();
      for (unsigned int iObject=0; iObject<nObjects; iObject++) {
	CkcdObjectShPtr kcdObject = attObstacleList[iObject];
	// Add object to list of outer objects
	collisionList.push_back(kcdObject);
      }
      // Set updated list as new list of outer objects
      kcdBody->outerObjects(collisionList);
    }
    else {
      std::cout << "CcppProblem::obstacleList: body is not KCD body. Obstacle not inserted." << std::endl;
    }
  }
  return KD_OK;
}

// ==========================================================================

const std::vector<CkcdObjectShPtr>& ChppProblem::obstacleList()
{
  return attObstacleList;
}

// ==========================================================================

ktStatus ChppProblem::addObstacle(const CkcdObjectShPtr& inObject)
{
  return attRobot->addObstacle(inObject);  
}

// ==========================================================================

void ChppProblem::steeringMethod(const CkwsSteeringMethodShPtr &inSteeringMethod)
{
  attRobot->steeringMethod(inSteeringMethod);
}

// ==========================================================================

CkwsSteeringMethodShPtr ChppProblem::steeringMethod() const 
{
  return attRobot->steeringMethod();
}


// ==========================================================================

void ChppProblem::addPath(CkwsPathShPtr kwsPath)
{
  attPathVector.push_back(kwsPath);
  std::string robotName = attRobot->name();

  // attNotificator->sendNotification(ID_HPP_ADD_PATH, this);
  CkitNotificationShPtr notification 
     = CkitNotification::createWithPtr<ChppProblem>(ChppProblem::ID_HPP_ADD_PATH, this);
  // set attribute to retreave
  notification->shPtrValue<CkwsPath>(PATH_KEY, kwsPath);
  notification->shPtrValue<ChppDevice>(DEVICE_KEY, attRobot);
  // set path number
  notification->unsignedIntValue(PATH_ID_KEY, attPathVector.size());
  attNotificator->notify(notification);

  
}

// ==========================================================================

CkwsPathShPtr ChppProblem::getIthPath(unsigned int pathId) const
{
  CkwsPathShPtr resultPath;
  
  if (pathId < attPathVector.size()) {
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

void ChppProblem::roadmapBuilder( CkwsRoadmapBuilderShPtr inRoadmapBuilder) {

  attRoadmapBuilder = inRoadmapBuilder ;
}
 
// ==========================================================================

CkwsRoadmapBuilderShPtr ChppProblem::roadmapBuilder() {
  
  return attRoadmapBuilder ; 
}
 
// ==========================================================================

CkwsPlusRoadmapShPtr ChppProblem::roadmap() const {
  
  return attRoadmap ;
}

// ==========================================================================

void ChppProblem::roadmap(CkwsPlusRoadmapShPtr inRoadmap)  {
  
  attRoadmap = inRoadmap ;
}

// ==========================================================================

void ChppProblem::pathOptimizer( CkwsPathOptimizerShPtr inOptimizer){

  attPathOptimizer = inOptimizer ;
}

// ==========================================================================

CkwsPathOptimizerShPtr ChppProblem::pathOptimizer() {
  return attPathOptimizer ;
}



/** @}
 */

