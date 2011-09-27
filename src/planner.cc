//
// Copyright (c) 2005, 2006, 2007, 2008, 2009, 2010, 2011 CNRS
// Authors: Florent Lamiraux
//
// This file is part of hpp-core
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#define HPPPLANNER_DEFAULT_PENETRATION 0.05

// TODO : nettoyer les reste de la classe hppProblem


/*****************************************
 INCLUDES
*******************************************/

#include <iostream>
#include <fstream>

#include "KineoModel/kppDeviceComponent.h"

#include <hpp/util/debug.hh>
#include "hpp/core/planner.hh"
#include "hpp/core/problem.hh"
#include "hpp/model/parser.hh"
#include "hpp/model/body.hh"
#include <hpp/model/humanoid-robot.hh>

#include "KineoUtility/kitNotificator.h"
#include "KineoWorks2/kwsDirectPath.h"
#include "KineoWorks2/kwsJoint.h"

#include "kprParserXML/kprParserManager.h"
#include "KineoModel/kppModelTree.h"
#include "KineoModel/kppDeviceNode.h"
#include "KineoModel/kppGeometryNode.h"
#include "KineoModel/kppPathNode.h"
#include "KineoModel/kppPathComponent.h"
#include "KineoModel/kppJointComponent.h"

#include "kwsPlus/roadmap/kwsPlusStopRdmBuilderDelegate.h"

KIT_PREDEF_CLASS(CkwsSteeringMethod);

namespace hpp {
  namespace core {
    const CkitNotification::TType
    Planner::ID_HPP_ADD_ROBOT(CkitNotification::makeID());
    const CkitNotification::TType
    Planner::ID_HPP_SET_CURRENT_CONFIG(CkitNotification::makeID());
    const CkitNotification::TType
    Planner::ID_HPP_REMOVE_OBSTACLES(CkitNotification::makeID());
    const CkitNotification::TType
    Planner::ID_HPP_SET_OBSTACLE_LIST(CkitNotification::makeID());
    const CkitNotification::TType
    Planner::ID_HPP_ADD_OBSTACLE(CkitNotification::makeID());
    const CkitNotification::TType
    Planner::ID_HPP_REMOVE_ROADMAPBUILDER(CkitNotification::makeID());
    const CkitNotification::TType
    Planner::ID_HPP_ADD_ROADMAPBUILDER(CkitNotification::makeID());

    const std::string Planner::ROBOT_KEY("robot");
    const std::string Planner::OBSTACLE_KEY("obstacle");
    const std::string Planner::CONFIG_KEY("config");
    const std::string Planner::ROADMAP_KEY("roadmap");

    // ======================================================================

    Planner::Planner()
    {
      notificator_ = CkitNotificator::defaultNotificator();
      obstacleVector_.clear();
      stopRdmBuilderDelegate_ = new CkwsPlusStopRdmBuilderDelegate;
      parser_ = new hpp::model::Parser();
    }

    // ======================================================================

    Planner::Planner(const Planner& inPlanner) :
      notificator_(inPlanner.notificator_),
      problemVector_(inPlanner.problemVector_),
      obstacleVector_(inPlanner.obstacleVector_),
      stopRdmBuilderDelegate_
      (new CkwsPlusStopRdmBuilderDelegate(*stopRdmBuilderDelegate_)),
      parser_(NULL)
    {
    }

    // ======================================================================


    Planner::~Planner()
    {
      delete stopRdmBuilderDelegate_;
      if (parser_)
	delete parser_;
    }

    // ======================================================================

    ktStatus Planner::addHppProblem(CkppDeviceComponentShPtr robot,
				    double penetration)
    {
      Problem hppProblem(robot, obstacleVector_, penetration);

      hppDout(info, "Adding a problem in vector");
      // Add robot in vector .
      problemVector_.push_back(hppProblem);


      CkitNotificationShPtr notification
	= CkitNotification::createWithPtr<Planner>
	(Planner::ID_HPP_ADD_ROBOT, this);
      // set attribute if necessary
      notification->shPtrValue<CkppDeviceComponent>(ROBOT_KEY, robot);
      notificator_->notify(notification);


      return KD_OK;
    }

    // ======================================================================

    ktStatus Planner::removeHppProblem()
    {

      if(problemVector_.size()){
	problemVector_.pop_back();
	obstacleVector_.clear();
	return KD_OK;
      }

      hppDout(error, "No problem to remove.");
      return KD_ERROR;

    }

    // ======================================================================

    ktStatus
    Planner::addHppProblemAtBeginning(CkppDeviceComponentShPtr robot,
				      double penetration)
    {
      Problem hppProblem(robot, obstacleVector_, penetration);

      hppDout(info, "Adding a problem");
      // Add robot in vector .
      problemVector_.push_front(hppProblem);

      CkitNotificationShPtr notification  =
	CkitNotification::createWithPtr<Planner>(Planner::ID_HPP_ADD_ROBOT,
						 this);
      // set attribute if necessary
      notification->shPtrValue<CkppDeviceComponent>(ROBOT_KEY, robot);
      notificator_->notify(notification);


      return KD_OK;
    }

    // ======================================================================

    ktStatus Planner::removeHppProblemAtBeginning(){

      if(problemVector_.size()){
	problemVector_.pop_front();
	obstacleVector_.clear();
	return KD_OK;
      }

      hppDout(error, "No problem to remove.");
      return KD_ERROR;

    }


    // ======================================================================

    const CkppDeviceComponentShPtr
    Planner::robotIthProblem(unsigned int rank) const
    {
      CkppDeviceComponentShPtr nullShPtr;

      if (rank < getNbHppProblems()) {
	return problemVector_[rank].getRobot();
      }
      return nullShPtr;
    }

    // ======================================================================

    CkwsConfigShPtr Planner::robotCurrentConfIthProblem(unsigned int rank) const
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

    // ======================================================================

    ktStatus Planner::robotCurrentConfIthProblem(unsigned int rank,
						 const CkwsConfigShPtr& config)
    {
      ktStatus status = KD_ERROR;

      if (rank < getNbHppProblems()) {
	// status = problemVector_[rank].getRobot()->setCurrentConfig(config);
	// upadate it to use kppDeviceComponent
	status = problemVector_[rank].getRobot()->applyCurrentConfig(config);
      }

      /* send notification */
      if (status == KD_OK) {
	CkitNotificationShPtr notification
	  = CkitNotification::createWithPtr<Planner>
	  (Planner::ID_HPP_SET_CURRENT_CONFIG, this);
	notification->shPtrValue<CkwsConfig>(CONFIG_KEY, config);
	notificator_->notify(notification);
      }

      return status;
    }

    // ======================================================================

    ktStatus Planner::robotCurrentConfIthProblem(unsigned int rank,
						 const CkwsConfig& config)
    {
      ktStatus status = KD_ERROR;

      if (rank < getNbHppProblems()) {
	status = problemVector_[rank].getRobot()->setCurrentConfig(config);
      }


      // If success, send notification
      /* send notification */
      if (status == KD_OK) {
	CkitNotificationShPtr notification
	  = CkitNotification::createWithPtr<Planner>
	  (Planner::ID_HPP_SET_CURRENT_CONFIG, this);
	notificator_->notify(notification);
	// set attribute if necessary

      }

      return status;
    }

    // ======================================================================

    CkwsConfigShPtr Planner::initConfIthProblem(unsigned int rank) const
    {
      CkwsConfigShPtr config;

      if (rank < getNbHppProblems()) {
	config = problemVector_[rank].initConfig();
      }
      else {
	hppDout(error, "Wrong problem id");
      }

      return config;
    }

    // ======================================================================

    ktStatus Planner::initConfIthProblem(unsigned int rank,
					 const CkwsConfigShPtr config)
    {
      ktStatus status = KD_ERROR;

      if (rank < getNbHppProblems()) {
	status = problemVector_[rank].initConfig(config);
      }
      else {
	hppDout(error, "Wrong problem id");
      }

      return status;
    }

    // ======================================================================

    CkwsConfigShPtr Planner::goalConfIthProblem(unsigned int rank) const
    {
      CkwsConfigShPtr config;

      if (rank < getNbHppProblems()) {
	config = problemVector_[rank].goalConfig();
      }
      else {
	hppDout(error, "Wrong problem id");
      }

      return config;
    }

    // ======================================================================

    ktStatus Planner::goalConfIthProblem(unsigned int rank,
					 const CkwsConfigShPtr config)

    {
      ktStatus status = KD_ERROR;

      if (rank < getNbHppProblems()) {
	status = problemVector_[rank].goalConfig(config);
      }
      else {
	hppDout(error, "Wrong problem id");
      }

      return status;
    }

    // ======================================================================


    ktStatus
    Planner::roadmapBuilderIthProblem(unsigned int rank,
				      CkwsRoadmapBuilderShPtr roadmapBuilder,
				      bool display)
    {
      if (rank >= getNbHppProblems()) {
	hppDout(error, "rank should be less than number of problems.");
	return KD_ERROR;
      }

      //If a roadmap was already stored, it will be removed. If this roadmap is
      //displayed in the interface, we need first to remove the corresponding
      //data-structure in the interface. This is done by sending a notification.
      CkitNotificationShPtr notification  =
	CkitNotification::createWithPtr<Planner>
	(Planner::ID_HPP_REMOVE_ROADMAPBUILDER, this);
      notification->unsignedIntValue(Planner::ROADMAP_KEY, rank);
      notificator_->notify(notification);

      //Add an interruption delegate to the roadmap builder
      Problem& hppProblem =  problemVector_[rank];
      hppProblem.roadmapBuilder(roadmapBuilder);

      //If the new roadmap is displayed in the interface, send a notification to
      //trigger appropriate action.
      if (display) {
	notification =
	  CkitNotification::createWithPtr<Planner>
	  (Planner::ID_HPP_ADD_ROADMAPBUILDER, this);
	notification->unsignedIntValue(Planner::ROADMAP_KEY, rank);
	notificator_->notify(notification);
      }

      return KD_OK;
    }

    // ======================================================================

    CkwsRoadmapBuilderShPtr Planner::roadmapBuilderIthProblem(unsigned int rank)
    {
      CkwsRoadmapBuilderShPtr roadmapBuilder;
      if (rank < getNbHppProblems()) {
	Problem& hppProblem =  problemVector_[rank];
	roadmapBuilder = hppProblem.roadmapBuilder();
      }
      return roadmapBuilder;
    }

    // ======================================================================

    ktStatus
    Planner::pathOptimizerIthProblem(unsigned int rank,
				     CkwsPathOptimizerShPtr pathOptimizer)
    {
      if (rank >= getNbHppProblems()) {
	hppDout(error, "rank should be less than number of problems.");
	return KD_ERROR;
      }

      Problem& hppProblem =  problemVector_[rank];
      hppProblem.pathOptimizer(pathOptimizer);

      return KD_OK;
    }


    // ======================================================================

    CkwsPathOptimizerShPtr Planner::pathOptimizerIthProblem(unsigned int rank)
    {
      CkwsPathOptimizerShPtr pathOptimizer;
      if (rank < getNbHppProblems()) {
	Problem& hppProblem =  problemVector_[rank];
	pathOptimizer = hppProblem.pathOptimizer();
      }
      return pathOptimizer;
    }

    // ======================================================================


    ktStatus
    Planner::steeringMethodIthProblem(unsigned int rank,
				      CkwsSteeringMethodShPtr sm)
    {

      if (rank >= getNbHppProblems()) {
	hppDout(error, "Rank should be less than number of problems.");
	return KD_ERROR;
      }

      problemVector_[rank].getRobot()->steeringMethod(sm) ;

      return KD_OK ;
    }

    // ======================================================================

    CkwsSteeringMethodShPtr Planner::steeringMethodIthProblem(unsigned int rank)
    {

      CkwsSteeringMethodShPtr sm ;
      if (rank < getNbHppProblems()) {
	sm = problemVector_[rank].getRobot()->steeringMethod() ;
      }
      return sm ;

    }

    // ======================================================================

    ktStatus Planner::configExtractorIthProblem(unsigned int rank,
						const CkwsConfigExtractorShPtr&
						inConfigExtractor)
    {
      if (rank >= getNbHppProblems()) {
	hppDout(error, "Rank should be less than number of problems.");
	return KD_ERROR;
      }

      problemVector_[rank].configExtractor(inConfigExtractor);
      return KD_OK ;
    }

    // ======================================================================

    ktStatus Planner::penetration(unsigned int rank, double penetration)
    {
      if (rank >= getNbHppProblems()) {
	hppDout(error, "Rank should be less than number of problems.");
	return KD_ERROR;
      }
      problemVector_[rank].penetration(penetration);
      return KD_OK;
    }

    // ======================================================================

    double Planner::penetration(unsigned int rank) const
    {
      if (rank >= getNbHppProblems()) {
	hppDout(error, "Rank should be less than number of problems.");
	return KD_ERROR;
      }
      return   problemVector_[rank].penetration();
    }

    // ======================================================================

    ktStatus Planner::obstacleList(std::vector<CkcdObjectShPtr> collisionList)
    {
      // Send notification to destroy current obstacles in Kpp.

      CkitNotificationShPtr notification  =
	CkitNotification::createWithPtr<Planner>
	(Planner::ID_HPP_REMOVE_OBSTACLES, this);
      notificator_->notify(notification);

      // Set list of obstacles.
      obstacleVector_ = collisionList;

      // Set the list of obstacles for each robot.
      unsigned int nProblem = getNbHppProblems();
      for (unsigned int iProblem=0; iProblem<nProblem; iProblem++) {
	Problem& problem = problemVector_[iProblem];
	problem.obstacleList(collisionList);
      }

      // Send notification for new list of obstacles.
      notification = CkitNotification::createWithPtr<Planner>
	(Planner::ID_HPP_SET_OBSTACLE_LIST, this);
      // set attributes if necessary
      notification->ptrValue< std::vector<CkcdObjectShPtr> >
	(OBSTACLE_KEY, &obstacleVector_);
      notificator_->notify(notification);

      return KD_OK;
    }

    // ======================================================================

    const std::vector< CkcdObjectShPtr > Planner::obstacleList()
    {
      return obstacleVector_;
    }

    // ======================================================================

    ktStatus Planner::addObstacle(CkcdObjectShPtr object,
				  bool distanceComputation)
    {
      obstacleVector_.push_back(object);

      // Set the list of obstacles for each robot.
      unsigned int nProblem = getNbHppProblems();
      for (unsigned int iProblem=0; iProblem<nProblem; iProblem++) {
	Problem& problem = problemVector_[iProblem];
	problem.addObstacle(object, distanceComputation);
      }

      // Send notification
      CkitNotificationShPtr notification =
	CkitNotification::createWithPtr<Planner>
	(Planner::ID_HPP_ADD_OBSTACLE, this);
      // set attributes if necessary
      notification->ptrValue< std::vector<CkcdObjectShPtr> >
	(OBSTACLE_KEY, &obstacleVector_);
      notificator_->notify(notification);

      return KD_OK;
    }

    // ======================================================================

    ktStatus Planner::solveOneProblem(unsigned int rank)
    {

      if (rank >= getNbHppProblems()) {
	hppDout(error, "Problem Id=" << rank
		<< "is bigger than vector size=" << getNbHppProblems());

	return KD_ERROR;
      }

      Problem& hppProblem = problemVector_[rank];

      return hppProblem.solve();
    }

    ktStatus
    Planner::optimizePath(unsigned int problemId, unsigned int pathId)
    {
      if (problemId >= getNbHppProblems()) {
	hppDout(error, "Problem Id=" << problemId
		<< " is bigger than vector size=" << getNbHppProblems());

	return KD_ERROR;
      }

      Problem& hppProblem = problemVector_[problemId];
      CkwsRoadmapBuilderShPtr roadmapBuilder = hppProblem.roadmapBuilder();
      if (!roadmapBuilder) {
	hppDout(error,"Roadmap builder should be set to define penetration.");
	return KD_ERROR;
      }
      double penetration = hppProblem.penetration();

      if (pathId >= hppProblem.getNbPaths()) {
	hppDout(error, "Problem Id="
		<< pathId << " is bigger than number of paths="
		<< hppProblem.getNbPaths());
	return KD_ERROR;
      }
      CkwsPathShPtr kwsPath = hppProblem.getIthPath(pathId);

      // optimizer for the path
      if (hppProblem.pathOptimizer()) {
	hppProblem.pathOptimizer()->optimizePath(kwsPath, penetration);

	hppDout(info, "Path optimized with penetration "
		<< penetration);

      } else {
	hppDout(error, "No optimizer defined");
      }
      return KD_OK;
    }

    // ======================================================================

    unsigned int Planner::getNbPaths(unsigned int problemId) const
    {
      unsigned int nbPaths = 0;
      if (problemId < getNbHppProblems()) {
	nbPaths =  problemVector_[problemId].getNbPaths();
      } else {
	hppDout(error, "ProblemId = "<< problemId
		<< " should be smaller than nb of problems: "
		<< getNbHppProblems());
      }
      return nbPaths;
    }

    // ======================================================================

    CkwsPathShPtr Planner::getPath(unsigned int problemId,
                                   unsigned int pathId) const
    {
      CkwsPathShPtr resultPath;

      if (problemId < getNbHppProblems()) {
	if (pathId < problemVector_[problemId].getNbPaths()) {
	  resultPath = problemVector_[problemId].getIthPath(pathId);
	}
      }
      return resultPath;
    }

    // ======================================================================

    ktStatus Planner::addPath(unsigned int problemId, CkwsPathShPtr kwsPath)
    {
      if (problemId >= problemVector_.size()) {
	hppDout(error, "ProblemId bigger than vector size.");
	return KD_ERROR;
      }
      problemVector_[problemId].addPath(kwsPath);
      return KD_OK;
    }

    // ======================================================================

    CkwsKCDBodyConstShPtr
    Planner::findBodyByJointName(const std::string& inJointName) const
    {
      CkwsKCDBodyConstShPtr kcdBody;
      unsigned int nbProblems = getNbHppProblems();

      // Loop over hppProblem.
      for (unsigned int iProblem=0; iProblem < nbProblems; iProblem++) {
	CkwsDevice::TJointVector jointVector;
	const CkppDeviceComponentShPtr hppRobot = robotIthProblem(iProblem);
	hppRobot->getJointVector(jointVector);

	// Loop over bodies of the robot.
	for (CkwsDevice::TJointIterator jointIter=jointVector.begin();
	     jointIter < jointVector.end();
	     jointIter++) {
	  // Cast joint into CkppJointComponent
	  if (CkppJointComponentShPtr kppJointInRobot =
	      KIT_DYNAMIC_PTR_CAST(CkppJointComponent, *jointIter)) {
	    if (kppJointInRobot->name() == inJointName) {
	      kcdBody =
		KIT_DYNAMIC_PTR_CAST(const CkwsKCDBody,
				     kppJointInRobot->kwsJoint()->attachedBody());
	      return kcdBody;
	    }
	  } else {
	    hppDout(error, "One body of robot in hppProblem "
		    << iProblem << " is not a CkppJointComponent.");
	  }
	}
      }
      return kcdBody;
    }


    ktStatus Planner::solve()
    {
      ktStatus success=KD_OK;
      for (unsigned int iProblem=0; iProblem<getNbHppProblems(); iProblem++) {
	if (solveOneProblem(iProblem) != KD_OK) {
	  success = KD_ERROR;
	}
      }
      return success;
    }

    void Planner::interruptPathPlanning()
    {
      if (stopRdmBuilderDelegate_ == NULL) {
	hppDout(error, "No stop delegate.");
	return;
      }
      stopRdmBuilderDelegate_->shouldStop(true);
    }

    ktStatus Planner::parseFile(const std::string& inFileName)
    {

      CkppComponentShPtr modelTreeComponent;
      CkprParserManagerShPtr parser = CkprParserManager::defaultManager();

      if(inFileName.length() == 0) {
	hppDout(error, "No file name");
	return KD_ERROR;
      }

      if(KD_ERROR == parser->loadComponentFromFile(inFileName,
						   modelTreeComponent)) {
	hppDout(error, "Unable to load file " << inFileName);
	hppDout(error, parser->lastError().errorMessage());
	return KD_ERROR;
      }

      CkppModelTreeShPtr modelTree =
	KIT_DYNAMIC_PTR_CAST(CkppModelTree,modelTreeComponent);

      CkppSolidComponentShPtr solidComponent =
	KIT_DYNAMIC_PTR_CAST(CkppSolidComponent,modelTreeComponent);

      if(modelTree) {
	hppDout(error, "Found modelTree");


	std::map<CkppDeviceComponentShPtr,unsigned int> devicesIndex;

	std::set<CkcdObjectShPtr> devicesBodies;

	unsigned int currentRank=0;

	if(!modelTree->deviceNode()) std::cout << "No devices" << std::endl;
	else {
	  for(unsigned int i = 0;
	      i< modelTree->deviceNode()->countChildComponents(); i++) {

	    CkppDeviceComponentShPtr deviceComponent =
	      KIT_DYNAMIC_PTR_CAST(CkppDeviceComponent,
				   modelTree->deviceNode()->childComponent(i));

	    if(deviceComponent) {
	      // If device is of type hpp::model::HumanoidRobot, initialize
	      // kinematic chain.
	      hpp::model::HumanoidRobotShPtr humanoidRobot =
		KIT_DYNAMIC_PTR_CAST(hpp::model::HumanoidRobot,
				     deviceComponent);
	      if (humanoidRobot) {
		humanoidRobot->initialize();
		// Set config to 0
		CkwsConfig config(deviceComponent);
		deviceComponent->setCurrentConfig(config);
	      }
	      addHppProblem(humanoidRobot, HPPPLANNER_DEFAULT_PENETRATION);
	      devicesIndex.insert
		(std::pair<CkppDeviceComponentShPtr,unsigned int>(humanoidRobot,
								  currentRank));

	      std::vector< CkppSolidComponentRefShPtr > solidComponentRefVector;

	      humanoidRobot->getSolidComponentRefVector (solidComponentRefVector);

	      for(std::vector< CkppSolidComponentRefShPtr >::iterator
		    it=solidComponentRefVector.begin();
		  it!=solidComponentRefVector.end();it++) {
		CkcdObjectShPtr kcdObject =
		  KIT_DYNAMIC_PTR_CAST(CkcdObject,
				       (*it)->referencedSolidComponent());
		if(kcdObject) devicesBodies.insert(kcdObject);
		else std::cout << "Cannot cast component to kcdObject"
			       << std::endl;
	      }
	      currentRank++;
	    }
	  }
	}
	if(!modelTree->geometryNode()) std::cout<<"No geometries"<<std::endl;
	else{
	  hppDout(info, "geometries");
	  for(unsigned int i = 0;
	      i< modelTree->geometryNode()->countChildComponents(); i++){
	    CkppComponentShPtr child =
	      modelTree->geometryNode()->childComponent(i);
	    CkcdObjectShPtr kcdObject =
	      KIT_DYNAMIC_PTR_CAST(CkcdObject, child);
	    hppDout(info, child->name());
	    if(kcdObject) addObstacle(kcdObject);
	    else std::cerr << "Cannot cast component to kcdObject" << std::endl;
	  }
	}

	if(!modelTree->pathNode()) std::cout<<"No paths"<<std::endl;
	else {
	  for(unsigned int i = 0; i< modelTree->pathNode()->countChildComponents();
	      i++){
	    CkppPathComponentShPtr pathComponent =
	      KIT_DYNAMIC_PTR_CAST(CkppPathComponent,
				   modelTree->pathNode()->childComponent(i));

	    std::map<CkppDeviceComponentShPtr,unsigned int>::iterator it =
	      devicesIndex.find(pathComponent->deviceComponent());

	    if(it==devicesIndex.end()) {
	      hppDout(error, "No device matching path");
	      return KD_ERROR;
	    }

	    unsigned int rank=it->second;

	    addPath(rank,pathComponent->kwsPath());

	  }

	}
      }


      else if(solidComponent)
	{
	  hppDout(error, "Found solid component");
	  CkcdObjectShPtr kcdObject =
	    KIT_DYNAMIC_PTR_CAST(CkcdObject,solidComponent);
	  if(!kcdObject) {
	    hppDout(error, "Cannot cast solidComponent to kcdObject");
	    return KD_ERROR;
	  }
	  addObstacle(kcdObject);
	}

      else {
	hppDout(error, "Cannot cast file neither to CkppSolidComponent "
		<< "nor to CkppModelTree ");
	return KD_ERROR;
      }

      hppDout(error, "Everything is fine ");
      return KD_OK ;
    }
  } // namespace core
} // namespace namespace hpp
