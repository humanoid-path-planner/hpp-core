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

#include <iostream>

#include <hpp/util/debug.hh>
#include <hpp/model/device.hh>
#include <hpp/model/body-distance.hh>
#include <hpp/core/problem.hh>
#include <hpp/kwsio/configuration.hh>

#include "KineoWorks2/kwsConfigExtractor.h"
#include "KineoWorks2/kwsValidatorDPCollision.h"
#include "KineoWorks2/kwsValidatorSet.h"
#include "KineoWorks2/kwsRoadmap.h"
#include "KineoWorks2/kwsNode.h"
#include "KineoWorks2/kwsEdge.h"
#include "KineoWorks2/kwsReportCfgDof.h"
#include "KineoWorks2/kwsSteeringMethod.h"
#include "KineoWorks2/kwsPathPlanner.h"
#include <kwsKcd2/kwsKCDBodyAdvanced.h>
#include "KineoModel/kppSteeringMethodComponent.h"

namespace hpp {
  namespace core {
    const CkitNotification::TType
    Problem::ID_HPP_ADD_PATH ( CkitNotification::makeID() );

    const std::string Problem::PATH_KEY ( "path" );
    const std::string Problem::PATH_ID_KEY ( "path_id" );
    const std::string Problem::DEVICE_KEY ( "device" );


    // ======================================================================

    Problem::Problem ( CkppDeviceComponentShPtr robot, double penetration ) :
      notificator_ ( CkitNotificator::defaultNotificator() ),
      robot_ ( robot ),
      penetration_(penetration),
      alwaysOptimize_(false)
    {
      // Set the input penetration in the direct path collision
      // validator of the robot
      setPenetration();
      initMapOuter();
    }

    // ======================================================================

    Problem::Problem (CkppDeviceComponentShPtr robot,
		      const std::vector<CkcdObjectShPtr>& obstacles,
		      double penetration) :
      notificator_ ( CkitNotificator::defaultNotificator() ),
      robot_ ( robot ),
      penetration_(penetration),
      alwaysOptimize_(false)
    {
      // Set the input penetration in the direct path collision
      // validator of the robot
      setPenetration();
      initMapOuter();
      obstacleList ( obstacles );
    }

    // ======================================================================

    Problem::Problem(const Problem& inProblem) :
      notificator_(inProblem.notificator_),
      robot_(inProblem.robot_),
      initConf_(inProblem.initConf_),
      goalConfigurations_(inProblem.goalConfigurations_),
      roadmapBuilder_(inProblem.roadmapBuilder_),
      pathOptimizer_(inProblem.pathOptimizer_),
      configExtractor_(inProblem.configExtractor_),
      obstacleVector_(inProblem.obstacleVector_),
      pathVector_(inProblem.pathVector_),
      mapOuter_(inProblem.mapOuter_),
      penetration_(inProblem.penetration_),
      alwaysOptimize_(inProblem.alwaysOptimize_)
    {
    }

    // ======================================================================

    void Problem::initConfig ( const CkwsConfigShPtr& inConfig )
    {
      assert (inConfig->device() == robot_);
      initConf_ = inConfig;
    }

    // ======================================================================

    void Problem::addGoalConfig (const CkwsConfigShPtr& inConfig)
    {
      assert (inConfig->device () == robot_);
      goalConfigurations_.push_back (inConfig);
    }

    // ======================================================================

    const std::vector<CkwsConfigShPtr>& Problem::goalConfigurations () const
    {
      return goalConfigurations_;
    }

    // ======================================================================

    void Problem::resetGoalConfig ()
    {
      goalConfigurations_.clear ();
      if (roadmapBuilder_) roadmapBuilder_->resetGoalNodes ();
    }

    // ======================================================================

    void Problem::initMapOuter()
    {
      CkwsDevice::TBodyVector bodyVector;
      robot_->getBodyVector ( bodyVector );

      // Loop over bodies of robot.
      for ( CkwsDevice::TBodyIterator bodyIter = bodyVector.begin(); bodyIter < bodyVector.end(); bodyIter++ )
	{
	  // Try to cast body into CkwsKCDBodyAdvanced
	  CkwsKCDBodyAdvancedShPtr kcdBody;
	  if ( kcdBody = boost::dynamic_pointer_cast<CkwsKCDBodyAdvanced>
	       ( *bodyIter ) )
	    {
	      // Copy list of outer objects
	      mapOuter_[kcdBody]= kcdBody->obstacleObjects();
	    }
	}
    }

    // ======================================================================

    ktStatus
    Problem::obstacleList(const std::vector<CkcdObjectShPtr>& inCollisionList)
    {
      for (std::vector<CkcdObjectShPtr>::const_iterator it =
	     inCollisionList.begin();
	   it < inCollisionList.end();
	   it++) {
	addObstacle(*it, true);
      }
      return KD_OK;
    }

    // ======================================================================

    const std::vector<CkcdObjectShPtr>& Problem::obstacleList()
    {
      return obstacleVector_;
    }

    // ======================================================================

    ktStatus Problem::addObstacle(const CkcdObjectShPtr& inObject,
				  bool distanceComputation)
    {
      // Add object in local list
      obstacleVector_.push_back(inObject);

      // Retrieve hpp device.
      model::DeviceShPtr hppRobot = KIT_DYNAMIC_PTR_CAST(model::Device, robot_);
      if (!hppRobot)
	{
	  hppDout (notice, "Robot is not HPP Device.");

	  // Get robot vector of bodies.
	  CkwsDevice::TBodyVector bodyVector;
	  robot_->getBodyVector ( bodyVector );

	  // Loop over bodies of robot.
	  for ( CkwsDevice::TBodyIterator bodyIter = bodyVector.begin();
		bodyIter < bodyVector.end();
		bodyIter++ )
	    if (CkwsKCDBodyAdvancedShPtr kcdBody =
		KIT_DYNAMIC_PTR_CAST(CkwsKCDBodyAdvanced, *bodyIter))
	      {
		hppDout(info,"CkwsKCDBodyAdvanced type.");
		// Append object at the end of KineoWorks set of outer
		// objects for collision checking.
		std::vector<CkcdObjectShPtr> outerList = kcdBody->obstacleObjects();
		outerList.push_back(inObject);
		kcdBody->obstacleObjects(outerList);
	      }
	}
      else
	// Loop over body distances of robot.
	BOOST_FOREACH(model::BodyDistanceShPtr bodyDistance,
		      hppRobot->bodyDistances ())
	  {
	    bodyDistance->addOuterObject (inObject, distanceComputation);
	  }

      return KD_OK;
    }

    // ======================================================================

    void Problem::steeringMethod
    (const CkppSteeringMethodComponentShPtr& inSteeringMethod)
    {
      robot_->steeringMethodComponent (inSteeringMethod);
    }

    // ======================================================================

    CkppSteeringMethodComponentShPtr Problem::steeringMethod() const
    {
      return robot_->steeringMethodComponent();
    }

    // ======================================================================

    ktStatus Problem::checkProblem() const
    {
      if (!getRobot()) {
	hppDout(error, "No device in problem.");
	return KD_ERROR;
      }

      if (!initConfig()) {
	hppDout(error,"No init config in problem.");
	return KD_ERROR;
      }
      if (!goalConfigurations ().size ()) {
	hppDout(error,"No goal config in problem.");
	return KD_ERROR;
      }

      if(!roadmapBuilder()){
	hppDout(error,"Define a roadmap builder with penetration");
	return KD_ERROR;
      }

      if (!steeringMethod()) {
	hppDout(error,"Define a steering method.");
	return KD_ERROR;
      }
      // Test that goal configurations are valid
      for (goalConfigConstIterator_t it = goalConfigurations_.begin ();
	   it != goalConfigurations_.end (); it++) {
	CkwsConfigShPtr goalConf (*it);
	if (validateConfig(getRobot(), goalConf) != KD_OK) {
	  hppDout(error,"Found one goal configuration not valid.");
	  return KD_ERROR;
	}
      }
      return KD_OK;
    }

    // ======================================================================

    ktStatus Problem::validateInitConfig(CkwsConfigShPtr& inOutInitConfig,
					 CkwsPathShPtr& inOutPath) const
    {
      if (validateConfig(getRobot(), initConfig()) == KD_OK) {
	hppDout(info,"Initial configuration is valid.");
	return KD_OK;
      }
      /*
	If initial configuration is not valid and configuration extractor
	has been set, try to extract a valid configuration in the neighborhood
	of the initial configuration.
      */
      if (CkwsConfigExtractorShPtr confExtractor = configExtractor_) {
	hppDout(error, "ConfigExtractor->minRadius = "
		<< confExtractor->minRadius());
	hppDout(info, "ConfExtractor->maxRadius = "
		<< confExtractor->maxRadius());
	hppDout(info, "ConfigExtractor->scaleFactor = "
		<< confExtractor->scaleFactor());
	CkwsPathShPtr initConfigPath =
	  CkwsPath::createWithConfig(*inOutInitConfig);

	hppDout(info, "Number of configurations in initConfigPath = "
		<< initConfigPath->countConfigurations());

	if (confExtractor->plan(initConfigPath, CkwsPathPlanner::STABLE_START,
				inOutPath) == KD_OK) {
	  hppDout(info, "Number of configurations in extracted path = "
		  << inOutPath->countConfigurations());

	  // Replace inOutInitConfig by end of extraction path for path
	  // planning problem
	  inOutInitConfig = inOutPath->configAtEnd();
	  if (!inOutInitConfig) {
	    hppDout(error, "No configuration at end of extraction path.");
	    return KD_ERROR;
	  }
	}
	else {
	  hppDout(error, "Failed to extract initial configuration.");
	  return KD_ERROR;
	}
      }
      else {
	hppDout(info, "Initial configuration not valid.");
	return KD_ERROR;
      }
      return KD_OK;
    }

    // ======================================================================

    ktStatus Problem::planPath(const CkwsConfigConstShPtr inInitConfig,
			       const CkwsPathShPtr& inOutPath)
    {
      /*
	Try first a direct path.
      */
      CkppSteeringMethodComponentShPtr steeringMethod =
	getRobot()->steeringMethodComponent ();

      for (goalConfigIterator_t it = goalConfigurations_.begin ();
	   it != goalConfigurations_.end (); it++) {
	CkwsConfigShPtr goalConfig = *it;
	CkwsDirectPathShPtr directPath =
	  steeringMethod->kwsSteeringMethod ()->makeDirectPath
	  (*inInitConfig, *goalConfig);

	if (directPath) {
	  // Retrieve validators of device
	  CkwsValidatorSetConstShPtr dpValidators =
	    getRobot()->directPathValidators();

	  dpValidators->validate(*directPath);
	  if (directPath->isValid()) {

	    hppDout(info,"Problem solved with direct connection. ");

	    // Add direct path to roadmap if not already included

	    CkwsRoadmapShPtr roadmap = roadmapBuilder()->roadmap();
	    roadmap->beginCriticalSectionReadWrite();
	    hppDout(info, "Number of edges in roadmap before inserting nodes = "
		    << roadmap->countEdges());

	    CkwsNodeShPtr startNode = roadmap->nodeWithConfig(*inInitConfig);
	    CkwsNodeShPtr goalNode = roadmap->nodeWithConfig(*goalConfig);

	    hppDout(info,"Number of edges in roadmap after creating nodes = "
		    << roadmap->countEdges());

	    // If start and goal node are not in roadmap, add them.
	    if (!startNode) {
	      startNode = CkwsNode::create(*inInitConfig);
	      if (roadmap->addNode(startNode) != KD_OK) {
		hppDout(error,"Failed to add start node in roadmap.");
		startNode.reset();
	      }
	    }
	    if (!goalNode) {
	      goalNode = CkwsNode::create(*goalConfig);
	      if (roadmap->addNode(goalNode) != KD_OK) {
		hppDout(error,"Failed to add goal node in roadmap.");
		goalNode.reset();
	      }
	    }

	    hppDout(info,"Number of edges in roadmap after adding nodes = "
		    << roadmap->countEdges());

	    if (startNode && goalNode) {
	      // Add edge only if goal node is not accessible from initial node
	      if (!startNode->hasTransitiveOutNode(goalNode)) {
		CkwsEdgeShPtr edge=CkwsEdge::create (directPath);

		if (roadmap->addEdge(startNode, goalNode, edge) == KD_ERROR) {
		  hppDout(info,"Failed to add direct path in roadmap.");
		}
	      }
	      hppDout(info,
		      "Number edges in roadmap after attempt at adding edge = "
		      << roadmap->countEdges());
	    }
	    roadmap->endCriticalSectionReadWrite();
	    inOutPath->appendDirectPath(directPath);
	    // Add the path to vector of paths of the problem.
	    addPath(KIT_DYNAMIC_PTR_CAST(CkwsPath, inOutPath->clone()));
	    return KD_OK;
	  } // if (directPath->isValid())
	  else {
	    hppDout (info, "Failed to validate direct path between: "
		     << *inInitConfig);
	    hppDout (info, "and: " << *goalConfig);
	    std::string validatorName;
	    for (std::size_t i=0; i<directPath->countReports (); ++i) {
	      directPath->report (i, validatorName);
	      hppDout (info, "Validator " << validatorName <<
		       " unvalidated the direct path");
	    }
	  }
	} // if (directPath) {
	else {
	  hppDout (info, "Failed to create direct path between: "
		   << *inInitConfig);
	  hppDout (info, "and: " << *goalConfig);
	}
      } // for (goalConfigIterator_t it = goalConfigurations_.begin ();...

      // solve the problem with the roadmapBuilder
      if (!roadmapBuilder_) {
	hppDout (error, "No roadmap builder.");
	return KD_ERROR;
      }
      // Set init and goal nodes.
      roadmapBuilder_->resetStartNodes ();
      roadmapBuilder_->resetGoalNodes ();
      CkwsNodeShPtr initNode (roadmapBuilder_->roadmapNode (*initConf_));
      roadmapBuilder_->addStartNode (initNode);
      for (goalConfigIterator_t it = goalConfigurations_.begin ();
	   it != goalConfigurations_.end (); it++) {
	CkwsConfigShPtr goalConfig = *it;
	CkwsNodeShPtr goalNode (roadmapBuilder_->roadmapNode (*goalConfig));
	roadmapBuilder_->addGoalNode (goalNode);
      }
      CkwsPathShPtr kwsPath = CkwsPath::create(getRobot());
      std::list< CkwsEdgeShPtr > unusedEdges;
      if(KD_OK == roadmapBuilder()->solveCurrentProblem(kwsPath, unusedEdges)) {
	hppDout(info,"--- Problem solved.----");
      } else {
	hppDout(error,"---- Problem NOT solved.----");
	return KD_ERROR;
      }
      if (!kwsPath) {
	hppDout(error,"No path after successfully solving the problem");
	hppDout(error,"This should not happen.");
	return KD_ERROR;
      }

      if (kwsPath->length()== 0) {
	hppDout(error,
		"Path length is 0 after successfully solving the problem");
	hppDout(error,"This should not happen.");
	return KD_ERROR;
      }

      /*
	Store path before optimization
      */
      if (inOutPath->appendPath(kwsPath) != KD_OK) {
	hppDout(error, "Failed at appending solution path to extraction path.");
	return KD_ERROR;
      }
      addPath(CkwsPath::createCopy(inOutPath));
      return KD_OK;
    }

    // ======================================================================

    ktStatus Problem::solve()
    {
      if (checkProblem() != KD_OK) {
	hppDout(error,"Problem formulation is not correct.");
	return KD_ERROR;
      }

      CkwsConfigShPtr initConf = initConfig();
      /*
	Test that initial configuration is valid
      */
      CkwsPathShPtr solutionPath = CkwsPath::create(getRobot());

      if (validateInitConfig(initConf, solutionPath) != KD_OK) {
	return KD_ERROR;
      }

      if (planPath(initConf, solutionPath) != KD_OK) {
	hppDout(error,"Failed to plan a path between init and goal config.");
	return KD_ERROR;
      }

      /*
	Optimize if
	- there is a path optimizer and
	- the path has more than one direct path or the user has requested
	that optimization is always performed.
      */
      bool shouldOptimize =
	pathOptimizer() && (alwaysOptimize_ ||
			    solutionPath->countDirectPaths() > 1);

      // optimizer for the path
      CkwsPathShPtr optimizedPath;
      if (shouldOptimize) {

	if (pathOptimizer()->plan(solutionPath, CkwsPathPlanner::STABLE_NONE,
				  optimizedPath)
	    == KD_OK) {

	  hppDout(info,"Path optimized with penetration "
		  << penetration_);
	}
	else {
	  hppDout(error,"Path optimization failed.");
	  return KD_ERROR;
	}

      } else {
	hppDout(info,"No optimization performed ");
	return KD_OK;
      }

      if (optimizedPath) {
	hppDout(info,"Number of direct path: "
		<< optimizedPath->countDirectPaths());
	// Add the path to vector of paths of the problem.
	addPath(optimizedPath);
      }

      return KD_OK ;
    }

    // ======================================================================

    void Problem::addPath ( CkwsPathShPtr kwsPath )
    {
      pathVector_.push_back ( kwsPath );
      std::string robotName = robot_->name();

      // notificator_->sendNotification(ID_HPP_ADD_PATH, this);
      CkitNotificationShPtr notification
	= CkitNotification::createWithPtr<Problem> ( Problem::ID_HPP_ADD_PATH, this );
      // set attribute to retreave
      notification->shPtrValue<CkwsPath> ( PATH_KEY, kwsPath );
      notification->shPtrValue<CkppDeviceComponent> ( DEVICE_KEY, robot_ );
      // set path number
      notification->unsignedIntValue ( PATH_ID_KEY, pathVector_.size() );
      notificator_->notify ( notification );


    }

    // ======================================================================

    CkwsPathShPtr Problem::getIthPath ( unsigned int pathId ) const
    {
      CkwsPathShPtr resultPath;

      if ( pathId < pathVector_.size() )
	{
	  resultPath = pathVector_[pathId];
	}
      return resultPath;
    }

    // ======================================================================

    unsigned int Problem::getNbPaths() const
    {
      return pathVector_.size();
    }


    // ======================================================================

    void Problem::roadmapBuilder ( CkwsRoadmapBuilderShPtr roadmapBuilder )
    {

      roadmapBuilder_ = roadmapBuilder;
    }

    // ======================================================================

    CkwsRoadmapBuilderShPtr Problem::roadmapBuilder() const
    {

      return roadmapBuilder_ ;
    }

    // ======================================================================

    CkwsRoadmapShPtr Problem::roadmap() const
    {
      if (roadmapBuilder_) return roadmapBuilder_->roadmap ();
      return CkwsRoadmapShPtr ();
    }

    // ======================================================================

    void Problem::clearRoadmap ()
    {
      if (roadmapBuilder_) {
	if (roadmapBuilder_->roadmap ()) {
	  roadmapBuilder_->roadmap ()->clear ();
	}
	resetGoalConfig ();
      }
    }

    // ======================================================================

    void Problem::pathOptimizer ( CkwsPathPlannerShPtr inOptimizer )
    {

      pathOptimizer_ = inOptimizer ;
    }

    // ======================================================================

    CkwsPathPlannerShPtr Problem::pathOptimizer()
    {
      return pathOptimizer_ ;
    }

    // ======================================================================

    void Problem::configExtractor(const CkwsConfigExtractorShPtr& inConfigExtractor)
    {
      configExtractor_ = inConfigExtractor;
    }

    // ======================================================================

    const CkwsConfigExtractorShPtr& Problem::configExtractor()
    {
      return configExtractor_;
    }

    // ======================================================================

    void Problem::penetration(double penetration)
    {
      penetration_ = penetration;
      setPenetration();
    }

    // ======================================================================

    double Problem::penetration() const
    {
      return penetration_;
    }

    void Problem::setPenetration()
    {
      CkwsValidatorSetConstShPtr dpValidators =
	robot_->directPathValidators();

      /*
	Retrieve collision validator if any and set penetration as penetration
	distance of roadmap builder.
      */
      CkwsValidatorDPCollisionShPtr collisionValidator =
	dpValidators->retrieve<CkwsValidatorDPCollision>();
      if (collisionValidator) {
	collisionValidator->penetration(penetration_);
      }
    }

    // ======================================================================

    ktStatus
    Problem::validateConfig(CkppDeviceComponentShPtr inDevice,
			    const CkwsConfigConstShPtr& inConfig) const
    {
      inDevice->setCurrentConfig (*inConfig);
      inDevice->configValidators()->validate(*inConfig);

      if (inConfig->isValid()) {
	return KD_OK;
      }
      for(unsigned int i=0; i<inConfig->countReports(); ++i) {
	std::string theValidatorName;
	CkwsValidationReportConstShPtr theReport(inConfig->report(i, theValidatorName));
	if(!theReport->isValid()) {
	  hppDout(info, theValidatorName <<
		  " failed at validating the configuration.");

	  // If this is a CkwsDofReport then we can retrieve more information...
	  CkwsReportCfgDofConstShPtr theDofReport;
	  theDofReport = KIT_DYNAMIC_PTR_CAST(CkwsReportCfgDof const, theReport);
	  if(theDofReport) {
	    for(unsigned int j=0; j<theDofReport->countDofs(); ++j) {
	      if(!theDofReport->isDofValid(j)) {
		hppDout(error," Dof #" << j << " is invalid.");
	      }
	    }
	  }

	}
      }
      return KD_ERROR;
    }
  } // namespace core
} // namespace hpp
