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

#ifndef HPP_CORE_PROBLEM_HH
#define HPP_CORE_PROBLEM_HH

#include "KineoWorks2/kwsPathOptimizer.h"
#include "KineoWorks2/kwsRoadmapBuilder.h"
#include "KineoWorks2/kwsConfig.h"

#include "KineoModel/kppDeviceComponent.h"
#include "kcd2/kcdInterface.h"
#include "kwsKcd2/kwsKCDBody.h"

#include "KineoUtility/kitNotificator.h"

#include <hpp/util/deprecated.hh>

KIT_PREDEF_CLASS(CkwsConfigExtractor);

namespace hpp {
  namespace core {
    /// \brief Defines a path planning problem for one robot.
    /// A path planning problem is defined by
    /// \li a robot: a KineoWorks device,
    /// \li a set of obstacles: a list of Kcd objects,
    /// \li a steering method: stored in the KwsDevice,
    /// \li initial and goal configurations,
    /// \li a roadmap : to store a roadmap
    /// \li a roadmapBuilder : roadmap strategy
    /// \li a pathOptimizer : to Optimise the path
    class Problem
    {
    public:
      /// \brief Create a path planning problem.
      /// \param robot robot associated to the path planning problem.
      /// \param penetration dynamic penetration for validating direct paths.
      Problem (CkppDeviceComponentShPtr robot, double penetration);

      ///
      /// \brief Create a path planning problem.
      /// \param robot robot associated to the path planning problem.
      /// \param obstacleList list of obstacle of this problem.
      /// \param penetration dynamic penetration for validating direct paths.
      Problem (CkppDeviceComponentShPtr robot,
	       const std::vector<CkcdObjectShPtr>& obstacleList,
	       double penetration);


      /// \brief Copy constructor
      Problem(const Problem& inProblem);

      /// \name Problem definition
      /// @{

      /// \brief return shared pointer to robot.
      const CkppDeviceComponentShPtr& getRobot() const {
	return robot_;
      };

      /// \brief Get shared pointer to initial configuration.
      const CkwsConfigShPtr& initConfig() const {
	return initConf_;
      };
      /// \brief Set initial configuration.
      ktStatus initConfig ( const CkwsConfigShPtr& inConfig );
      /// \brief Get shared pointer to goal configuration.
      const CkwsConfigShPtr& goalConfig() const {
	return goalConf_;
      };
      /// \brief Set goal configuration.
      ktStatus goalConfig ( const CkwsConfigShPtr& inConfig );
      /// \brief Check that problem is well formulated
      ktStatus checkProblem() const;

      /// @}

      /// \name Obstacles
      /// @{

      /// \brief Store a copy of the list of obstacles.
      /// \param inCollisionList list of obstacles to be taken into account
      /// for this problem.

      /// \note It is not recommended to call this method several times for
      ///	 the same problem.
      ktStatus obstacleList
      (const std::vector<CkcdObjectShPtr>& inCollisionList );

      /// \brief return a shared pointer to the obstacle list
      const std::vector<CkcdObjectShPtr>& obstacleList();

      /// \brief Add obstacle to the list.
      /// \param inObject a new object.
      /// \param distanceComputation whether distance computation should be
      /// performed for this object.

      /// \note Compute collision entities.
      ktStatus addObstacle (const CkcdObjectShPtr& inObject,
			    bool distanceComputation=true);


      /// @}

      /// \name Problem resolution
      ///	 @{

      /// \brief set device steering method
      void steeringMethod ( const CkwsSteeringMethodShPtr &inSteeringMethod );
      /// \brief Get device steering method
      CkwsSteeringMethodShPtr steeringMethod() const;
      /// \brief Set roadmap building strategy.
      void roadmapBuilder ( CkwsRoadmapBuilderShPtr inroadmapBuilder );
      /// \brief Get roadmap building strategy.
      CkwsRoadmapBuilderShPtr roadmapBuilder() const;
      /// \brief set roadmap.
      /// \param inRoadmap shared pointer to a roadmap.
      void roadmap ( CkwsRoadmapShPtr inRoadmap ) ;
      /// \brief Get roadmap.
      /// \return shared pointer to the roadmap.
      CkwsRoadmapShPtr roadmap() const;
      /// \brief Set pathOptimizer.
      /// \param pathOptimizer path optimizer.
      void pathOptimizer ( CkwsPathOptimizerShPtr pathOptimizer );
      /// \brief Get path optimizer
      /// \return shared pointer to the path optimiser
      CkwsPathOptimizerShPtr pathOptimizer() ;

      /// \brief Determine whether the path optimizer should always be called

      /// In the default behaviour, Problem::solve() does not call the path
      /// optimizer if the path resulting from path planning includes one single
      /// direct path. This behaviour can be changed by calling this function
      /// with true as an argument.
      void alwaysOptimize(bool inAlwaysOptimize) {
	alwaysOptimize_ = inAlwaysOptimize;
      };

      /// \brief Set configuration extractor
      /// \param inConfigExtractor
      void configExtractor(const CkwsConfigExtractorShPtr& inConfigExtractor);

      /// \brief Get configuration extractor
      const CkwsConfigExtractorShPtr& configExtractor();

      /// \brief Set dynamic penetration of given problem
      /// \param penetration dynamic penetration for validating direct paths.
      void penetration(double penetration);

      /// \brief Get dynamic penetration of given problem
      /// \return dynamic penetration allowed for validating a direct path.
      double penetration() const;

      /// \brief Solve the problem

      /// This function successively performs the following steps
      /// \li check that the problem is well defined,
      /// \li solve the path planning problem
      /// \li optimize the resulting path if success.

      /// The path planning step performs the following operations:
      /// \li call the steering method and validate the resulting path,
      /// \li if failure, call the roadmap builder.

      /// Unless otherwise specified (see Problem::alwaysOptimize()), a path
      /// containing only one direct path is not optimized.

      /// \return KD_OK if success, KD_ERROR if failure
      ktStatus solve();

      /// \brief Add a path to the vector.
      /// \xrefitem <send-notif> "Notification" "Send Notification"
      /// Send ID_HPP_ADD_PATH.
      void addPath ( CkwsPathShPtr kwsPath );
      /// \brief Get I-th path in vector
      CkwsPathShPtr getIthPath ( unsigned int pathId ) const;
      /// \brief et number of paths in vector
      unsigned int getNbPaths() const;

      /// \brief et initialize the map of innerBody (mapOuter_)
      void initMapOuter();
      /// @}
    private :
      /// \brief Validate configuration and track validation reports.
      ktStatus validateConfig(CkppDeviceComponentShPtr inDevice,
			      const CkwsConfigConstShPtr& inConfig) const;

      /// \brief Validate initial configuration

      /// If initial configuration is not valid, and if a config extractor
      /// is set, try to build an extraction path.
      ktStatus validateInitConfig(CkwsConfigShPtr& inOutInitConfig,
				  CkwsPathShPtr& inOutPath) const;

      /// \brief Plan a path between initial and goal configurations

      /// First, tries a direct path and then calls the roadmap builder.
      ktStatus planPath(const CkwsConfigConstShPtr inInitConfig,
			const CkwsConfigConstShPtr inGoalConfig,
			const CkwsPathShPtr& inOutPath);

      /// \brief Set penetration of collision direct path validator of the robot
      void setPenetration();
      /// \brief pointer to a KineoWorks notificator.
      CkitNotificatorShPtr notificator_;
      /// \brief the robot is a KineoWorks Device.
      CkppDeviceComponentShPtr robot_;
      /// \brief Shared pointer to initial configuration.
      CkwsConfigShPtr initConf_;
      /// \brief Shared pointer to goal configuration.
      CkwsConfigShPtr goalConf_;
      /// \brief Shared pointer to a roadmap associate to the robot
      CkwsRoadmapShPtr roadmap_;
      /// \brief Shared pointer to a roadmapBuilder
      CkwsRoadmapBuilderShPtr roadmapBuilder_;
      /// \brief Shared pointer to a optimizer for the path
      CkwsPathOptimizerShPtr pathOptimizer_;

      /// \brief Configuration extractor

      /// A configuration extractor attempts at extracting a
      /// collision-free configuration in the neighborhood
      /// of the initial configuration of a path planning
      /// problem when the initial configuration is in
      /// collision.
      CkwsConfigExtractorShPtr configExtractor_;

      /// \brief Get the list of obstacle of this problem.

      /// The set of obstacles is likely to be a copy or reference to
      /// the list of obstacles of Planner.
      /// However, to allow more general motion planning strategies, we
      /// leave to possiblity to define it differently. Obstacles are a
      /// list of KCD objects.
      std::vector< CkcdObjectShPtr > obstacleVector_;
      /// \brief A vector of paths corresponding to this problem.
      std::vector<CkwsPathShPtr> pathVector_;
      /// \brief A map of each body to the a vector of its outer bodies.
      std::map<CkwsKCDBodyShPtr,std::vector<CkcdObjectShPtr> > mapOuter_;
      /// \brief Penetration
      double penetration_;
      /// \brief Whether the path optimizer should be called anyway
      bool alwaysOptimize_;

    public:
      // for notification:
      static const CkitNotification::TType  ID_HPP_ADD_PATH;

      // key to retrieve
      static const std::string   PATH_KEY;
      static const std::string   PATH_ID_KEY;
      static const std::string   DEVICE_KEY;

    }; // class Problem
  } // namespace core
} // namespace hpp
typedef hpp::core::Problem ChppProblem HPP_DEPRECATED;
#endif // HPP_CORE_PROBLEM_HH
