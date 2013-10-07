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

#ifndef HPP_CORE_PLANNER_HH
#define HPP_CORE_PLANNER_HH

/*************************************
INCLUDE
**************************************/
#include <deque>

#include <hpp/core/deprecated.hh>

#include "hpp/core/problem.hh"
#include "hpp/core/fwd.hh"

#include <hpp/model/parser.hh>

class CkwsPlusStopRdmBuilderDelegate;
HPP_KIT_PREDEF_CLASS (CkppKCDBody);
HPP_KIT_PREDEF_CLASS (CkwsConfigExtractor);
HPP_KIT_PREDEF_CLASS (CkppSteeringMethodComponent);
HPP_KIT_PREDEF_CLASS (CkwsPathPlanner);
HPP_KIT_PREDEF_CLASS (CkwsRoadmapBuilder);
HPP_KIT_PREDEF_CLASS (CkitNotificator);

namespace hpp {
  namespace core {
    /// \brief Motion planning strategies for humanoid robots among obstacles.

    /// This abstract class defines motion planning strategies for a
    /// humanoid robot in an environment with obstacles.  Different
    /// strategies can be implemented by deriving this class.

    /// Several strategies instantiate several robots and classical
    /// path planning problems. Therefore, this class contains a vector
    /// of Problem describing basic motion planning problems.
    class Planner {
    public:
      /// \brief Constructor
      /// \param addon whether the object is embedded in and addon application.
      /// \note Initialization is slightly different depending on whether the
      /// object is constructed by an addon application of by a plug-in to
      /// Kitelab.
      /// Allocate a KineoWorks CkitNotificator object.
      Planner(bool addon = true);

      /// \brief Copy constructor
      Planner(const Planner& inPlanner);

      /// \brief Destructor
      virtual ~Planner();

      /// \name Problem definition
      /// @{

      /// \brief Add a Problem to the Problem vector with the associed robot.
       /// \param robot A shared pointer to a robot
      /// \param penetration Dynamic penetration for validating direct paths.
      /// \return KD_OK if success, KD_ERROR otherwise

      /// \xrefitem <send-notif> "Notification" "Send Notification"
      /// Send ID_HPP_ADD_ROBOT.
      virtual ktStatus addHppProblem(CkppDeviceComponentShPtr robot,
				     double penetration);

      /// \brief Remove a Problem at the end of the Problem vector.
      /// \return KD_OK if success, KD_ERROR otherwise
      virtual ktStatus removeHppProblem();

      /// \brief Add a Problem at beginning of the Problem vector
      /// \param robot A shared pointer to a robot
      /// \param penetration dynamic penetration for validating direct paths.
      /// \return KD_OK if success, KD_ERROR otherwise

      /// \xrefitem <send-notif> "Notification" "Send Notification"
      /// Send ID_HPP_ADD_ROBOT.
      virtual ktStatus addHppProblemAtBeginning(CkppDeviceComponentShPtr robot,
						double penetration);

      /// \brief Remove a Problem at the beginning the Problem vector.
      /// \return KD_OK if success, KD_ERROR otherwise
      virtual ktStatus removeHppProblemAtBeginning();

      /// \brief Get the problem in the problem vector at the given rank.
      /// \param rank Rank of the hppProblem to retrieve
      /// \return The rank-th problem
      Problem * hppProblem(unsigned int rank) {
	if(rank<getNbHppProblems())
	  return &(problemVector_.at(rank));
	else
	  return NULL;
      }

      /// \brief Get the number of problems in vector.
      /// \return the number of problems in the vector
      unsigned int getNbHppProblems() const {
	return (unsigned int)problemVector_.size();
      }

      /// \brief Get robot at given rank in the Problem vector.
      /// \param rank
      /// \return the shared pointer on the robot
      CkppDeviceComponentShPtr robotIthProblem(unsigned int rank) const;

      /// \brief Get current configuration of i-th robot.
      /// \param rank : Id of problem in vector.
      /// \return : return a copy of the current configuration of robot.
      CkwsConfigShPtr robotCurrentConfIthProblem(unsigned int rank) const;

      /// \brief Set current configuration of i-th robot.
      /// \param rank Id of robot in vector,
      /// \param config Set current configuration of robot in given problem,
      /// \return KD_OK or KD_ERROR.

      /// \xrefitem <send-notif> "Notification" "Send Notification"
      /// Send ID_HPP_SET_CURRENT_CONFIG.
      ktStatus robotCurrentConfIthProblem(unsigned int rank,
					  const CkwsConfigShPtr& config);

      /// \brief Set current configuration of i-th robot.
      /// \param rank Id of robot in vector,
      /// \param config the configuration
      /// \return KD_OK or KD_ERROR.

      /// \xrefitem <send-notif> "Notification" "Send Notification"
      /// Send ID_HPP_SET_CURRENT_CONFIG.
      ktStatus robotCurrentConfIthProblem(unsigned int rank,
					  const CkwsConfig& config);

      /// \brief Get initial configuration of i-th robot.
      /// \param rank : Id of problem in vector.
      /// \return initial configuration of robot.
      /// \return KD_OK or KD_ERROR
      CkwsConfigShPtr initConfIthProblem(unsigned int rank) const;

      /// \brief Set initial configuration of i-th robot.
      /// \param rank Id of robot in vector.
      /// \param config the configuration
      void initConfIthProblem(unsigned int rank,
			      const CkwsConfigShPtr& config);

      /// \brief Get vector of goal configurations of i-th robot.
      /// \param rank : Id of problem in vector.
      /// \return vector of goal configurations.
      const std::vector<CkwsConfigShPtr>&
      goalConfIthProblem(unsigned int rank) const;

      /// \brief Add a goal configuration of i-th robot.
      /// \param rank Id of robot in vector.
      /// \param config the configuration
      /// \return KD_OK or KD_ERROR
      void addGoalConfIthProblem(unsigned int rank,
				 const CkwsConfigShPtr& config);

      /// Reset the set of goal configurations
      void resetGoalConfIthProblem (unsigned int rank);

      /// \brief Set roadmap builder of i-th problem.
      /// \param rank Rank of problem in Planner::problemVector_.
      /// \param roadmapBuilder roadmap builder.
      /// \param display whether the roadmap should be displayed in gui.

      /// \xrefitem <send-notif> "Notification" "Send Notification"
      /// Send ID_HPP_ADD_ROADMAP.

      ///  \xrefitem <send-notif> "Notification" "Send Notification"
      ///  Send ID_HPP_REMOVE_ROADMAP.
      virtual ktStatus
      roadmapBuilderIthProblem(unsigned int rank,
			       CkwsRoadmapBuilderShPtr roadmapBuilder,
			       bool display=false);

      /// Clear roadmaps of each problem
      virtual void clearRoadmaps ();

      /// \brief Get roadmap builder of i-th problem.
      /// \param rank Rank of problem in Planner::problemVector_.
      /// \return shared pointer to roadmap builder.
      CkwsRoadmapBuilderShPtr roadmapBuilderIthProblem(unsigned int rank);

      /// \brief Set path optimizer of i-th problem.
      /// \param rank Rank of problem in Planner::problemVector_.
      /// \param pathOptimizer path optimizer.
      ktStatus pathOptimizerIthProblem(unsigned int rank,
				       CkwsPathPlannerShPtr pathOptimizer);

      /// \brief Get path optimizer of i-th problem.
      /// \param rank Rank of problem in Planner::problemVector_.
      /// \return shared pointer to path optimizer.
      CkwsPathPlannerShPtr pathOptimizerIthProblem(unsigned int rank);

      /// \brief Set steering Method of i-th problem.
      /// \param rank Rank of problem in Planner::problemVector_.
      /// \param sm steering Method.
      ktStatus steeringMethodIthProblem(unsigned int rank,
					CkppSteeringMethodComponentShPtr sm);

      /// \brief Get steering Method of i-th problem.
      /// \param rank Rank of problem in Planner::problemVector_.
      /// \return shared pointer to steering Method.
      CkppSteeringMethodComponentShPtr
      steeringMethodIthProblem(unsigned int rank);

      /// \brief Set configuration extractor to given problem
      /// \param rank Rank of problem in Planner::problemVector_.
      /// \param inConfigExtractor Configuration extractor
      /// A configuration extractor attempts at extracting a collision-free
      /// configuration in the neighborhood of the initial configuration of
      /// a path planning problem when the initial configuration is in
      /// collision.
      ktStatus configExtractorIthProblem
      (unsigned int rank, const CkwsConfigExtractorShPtr& inConfigExtractor);

      /// \brief Set dynamic penetration of given problem
      /// \param rank Rank of problem in Planner::problemVector_.
      /// \param penetration dynamic penetration for validating direct paths.
      ktStatus penetration(unsigned int rank, double penetration);

      /// \brief Get dynamic penetration of given problem
      /// \param rank Rank of problem in Planner::problemVector_.
      /// \return dynamic penetration allowed for validating a direct path.
      double penetration(unsigned int rank) const;

      /// \brief Initialize the list of obstacles.
      /// \param collisionList list of obstacles.

      /// \xrefitem <send-notif> "Notification" "Send Notification"
      /// Send ID_HPP_REMOVE_OBSTACLES.

      /// \xrefitem <send-notif> "Notification" "Send Notification"
      /// Send ID_HPP_SET_OBSTACLE_LIST.

      /// Replaces previous list of obstacle if any. Copy the list of
      /// obstacle in each problem of the object.
      virtual ktStatus obstacleList(std::vector< CkcdObjectShPtr > collisionList);
      /// \brief return a shared pointer to the obstacle list
      const std::vector< CkcdObjectShPtr > obstacleList();

      /// \brief Add obstacle to the list.
      /// \param object a new object.
      /// \param distanceComputation whether distance to this object should be computed in Body class.

      /// \xrefitem <send-notif> "Notification" "Send Notification" Send ID_HPP_ADD_OBSTACLE.

      /// Add the obstacle to each problem of the object.
      virtual ktStatus addObstacle(CkcdObjectShPtr object,
				   bool distanceComputation = true);

      /// @}
      /// \name Parser
      /// @{

      /// \brief Parsing a file
      ktStatus parseFile(const std::string& inFileName);

      /// \brief Load a path from file.
      ktStatus loadPathFromFile (const std::string& fileName);

      /// \brief Write path to file.
      ///
      /// \param rank rank of problem
      /// \param pathId id of path in problem
      /// \param pathName name to be given to the path component
      /// \param fileName file path where the component will be saved.
      ktStatus writePathToFile (unsigned int rank,
				unsigned int pathId,
				const std::string& pathName,
				const std::string& fileName);

      /// @}

      /// \name Initialization
      /// @{

      /// \brief Initialization of the path planning problem
      virtual ktStatus initializeProblem()
      {
	return KD_OK;
      }

      /// @}

      /// \name Problem resolution
      /// @{

      /// \brief Compute a solution path for each problem.
      /// \return KD_OK or KD_ERROR
      virtual ktStatus solve();

      /// \brief Solve a problem in the vector.
      /// \param rank the rank of the problem in vector.
      /// \return KD_OK or KD_ERROR
      /// See Problem::solve() for more information.
      /// If successful, the function stores the resulting path in the Problem.
      virtual ktStatus solveOneProblem(unsigned int rank);

      /// \brief Interrupt current roadmap builder.
      /// \note if no roadmap builder is running, this will interrupt the next one that starts.
      void interruptPathPlanning();

      /// \brief Optimize a given path
      /// \param problemId Id of the problem owning the path.
      /// \param pathId Id of the path in this problem.
      ktStatus optimizePath(unsigned int problemId, unsigned int pathId);

      /// \brief Get number of paths in given problem.
      unsigned int getNbPaths(unsigned int rank) const;

      /// \brief Get path of given rank in given problem.
      CkwsPathShPtr getPath(unsigned int rank, unsigned int pathId) const;

      /// \brief Add a path to a given problem.
      /// \param rank rank of problem in vector.
      /// \param kwsPath the path to add.
      /// \return KD_ERROR if rank too big, KD_OK otherwise.
      ktStatus addPath(unsigned int rank, CkwsPathShPtr kwsPath);

      /// @}

      /// \name Search functions
      /// @{

      /// \brief Find a body by the name of its joint
      ///\param inJointName name of the joint
      /// \return const shared pointer to the body

      /// Look among all joints of the robot of each problem for a given name.
      CkwsKCDBodyAdvancedConstShPtr
      findBodyByJointName(const std::string& inJointName) const;


      /// @}
    private:

      /// \brief pointer to a KineoWorks notificator.
      CkitNotificatorShPtr notificator_;

      /// \brief Vector (deque) of path planning problems.
      /// Some iterative path planning methods for humanoid robots
      /// solve iteratively different basic path planning
      /// problems. Each of these problems is defined by class Problem.
      /// These basic problems are put into a vector.
      std::deque<Problem> problemVector_;


      /// \brief Obstacles are a list of KCD fobjects.
      std::vector< CkcdObjectShPtr > obstacleVector_;

      /// \brief Roadmap builder delegate enabling to interrupt roadmap builder.
      CkwsPlusStopRdmBuilderDelegate* stopRdmBuilderDelegate_;

      /// \brief Parser specialization
      model::Parser* parser_;

      // for notification.
    public:
      static const CkitNotification::TType ID_HPP_ADD_ROBOT;
      static const CkitNotification::TType ID_HPP_SET_CURRENT_CONFIG;
      static const CkitNotification::TType ID_HPP_REMOVE_OBSTACLES;
      static const CkitNotification::TType ID_HPP_SET_OBSTACLE_LIST;
      static const CkitNotification::TType ID_HPP_ADD_OBSTACLE;
      static const CkitNotification::TType ID_HPP_REMOVE_ROADMAPBUILDER;
      static const CkitNotification::TType ID_HPP_ADD_ROADMAPBUILDER;

      // key to retrieve for notification.
      static const std::string ROBOT_KEY;
      static const std::string OBSTACLE_KEY;
      static const std::string CONFIG_KEY;
      static const std::string ROADMAP_KEY;

    }; // class Planner
  } // namespace core
} // namespace namespace hpp

typedef hpp::core::Planner ChppPlanner HPP_CORE_DEPRECATED;
#endif
