/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)

*/

#ifndef HPP_PLANNER_H
#define HPP_PLANNER_H

/*************************************
INCLUDE
**************************************/
#include <deque>

#include "KineoWorks2/kwsPathOptimizer.h"
#include "KineoWorks2/kwsRoadmapBuilder.h"
#include "KineoUtility/kitNotificator.h"

#include "hppCore/hppProblem.h"

#ifndef WITHOUT_CHPPDEVICE
KIT_PREDEF_CLASS( ChppBody );
#endif


using namespace std ;

/*************************************
CLASS
**************************************/
/**
   \brief Motion planning strategies for humanoid robots among obstacles.

   This abstract class defines motion planning strategies for a humanoid robot in an environment with obstacles. 
   Different strategies can be implemented by deriving this class.

   Several strategies instantiate several robots and classical path planning problems. Therefore, this class contains a vector of ChppProblem describing basic motion planning problems. 
*/
class ChppPlanner {


 public: 
  /**
     \brief Allocate a KineoWorks CkitNotificator object.
  */
  ChppPlanner();
  
  virtual ~ChppPlanner();

  /**
     \name Problem definition
     @{
   */

  /**
     \brief Add a Problem to the Problem vector with the associed robot.
 
     \param robot : a shared pointer to a robot
     \return a int
     \xrefitem <send-notif> "Notification" "Send Notification" Send ID_HPP_ADD_ROBOT.
  */
  ktStatus addHppProblem(CkppDeviceComponentShPtr robot);

  /**
     \brief Remove a Problem at the end of the Problem vector.
 
     \return a int
  */
  ktStatus removeHppProblem();

  /**
     \brief Add a Problem at beginning of the Problem vector with the associed robot.
 
     \param robot : a shared pointer to a robot
     \return a int
     \xrefitem <send-notif> "Notification" "Send Notification" Send ID_HPP_ADD_ROBOT.
  */
  ktStatus addHppProblemAtBeginning(CkppDeviceComponentShPtr robot);

  /**
     \brief Remove a Problem at the beginning the Problem vector.
 
     \return a int
  */
  ktStatus removeHppProblemAtBeginning();

  /**
     \brief Retrieve the problem in the problem vector at the given rank.
     \param i_rank Rank of the hppProblem to retrieve
     \return The i_rank-th problem
   */
  ChppProblem * hppProblem(unsigned int inRank) { 
    if(inRank<getNbHppProblems()) 
      return &(hppProblemVector.at(inRank)); 
    else 
      return NULL;
  };

  /**
     \brief Get the number of problems in vector.
     \return the number of problems in the vector
  */
  unsigned int getNbHppProblems() const { return (unsigned int)hppProblemVector.size();};


  /**
     \brief Get robot at given rank in the Problem vector.
     \param rank 
     \return the shared pointer on the robot
  */
  const CkppDeviceComponentShPtr robotIthProblem(unsigned int rank) const;

  /**
     \brief Get current configuration of i-th robot.
     \param rank : Id of problem in vector.
     \return : return a copy of the current configuration of robot.
  */
  CkwsConfigShPtr robotCurrentConfIthProblem(unsigned int rank) const;

  /**
     \brief Set current configuration of i-th robot.
     \param rank Id of robot in vector.
     \param config A new config is allocated and placed into current config of robot.
     \return KD_OK or KD_ERROR
     \xrefitem <send-notif> "Notification" "Send Notification" Send ID_HPP_SET_CURRENT_CONFIG.
  */
  ktStatus robotCurrentConfIthProblem(unsigned int rank,
				      const CkwsConfigShPtr& config);
  ktStatus robotCurrentConfIthProblem(unsigned int rank,
				      const CkwsConfig& config);
  
  /**
     \brief Get initial configuration of i-th robot.
     \param rank : Id of problem in vector.
     \return initial configuration of robot.
     \return KD_OK or KD_ERROR
  */
   CkwsConfigShPtr initConfIthProblem(unsigned int rank) const;

  /**
     \brief Set initial configuration of i-th robot.
     \param rank Id of robot in vector.
     \param config A new config is allocated and placed into initial config of robot.
     \return KD_OK or KD_ERROR
  */
  ktStatus initConfIthProblem(unsigned int rank, 
			      const CkwsConfigShPtr config);

  /**
     \brief Get goal configuration of i-th robot.
     \param rank : Id of problem in vector.
     \return the goal configuration of robot.
     \return KD_OK or KD_ERROR
  */
  CkwsConfigShPtr goalConfIthProblem(unsigned int rank) const;
  
  /**
     \brief Set goal configuration of i-th robot.
     \param rank Id of robot in vector.
     \param config A new config is allocated and placed into goal config of robot.
     \return KD_OK or KD_ERROR
  */
  ktStatus goalConfIthProblem(unsigned int rank,
			      const CkwsConfigShPtr config);


  /** 
    \brief Set roadmap builder of i-th problem.
    \param rank Rank of problem in ChppPlanner::hppProblemVector.
    \param inRoadmapBuilder roadmap builder.
  */
  ktStatus roadmapBuilderIthProblem(unsigned int rank, CkwsRoadmapBuilderShPtr inRoadmapBuilder);

  /** 
    \brief Get roadmap builder of i-th problem.
    \param rank Rank of problem in ChppPlanner::hppProblemVector.
    \return shared pointer to roadmap builder.
  */
  CkwsRoadmapBuilderShPtr roadmapBuilderIthProblem(unsigned int rank);

  /** 
    \brief Set path optimizer of i-th problem.
    \param rank Rank of problem in ChppPlanner::hppProblemVector.
    \param inPathOptimizer path optimizer.
  */
  ktStatus pathOptimizerIthProblem(unsigned int rank, CkwsPathOptimizerShPtr inPathOptimizer);

  /** 
    \brief Get path optimizer of i-th problem.
    \param rank Rank of problem in ChppPlanner::hppProblemVector.
    \return shared pointer to path optimizer.
  */
  CkwsPathOptimizerShPtr pathOptimizerIthProblem(unsigned int rank);
   /** 
    \brief Set steering Method of i-th problem.
    \param rank Rank of problem in ChppPlanner::hppProblemVector.
    \param inSM steering Method.
  */
  ktStatus steeringMethodIthProblem(unsigned int rank, CkwsSteeringMethodShPtr inSM);

  /** 
    \brief Get steering Method of i-th problem.
    \param rank Rank of problem in ChppPlanner::hppProblemVector.
    \return shared pointer to steering Method.
  */
  CkwsSteeringMethodShPtr steeringMethodIthProblem(unsigned int rank);
 
  /**
   * \brief Initialize the list of obstacles.
   * \param collisionList list of obstacles.
   * \xrefitem <send-notif> "Notification" "Send Notification" Send ID_HPP_REMOVE_OBSTACLES.
   * \xrefitem <send-notif> "Notification" "Send Notification" Send ID_HPP_SET_OBSTACLE_LIST.
   *
   * Replaces previous list of obstacle if any. Copy the list of obstacle in each problem of the object.
   */
  virtual ktStatus obstacleList(std::vector< CkcdObjectShPtr > collisionList);
  /**
   * \brief return a shared pointer to the obstacle list
   */
  const std::vector< CkcdObjectShPtr > obstacleList();
  /**
   * \brief Add obstacle to the list.
   * \param object a new object.
   * \xrefitem <send-notif> "Notification" "Send Notification" Send ID_HPP_ADD_OBSTACLE.
   *
   * Add the obstacle to each problem of the object.
   */
  virtual ktStatus addObstacle(CkcdObjectShPtr object);

  /** 
   *@}
   */

  /**
     \name Initialization
     @{
  */

  /**
     \brief Initialization of the path planning problem
  */
  virtual ktStatus initializeProblem() {return KD_OK;};

  /** 
   *@}
   */

  /**
     \name Problem resolution
     @{
  */
    /**
       \brief Compute a solution path for each problem.
       \return KD_OK or KD_ERROR
    */
    virtual ktStatus solve();

    /**
       \brief Solve a problem in the vector.
       \param problemId the rank of the problem in vector.
       \return KD_OK or KD_ERROR
       
       If successful, the function stores the resulting path in the hppProblem (hppProblemVector[problemId].addPath()).
    */
    ktStatus solveOneProblem(unsigned int problemId);

    /**
       \brief Optimize a given path
       \param inProblemId Id of the problem owning the path.
       \param inPathId Id of the path in this problem.
    */
    ktStatus optimizePath(unsigned int inProblemId, unsigned int inPathId);

    /** 
      \brief Get number of paths in given problem.
    */
    unsigned int getNbPaths(unsigned int problemId) const;
    /** 
      \brief Get path of given rank in given problem.
    */
    CkwsPathShPtr getPath(unsigned int problemId, unsigned int pathId) const;
    /** 
      \brief Add a path to a given problem.
      \param problemId rank of problem in vector.
      \param kwsPath the path to add.
      \return KD_ERROR if problemId too big, KD_OK otherwise.
      
    */
  ktStatus addPath(unsigned int problemId, CkwsPathShPtr kwsPath);

  /** 
   *@}* 
   */

  /**
     \name Search functions
     @{
  */

  /** 
   * \brief Find a body by name.
   * \param inBodyName name of the body
   * \return const shared pointer to the body
   *
      Look among all the bodies of the robot of each problem for a given name.

   */
  ChppBodyConstShPtr findBodyByName(std::string inBodyName) const;

  /** 
   *@}* 
   */
  /**
     \name Notification management
     @{
  */
  /**
     \brief Subscribe to an event by calling kwsNotificator->subscribe;
  */
  // CkitNotificator::eventRegisterPair subscribe(int inEventType, ktStatus(*inCallback)(void*,void*),void* inPt);

  /**
     \brief Unsubscribe to an event by calling kwsNotificator->unsubscribe;
  */
  // ktStatus unsubscribe(const CkitNotificator::eventRegisterPair& inPair);
  /**
   * \brief return a pointer to notificator in order for interface to get notification information.
   */
  // const ChppNotificator* notificator() const;
  /**
    @}
  */

protected:

  /**
     \brief pointer to a KineoWorks notificator.
  */
  CkitNotificatorShPtr attNotificator;
  /**
     \brief Vector (deque) of path planning problems. 
     Some iterative path planning methods for humanoid robots 
     solve iteratively different basic path planning problems. Each of these problems is defined by class ChppProblem. 
     These basic problems are put into a vector.
  */
  deque<ChppProblem> hppProblemVector;
  
  
  /** 
      \brief Obstacles are a list of KCD fobjects. 
  */
  std::vector< CkcdObjectShPtr > mObstacleList;

  // for nortification.
 public:
  static const CkitNotification::TType ID_HPP_ADD_ROBOT;
  static const CkitNotification::TType ID_HPP_SET_CURRENT_CONFIG;
  static const CkitNotification::TType ID_HPP_REMOVE_OBSTACLES;
  static const CkitNotification::TType ID_HPP_SET_OBSTACLE_LIST;
  static const CkitNotification::TType ID_HPP_ADD_OBSTACLE;

  // key to retrieve for notification.
  static const std::string ROBOT_KEY;
  static const std::string OBSTACLE_KEY;
  static const std::string CONFIG_KEY;
   
};



#endif


