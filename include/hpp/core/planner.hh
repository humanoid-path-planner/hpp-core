/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)

*/

#ifndef HPP_CORE_PLANNER_HH
#define HPP_CORE_PLANNER_HH

/*************************************
INCLUDE
**************************************/
#include <deque>

#include "KineoWorks2/kwsPathOptimizer.h"
#include "KineoWorks2/kwsRoadmapBuilder.h"
#include "KineoUtility/kitNotificator.h"

#include "hpp/core/problem.hh"

class CkwsPlusStopRdmBuilderDelegate;

KIT_PREDEF_CLASS(CkppKCDBody);

#ifndef WITHOUT_CHPPDEVICE
KIT_PREDEF_CLASS( ChppBody );
#endif

KIT_PREDEF_CLASS(CkwsConfigExtractor);

KIT_PREDEF_CLASS(ChppPlanner);

namespace hpp {
  namespace core {
    class Parser;
  }
}

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
     \brief Empty constructor

     Allocate a KineoWorks CkitNotificator object.
  */
  ChppPlanner();

  /**
     \brief Copy constructor
  */
  ChppPlanner(const ChppPlanner& inPlanner);
  
  /**
     \brief Destructor
  */
  virtual ~ChppPlanner();

  /**
     \name Problem definition
     @{
   */

  /**
     \brief Add a Problem to the Problem vector with the associed robot.
 
     \param inRobot : a shared pointer to a robot
     \param inPenetration dynamic penetration allowed for validating direct paths.

     \return KD_OK if success, KD_ERROR otherwise

     \xrefitem <send-notif> "Notification" "Send Notification" Send ID_HPP_ADD_ROBOT.
  */
  ktStatus addHppProblem(CkppDeviceComponentShPtr inRobot, double inPenetration);

  /**
     \brief Remove a Problem at the end of the Problem vector.
 
     \return a int
  */
  ktStatus removeHppProblem();

  /**
     \brief Add a Problem at beginning of the Problem vector with the associed robot.
 
     \param inRobot : a shared pointer to a robot
     \param inPenetration dynamic penetration allowed for validating direct paths.
     \return KD_OK if success, KD_ERROR otherwise

     \xrefitem <send-notif> "Notification" "Send Notification" Send ID_HPP_ADD_ROBOT.
  */
  ktStatus addHppProblemAtBeginning(CkppDeviceComponentShPtr inRobot, double inPenetration);

  /**
     \brief Remove a Problem at the beginning the Problem vector.
 
     \return a int
  */
  ktStatus removeHppProblemAtBeginning();

  /**
     \brief Retrieve the problem in the problem vector at the given rank.
     \param inRank Rank of the hppProblem to retrieve
     \return The inRank-th problem
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
     \param inRank 
     \return the shared pointer on the robot
  */
  const CkppDeviceComponentShPtr robotIthProblem(unsigned int inRank) const;

  /**
     \brief Get current configuration of i-th robot.
     \param inRank : Id of problem in vector.
     \return : return a copy of the current configuration of robot.
  */
  CkwsConfigShPtr robotCurrentConfIthProblem(unsigned int inRank) const;

  /**
     \brief Set current configuration of i-th robot.
     \param inRank Id of robot in vector.
     \param config A new config is allocated and placed into current config of robot.
     \return KD_OK or KD_ERROR
     \xrefitem <send-notif> "Notification" "Send Notification" Send ID_HPP_SET_CURRENT_CONFIG.
  */
  ktStatus robotCurrentConfIthProblem(unsigned int inRank,
				      const CkwsConfigShPtr& config);
  ktStatus robotCurrentConfIthProblem(unsigned int inRank,
				      const CkwsConfig& config);
  
  /**
     \brief Get initial configuration of i-th robot.
     \param inRank : Id of problem in vector.
     \return initial configuration of robot.
     \return KD_OK or KD_ERROR
  */
   CkwsConfigShPtr initConfIthProblem(unsigned int inRank) const;

  /**
     \brief Set initial configuration of i-th robot.
     \param inRank Id of robot in vector.
     \param config A new config is allocated and placed into initial config of robot.
     \return KD_OK or KD_ERROR
  */
  ktStatus initConfIthProblem(unsigned int inRank, 
			      const CkwsConfigShPtr config);

  /**
     \brief Get goal configuration of i-th robot.
     \param inRank : Id of problem in vector.
     \return the goal configuration of robot.
     \return KD_OK or KD_ERROR
  */
  CkwsConfigShPtr goalConfIthProblem(unsigned int inRank) const;
  
  /**
     \brief Set goal configuration of i-th robot.
     \param inRank Id of robot in vector.
     \param config A new config is allocated and placed into goal config of robot.
     \return KD_OK or KD_ERROR
  */
  ktStatus goalConfIthProblem(unsigned int inRank,
			      const CkwsConfigShPtr config);


  /** 
    \brief Set roadmap builder of i-th problem.
    \param inRank Rank of problem in ChppPlanner::hppProblemVector.
    \param inRoadmapBuilder roadmap builder.
    \param inDisplay whether the roadmap should be displayed in the interface.
     \xrefitem <send-notif> "Notification" "Send Notification" Send ID_HPP_ADD_ROADMAP.
     \xrefitem <send-notif> "Notification" "Send Notification" Send ID_HPP_REMOVE_ROADMAP.
  */
  virtual ktStatus roadmapBuilderIthProblem(unsigned int inRank, CkwsRoadmapBuilderShPtr inRoadmapBuilder,
					    bool inDisplay=false);

  /** 
    \brief Get roadmap builder of i-th problem.
    \param inRank Rank of problem in ChppPlanner::hppProblemVector.
    \return shared pointer to roadmap builder.
  */
  CkwsRoadmapBuilderShPtr roadmapBuilderIthProblem(unsigned int inRank);

  /** 
    \brief Set path optimizer of i-th problem.
    \param inRank Rank of problem in ChppPlanner::hppProblemVector.
    \param inPathOptimizer path optimizer.
  */
  ktStatus pathOptimizerIthProblem(unsigned int inRank, CkwsPathOptimizerShPtr inPathOptimizer);

  /** 
    \brief Get path optimizer of i-th problem.
    \param inRank Rank of problem in ChppPlanner::hppProblemVector.
    \return shared pointer to path optimizer.
  */
  CkwsPathOptimizerShPtr pathOptimizerIthProblem(unsigned int inRank);
   /** 
    \brief Set steering Method of i-th problem.
    \param inRank Rank of problem in ChppPlanner::hppProblemVector.
    \param inSM steering Method.
  */
  ktStatus steeringMethodIthProblem(unsigned int inRank, CkwsSteeringMethodShPtr inSM);

  /** 
    \brief Get steering Method of i-th problem.
    \param inRank Rank of problem in ChppPlanner::hppProblemVector.
    \return shared pointer to steering Method.
  */
  CkwsSteeringMethodShPtr steeringMethodIthProblem(unsigned int inRank);
 
  /**
     \brief Set configuration extractor to given problem

    \param inRank Rank of problem in ChppPlanner::hppProblemVector.
    \param inConfigExtractor Configuration extractor

     A configuration extractor attempts at extracting a collision-free
     configuration in the neighborhood of the initial configuration of
     a path planning problem when the initial configuration is in
     collision.
  */
  ktStatus configExtractorIthProblem(unsigned int inRank, 
				     const CkwsConfigExtractorShPtr& inConfigExtractor);

  /**
     \brief Set dynamic penetration of given problem

     \param inRank Rank of problem in ChppPlanner::hppProblemVector.
     \param inPenetration dynamic penetration allowed for validating a direct path.
  */
  ktStatus penetration(unsigned int inRank, double inPenetration);

  /**
     \brief Get dynamic penetration of given problem

     \param inRank Rank of problem in ChppPlanner::hppProblemVector.
     \return dynamic penetration allowed for validating a direct path.
  */
  double penetration(unsigned int inRank) const;

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
     \brief Add obstacle to the list.
     \param object a new object.
     \param inDistanceComputation whether distance to this object should be computed in ChppBody class.

     \xrefitem <send-notif> "Notification" "Send Notification" Send ID_HPP_ADD_OBSTACLE.
     
     Add the obstacle to each problem of the object.
   */
  virtual ktStatus addObstacle(CkcdObjectShPtr object, 
			       bool inDistanceComputation = true);

  /** 
   *@}
   */

  /**
     \name Parser
     @{
  */
  
  /**
     \brief Parsing a file
  */
  
  ktStatus parseFile(const std::string& inFileName);
  
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

       \param inRank the rank of the problem in vector.
       \return KD_OK or KD_ERROR
       
       See ChppProblem::solve() for more information.

       If successful, the function stores the resulting path in the hppProblem.
    */
    virtual ktStatus solveOneProblem(unsigned int inRank);

  /**
     \brief Interrupt current roadmap builder.

     \note if no roadmap builder is running, this will interrupt the next one that starts.
  */
  void interruptPathPlanning();

    /**
       \brief Optimize a given path
       \param inProblemId Id of the problem owning the path.
       \param inPathId Id of the path in this problem.
    */
    ktStatus optimizePath(unsigned int inProblemId, unsigned int inPathId);

    /** 
      \brief Get number of paths in given problem.
    */
    unsigned int getNbPaths(unsigned int inRank) const;
    /** 
      \brief Get path of given rank in given problem.
    */
    CkwsPathShPtr getPath(unsigned int inRank, unsigned int pathId) const;
    /** 
      \brief Add a path to a given problem.
      \param inRank rank of problem in vector.
      \param kwsPath the path to add.
      \return KD_ERROR if inRank too big, KD_OK otherwise.
      
    */
  ktStatus addPath(unsigned int inRank, CkwsPathShPtr kwsPath);

  /** 
   *@}
   */

  /**
     \name Search functions
     @{
  */

  /** 
   * \brief Find a body by the name of its joint

   * \param inJointName name of the joint
   * \return const shared pointer to the body
   *
      Look among all the joitns of the robot of each problem for a given name.

   */
  CkwsKCDBodyConstShPtr findBodyByJointName(const std::string& inJointName) const;


  /** 
   *@} 
   */

private:

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
  std::deque<ChppProblem> hppProblemVector;
  
  
  /** 
      \brief Obstacles are a list of KCD fobjects. 
  */
  std::vector< CkcdObjectShPtr > attObstacleList;

  /**
     \brief Roadmap builder delegate enabling to interrupt roadmap builder.
  */
  CkwsPlusStopRdmBuilderDelegate* attStopRdmBuilderDelegate;

  /**
     \brief Parser specialization
  */
  hpp::core::Parser* parser_;

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
   
};



#endif


