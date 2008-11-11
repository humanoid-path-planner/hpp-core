/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS) and Eiichi Yoshida (JRL-FRANCE/LAAS-CNRS/AIST)

*/


#ifndef HPP_PROBLEM_H
#define HPP_PROBLEM_H

/*************************************
INCLUDE
**************************************/

#include "KineoWorks2/kwsPathOptimizer.h"
#include "KineoWorks2/kwsRoadmapBuilder.h"
#include "KineoWorks2/kwsConfig.h"

#include "KineoModel/kppDeviceComponent.h"
#include "kcd2/kcdInterface.h"
#include "kwsKcd2/kwsKCDBody.h"

#include "KineoUtility/kitNotificator.h"

KIT_PREDEF_CLASS(CkwsConfigExtractor);

/*************************************
CLASS
**************************************/
/**
    \brief Defines a path planning problem for one robot.
    A path planning problem is defined by
    \li a robot: a KineoWorks device,
    \li a set of obstacles: a list of Kcd objects,
    \li a steering method: stored in the KwsDevice,
    \li initial and goal configurations,
    \li a roadmap : to store a roadmap
    \li a roadmapBuilder : roadmap strategy
    \li a pathOptimizer : to Optimise the path
*/
class ChppProblem
{
	public:
		/**
		 * \brief Create a path planning problem with no initial nor goal configuration.
		 *
		 * \param inRobot robot associated to the path planning problem.
		 * \param inPenetration dynamic penetration allowed for validating direct paths.
		 */
		ChppProblem (CkppDeviceComponentShPtr inRobot, double inPenetration);

		/**
		 * \brief Create a path planning problem with no initial nor goal configuration.
		 * \param inRobot robot associated to the path planning problem.
		 * \param inObstacleList list of obstacle of this problem.
		 * \param inPenetration dynamic penetration allowed for validating direct paths.
		 */
		ChppProblem (CkppDeviceComponentShPtr inRobot,
			     const std::vector<CkcdObjectShPtr>& inObstacleList,
			     double inPenetration);

		/**
		   \brief Copy constructor
		*/
		ChppProblem(const ChppProblem& inProblem);

		/**
		   \name Problem definition
		   @{
		 */

		/**
		 * \brief return shared pointer to robot.
		 */
		CkppDeviceComponentShPtr getRobot() const;
		/**
		 * \brief Get shared pointer to initial configuration.
		 */
		CkwsConfigShPtr initConfig() const;
		/**
		 * \brief Set initial configuration.
		 */
		ktStatus initConfig ( CkwsConfigShPtr i_config );
		/**
		 * \brief Get shared pointer to goal configuration.
		 */
		CkwsConfigShPtr goalConfig() const;
		/**
		 * \brief Set goal configuration.
		 */
		ktStatus goalConfig ( CkwsConfigShPtr i_config );
		/**
		   \brief Set device steering method
		*/
		/**
		 *@}
		 */

		/**
		   \name Obstacles
		   @{
		*/
		/**
		 * \brief Store a copy of the list of obstacles.
		 * \param inCollisionList list of obstacles to be taken into account for this problem.
		 *
		 * Replaces previous list of obstacle if any. 
		 * \note Compute collision entities.
		 */
		ktStatus obstacleList ( const std::vector<CkcdObjectShPtr>& inCollisionList );
		/**
		 * \brief return a shared pointer to the obstacle list
		 */
		const std::vector<CkcdObjectShPtr>& obstacleList();

		/**
		 * \brief Add obstacle to the list.
		 * \param inObject a new object.
		 * \note Compute collision entities.
		 */
		ktStatus addObstacle ( const CkcdObjectShPtr& inObject );

		/**
		 *@}
		 */

		/**
		   \name Problem resolution
		   @{
		*/

		/**
		  \brief set device steering method
		*/
		void steeringMethod ( const CkwsSteeringMethodShPtr &inSteeringMethod );
		/**
		   \brief Get device steering method
		*/
		CkwsSteeringMethodShPtr steeringMethod() const;
		/**
		   \brief Set roadmap building strategy.
		*/
		void roadmapBuilder ( CkwsRoadmapBuilderShPtr inroadmapBuilder );
		/**
		   \brief Get roadmap building strategy.
		*/
		CkwsRoadmapBuilderShPtr roadmapBuilder() ;
		/**
		    \brief set roadmap.
		    \param inRoadmap shared pointer to a roadmap.
		 */
		void roadmap ( CkwsRoadmapShPtr inRoadmap ) ;
		/**
		    \brief Get roadmap.
		    \return shared pointer to the roadmap.
		*/
		CkwsRoadmapShPtr roadmap() const;
		/**
		   \brief Set pathOptimizer.
		   \param inPathOptimizer path optimizer.
		*/
		void pathOptimizer ( CkwsPathOptimizerShPtr inPathOptimizer );
		/**
		   \brief Get path optimizer
		   \return shared pointer to the path optimiser
		*/
		CkwsPathOptimizerShPtr pathOptimizer() ;

		/**
		   \brief Set configuration extractor
		   \param inConfigExtractor
		*/
		void configExtractor(const CkwsConfigExtractorShPtr& inConfigExtractor);

		/**
		   \brief Get configuration extractor
		*/
		const CkwsConfigExtractorShPtr& configExtractor();
		   

		/**
		   \brief Set dynamic penetration of given problem
		   
		   \param inPenetration dynamic penetration allowed for validating a direct path.
		*/
		void penetration(double inPenetration);
		
		/**
		   \brief Get dynamic penetration of given problem
		   
		   \return dynamic penetration allowed for validating a direct path.
		*/
		double penetration() const;

		/**
		  \brief Add a path to the vector.
		   \xrefitem <send-notif> "Notification" "Send Notification" Send ID_HPP_ADD_PATH.
		*/
		void addPath ( CkwsPathShPtr kwsPath );
		/**
		    \brief Get I-th path in vector
		*/
		CkwsPathShPtr getIthPath ( unsigned int pathId ) const;
		/**
		   \brief et number of paths in vector
		*/
		unsigned int getNbPaths() const;



		/**
		   \brief et initialize the map of innerBody (attMapOuter)
		*/
		void initMapOuter();
		/**
				 *@}
				 */


	private :
		/**
		   \brief pointer to a KineoWorks notificator.
		*/
		CkitNotificatorShPtr attNotificator;
		/**
		    \brief the robot is a KineoWorks Device.
		 */
		CkppDeviceComponentShPtr attRobot;
		/**
		   \brief Shared pointer to initial configuration.
		*/
		CkwsConfigShPtr attInitConf;
		/**
		   \brief Shared pointer to goal configuration.
		*/
		CkwsConfigShPtr attGoalConf;
		/**
		    \brief Shared pointer to a roadmap associate to the robot
		 */
		CkwsRoadmapShPtr attRoadmap ;
		/**
		    \brief Shared pointer to a roadmapBuilder associate to the the robot and the roadMap
		 */
		CkwsRoadmapBuilderShPtr attRoadmapBuilder;
		/**
		  \brief Shared pointer to a optimizer for the path
		*/
		CkwsPathOptimizerShPtr attPathOptimizer ;

		/**
		   \brief Configuration extractor 

		    A configuration extractor attempts at extracting a
		    collision-free configuration in the neighborhood
		    of the initial configuration of a path planning
		    problem when the initial configuration is in
		    collision.
		*/
		CkwsConfigExtractorShPtr attConfigExtractor;

		/**
		    \brief Get the list of obstacle of this problem.

		    The set of obstacles is likely to be a copy or reference to 
		    the list of obstacles of ChppPlanner. 
		    However, to allow more general motion planning strategies, we
		    leave to possiblity to define it differently. Obstacles are a
		    list of KCD objects.
		 */
		std::vector< CkcdObjectShPtr > attObstacleList;
		/**
		   \brief A vector of paths corresponding to this problem.
		*/
		std::vector<CkwsPathShPtr> attPathVector;


		/**
		   \brief A map of each body to the a vector of its outer bodies.
		*/
		std::map<CkwsKCDBodyShPtr,std::vector<CkcdObjectShPtr> > attMapOuter;

		/**
		   \brief Penetration
		*/
		double attPenetration;

	public:
		// for notification:
		static const CkitNotification::TType  ID_HPP_ADD_PATH;

		// key to retrieve
		static const std::string   PATH_KEY;
		static const std::string   PATH_ID_KEY;
		static const std::string   DEVICE_KEY;

};


#endif
