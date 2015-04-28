//
// Copyright (c) 2015 CNRS
// Author: Mylene Campana
//
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

#ifndef HPP_CORE_PATH_OPTIMIZATION_COLLISION_CONSTRAINTS_RESULT_HH
# define HPP_CORE_PATH_OPTIMIZATION_COLLISION_CONSTRAINTS_RESULT_HH

# include <hpp/fcl/distance.h>

namespace hpp {
  namespace core {
    using model::displayConfig;
    namespace pathOptimization {
      HPP_PREDEF_CLASS (CollisionConstraintsResult);

      using constraints::Transformation;
      using constraints::RelativeTransformation;
      using constraints::TransformationPtr_t;
      using constraints::RelativeTransformationPtr_t;
      /// Storing data about one collision, to facilitate building
      /// collision-constraints in Gradient-Based path-optimizer

      struct HPP_CORE_DLLAPI CollisionConstraintsResult
      {
	static size_type fSize_;
	CollisionConstraintsResult
	  (const DevicePtr_t& robot, const PathVectorPtr_t& previousPath,
	   const PathVectorPtr_t& currentPath,
	   const std::pair <CollisionPathValidationReportPtr_t,
	   std::size_t>& report,
	   size_type rowInJacobian, size_type nbNonLockedDofs) :
	  robot_ (robot), robotNumberDofs_ (robot_->numberDof ()),
	  nbNonLockedDofs_ (nbNonLockedDofs), rowInJacobian_ (rowInJacobian),
	  nbWaypoints_ ((size_type) previousPath->numberPaths () - 1)
	  {
	    Jcompressed_.resize (fSize_, nbNonLockedDofs_);
	    Jcompressed_.setZero ();

	    value_type t_local = report.first->parameter;
	    localPathId_ = report.second;
	    hppDout (info, "localPathId_ = " << localPathId_);
	    PathPtr_t localPath = currentPath->pathAtRank(localPathId_);
	    Configuration_t qColl = (*localPath) (t_local);
	    posAlongLocalPath_ = t_local/localPath->length ();
	    HPP_STATIC_CAST_REF_CHECK (CollisionValidationReport,
				       *(report.first->configurationReport));
	    object1_ =
	      HPP_STATIC_PTR_CAST (CollisionValidationReport,
				   report.first->configurationReport)->object1;
	    object2_ =
	      HPP_STATIC_PTR_CAST (CollisionValidationReport,
				   report.first->configurationReport)->object2;
	    hppDout (info, "posAlongLocalPath_ (in [0,1]) = "
		     << posAlongLocalPath_);
	    hppDout (info, "obj1 = " << object1_->name()
		     << " and obj2 = " << object2_->name());
	    hppDout (info, "qColl = " << displayConfig (qColl));

	    // Backtrack collision in previous path (x0) to create constraint
	    PathPtr_t prevLocalPath = previousPath->pathAtRank (localPathId_);
	    value_type t_local_new =
	      prevLocalPath->length () * posAlongLocalPath_;
	    constraintConfig_ = (*prevLocalPath) (t_local_new);
	    hppDout (info, "constraintConfig_ = "
		     << displayConfig (constraintConfig_));

	    computeConstraint ();
	  }

	/// Get id of row in Jacobian
	size_type rowInJacobian () const { return rowInJacobian_;}

	/// Get the configuration defining the constraint reference
	///
	/// When a collision is detected along a path, the configuration
	/// along the latest collision-free path with the same parameter is
	/// used as reference for a (relative) transformation constraint.
	const Configuration_t& constraintConfiguration () const
	{
	  return constraintConfig_;
	}

	/// Check that constraint is satisfied for given path
	///
	/// \param path iterate along optimization process
	///
	/// Compare distance to collision with maximal displacement of
	/// relative transformation
	bool satisfied (const PathVectorPtr_t& path)
	{
	  PathPtr_t localPath (path->pathAtRank (localPathId_));
	  value_type t_local = localPath->length () * posAlongLocalPath_;
	  Configuration_t q = (*localPath) (t_local);
	  vector_t fx; fx.resize (f_->outputSize ()); (*f_) (fx, q);
	  vector_t rotation (fx.bottomRows (3));
	  vector_t translation (fx.topRows (3));
	  value_type range = radius_*(rotation.norm ()) + translation.norm ();
	  if  (range <= distance_) return true;
	  return false;
	}

	bool linearize (const PathVectorPtr_t& path, matrixOut_t jacobian,
			vectorOut_t value) const
	{
	  PathPtr_t localPath (path->pathAtRank (localPathId_));
	  value_type t_local = localPath->length () * posAlongLocalPath_;
	  Configuration_t q = (*localPath) (t_local);
	  size_type rank = localPathId_ - 1;
	  value_type position = 0; // 0=not extremity, 1=begin, 2=end
	  if (rank + 1 == 0)
	    position = 1;
	  if (rank + 1 == (size_type) nbWaypoints_)
	    position = 2;

	  configProjector_->computeValueAndJacobian
	    (q, value.segment (rowInJacobian_, fSize_), Jcompressed_);
	  // Complete 2 blocks in J_ except for begin and end segments
	  if (position != 1) {
	    jacobian.block (rowInJacobian_, rank*nbNonLockedDofs_,
			    fSize_, nbNonLockedDofs_) =
	      (1-posAlongLocalPath_)*Jcompressed_;
	  }
	  if (position != 2) {
	    jacobian.block (rowInJacobian_, (rank+1)*nbNonLockedDofs_,
			    fSize_, nbNonLockedDofs_) =
	      posAlongLocalPath_ * Jcompressed_;
	  }
	  return true;
	}

	/// Update reference of transformation constraint
	void updateReference (const PathVectorPtr_t& path, vectorOut_t)
	{
	  PathPtr_t localPath (path->pathAtRank (localPathId_));
	  value_type t_local = localPath->length () * posAlongLocalPath_;
	  constraintConfig_ = (*localPath) (t_local);

	  robot_->currentConfiguration (constraintConfig_);
	  robot_->computeForwardKinematics ();
	  model::Transform3f Mref (computeReference ());
	  if (object2_->joint ()){ // object2_ = body part
	    HPP_STATIC_CAST_REF_CHECK (RelativeTransformation, *f_);
	    RelativeTransformationPtr_t f
	      (HPP_DYNAMIC_PTR_CAST (RelativeTransformation, f_));
	    f->reference (Mref);
	  }
	  else { // object2_ = fixed obstacle and has no joint
	    HPP_STATIC_CAST_REF_CHECK (Transformation, *f_);
	    TransformationPtr_t f (HPP_DYNAMIC_PTR_CAST (Transformation, f_));
	    f->reference (Mref);
	  }
	  hppDout (info, "previous distance : " << distance_);
	  computeDistance ();
	  hppDout (info, "new distance : " << distance_);
	}

      private:
	/// Compute reference of transformation constraint
	model::Transform3f computeReference ()
	{
	  JointPtr_t joint1 = object1_->joint ();
	  model::Transform3f M1 (joint1->currentTransformation());
	  if (object2_->joint ()){ // object2_ = body part
	    JointPtr_t joint2 = object2_->joint ();
	    model::Transform3f M21, M2inv;
	    M2inv = joint2->currentTransformation();
	    M2inv.inverse ();
	    M21 = M2inv * M1;
	    return M21;
	  }
	  else{ // object2_ = fixed obstacle and has no joint
	    return M1;
	  }
	}

	void computeConstraint ()
	{
	robot_->currentConfiguration (constraintConfig_);
	robot_->computeForwardKinematics ();
	computeDistance ();

	std::vector<bool> m (3, true);
	std::vector<bool> m6 (6, true);
	JointPtr_t joint1 = object1_->joint ();
	model::vector3_t p1; p1.setZero (); // in local frame = origin
	model::vector3_t p2; p2.setZero (); // in local frame = origin
	model::matrix3_t I3; I3.setIdentity ();
	radius_ = joint1->linkedBody ()-> radius ();

	model::Transform3f Mref (computeReference ());
	if (object2_->joint ()){ // object2_ = body part
	  JointPtr_t joint2 = object2_->joint ();
	  f_ = RelativeTransformation::create (robot_, joint2,
							    joint1, Mref, m6);
	}
	else{ // object2_ = fixed obstacle and has no joint
	  f_ = Transformation::create(robot_, joint1, Mref, m6);
	}
	configProjector_ = ConfigProjector::create (robot_,
						    "collision constraint",
						    1e-6, 30);
	configProjector_->add (NumericalConstraint::create (f_));
      }


	/// Compute distance between object1 - object2 in the 'constraintConfig_'
	/// configuration
	void computeDistance ()
	{
	  model::DistanceResult result;
	  fcl::DistanceRequest distanceRequest (true, 0, 0, fcl::GST_INDEP);
	  robot_->currentConfiguration (constraintConfig_);
	  robot_->computeForwardKinematics ();
	  fcl::distance (object1_->fcl ().get (), object2_->fcl ().get (),
			 distanceRequest, result.fcl);
	  distance_ = result.distance ();
	}

      private:
	DevicePtr_t robot_;
	size_type robotNumberDofs_;
	size_type nbNonLockedDofs_;
	CollisionObjectPtr_t object1_;
	CollisionObjectPtr_t object2_;
	value_type posAlongLocalPath_; // posAlongLocalPath_ \in [0,1]
	size_type localPathId_; // index in global path
	ConfigProjectorPtr_t configProjector_;
	DifferentiableFunctionPtr_t f_;
	Configuration_t constraintConfig_;
	value_type distance_;
	value_type radius_;
	size_type rowInJacobian_;
	size_type nbWaypoints_;
	mutable matrix_t Jcompressed_;
      }; // struct CollisionConstraintsResult
      size_type CollisionConstraintsResult::fSize_ = 6;
      typedef std::vector <CollisionConstraintsResult>
      CollisionConstraintsResults_t;
    } // namespace pathOptimization
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_PATH_OPTIMIZATION_COLLISION_CONSTRAINTS_RESULT_HH
