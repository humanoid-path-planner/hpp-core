//
// Copyright (c) 2014 CNRS
// Authors: Florian Valenza
//
// This file is part of hpp-core
//
// roboptim is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// roboptim is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with roboptim.  If not, see <http://www.gnu.org/licenses/>.


#include <roboptim/trajectory/sys.hh>

#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/optional.hpp>
#include <boost/make_shared.hpp>

#include <hpp/model/collision-object.hh>
#include <hpp/model/joint.hh>

#include <hpp/core/spline-cost.hh>
#include <hpp/core/path-vector.hh>



// ---------------------------------------------  BASIC COST ------------------------------------------------------------- //
// ----------------------------------------------------------------------------------------------------------------------- //
#ifdef basicCost

namespace hpp {
	namespace core{
		
  namespace
  {
    struct SumLength
    {
      SumLength (const CubicBSpline_t& traj, double& res)
	: traj_ (traj),
	  res_ (res)
      {}

      void operator () (const double& t)
      {
	res_ += traj_.derivative (t, 1).dot(traj_.derivative (t, 1));
      }

    private:
      const CubicBSpline_t& traj_;
      double& res_;
    };

    struct SumLengthGrad
    {
      SumLengthGrad (const CubicBSpline_t& traj,
		     SplineCost::gradient_t& grad)
	: traj_ (traj),
	  grad_ (grad)
      {}

      void operator () (const double& t)
      {
	grad_ += traj_.derivative (t, 1).adjoint ()
	  * traj_.variationDerivWrtParam (t, 1);
      }

    private:
      const CubicBSpline_t& traj_;
      SplineCost::gradient_t& grad_;
    };

  }

  SplineCost::SplineCost (const CubicBSpline_t& spline,
			      size_type nDiscretizationPoints,
			      boost::optional<interval_t> interval)
    : TrajectoryCost<CubicBSpline_t> (spline, "spline length"),
      interval_ (interval ? *interval : spline.timeRange ()),
      nDiscretizationPoints_ (nDiscretizationPoints)
  {
  }

  SplineCost::~SplineCost () throw()
  {
  }
  
  void
  SplineCost::impl_compute (result_t& res, const argument_t& p)
    const
  {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
      Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

    trajectory_t traj = trajectory_;
    traj.setParameters (p);

    res [0] = 0;
    SumLength sumlength (traj, res[0]);
    foreach (interval_, nDiscretizationPoints_, sumlength);

    const value_type delta =
      getUpperBound (interval_) - getLowerBound (interval_);
    res[0] *= delta / (value_type)nDiscretizationPoints_;
    res[0] /= 2.;
  }

  void
  SplineCost::impl_gradient (gradient_t& grad, const argument_t& p,
			       size_type i=0)
    const
  {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
      Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

    //assert (i == 0);
    grad.setZero ();

    trajectory_t traj = trajectory_;
    traj.setParameters (p);

    SumLengthGrad sumlengthgrad (traj, grad);
    foreach (interval_, nDiscretizationPoints_, sumlengthgrad);
    const value_type delta =
      getUpperBound (interval_) - getLowerBound (interval_);
    grad *= delta / (value_type)nDiscretizationPoints_;
  }
  } //   namespace core
} // namespace hpp







// --------------------------------------------- QUADRATIC ERROR ---------------------------------------------------------- //
// ----------------------------------------------------------------------------------------------------------------------- //
#elif defined quadraticError

namespace hpp {
	namespace core{
		
double computeSigmaPow(const CubicBSpline_t& spline, const PathVectorPtr_t& originalPath, const double& t){
	
	
	vector_t q1 (spline.outputSize ());
	spline (q1,t);
	
	vector_t q2 ((*originalPath)(t));
	vector_t reducedQ2(spline.outputSize ()) ;
	reducedQ2[0] = q2[0];reducedQ2[1] = q2[1];
	
	vector_t diff = reducedQ2 - q1;
	double sigma = diff.dot(diff);
	
	return sigma;
}
		
  namespace
  {
    struct SumLength
    {
      SumLength (const CubicBSpline_t& traj, double& res,PathVectorPtr_t originalPV)
	: traj_ (traj),
	  res_ (res),
	  pv_ (originalPV)
      {
		  }

      void operator () (const double& t)
      {
		 double sigmaPow =  computeSigmaPow(traj_,pv_,t);
		 res_ += sigmaPow;	
      }

    private:
      const CubicBSpline_t& traj_;
      const PathVectorPtr_t& pv_;
      double& res_;
    };

    struct SumLengthGrad
    {
      SumLengthGrad (const CubicBSpline_t& traj,
		     SplineCost::gradient_t& grad,
		     const PathVectorPtr_t& originalPV)
	: traj_ (traj),
	  grad_ (grad),
	  pv_ (originalPV)
      {
		  }

      void operator () (const double& t)
      {
	vector_t qs ((*pv_)(t));
	vector_t reducedQs(traj_.outputSize ());
	reducedQs[0] = qs[0];reducedQs[1] = qs[1];
	vector_t q (traj_.outputSize ());
	traj_ (q,t);
	vector_t diff = q - reducedQs;
	
	SplineCost::gradient_t step =  2 * diff.adjoint() *traj_.variationConfigWrtParam(t);
	grad_ += step;
      }

    private:
      const CubicBSpline_t& traj_;
      const PathVectorPtr_t& pv_;
      SplineCost::gradient_t& grad_;
    };

  }

  SplineCost::SplineCost (const CubicBSpline_t& spline,
			      const PathVectorPtr_t& straightPath,
			      size_type nDiscretizationPoints,
			      boost::optional<interval_t> interval)
    : TrajectoryCost<CubicBSpline_t> (spline, "spline length"),
      interval_ (interval ? *interval : spline.timeRange ()),
      nDiscretizationPoints_ (nDiscretizationPoints),
      blPath_ (straightPath)
  {
	  }

  SplineCost::~SplineCost () throw()
  {
  }

  void
  SplineCost::impl_compute (result_t& res, const argument_t& p)
    const
  {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
      Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

    trajectory_t traj = trajectory_;
    traj.setParameters (p);

    res [0] = 0;
    SumLength sumlength (traj, res[0],blPath_);
    foreach (interval_, nDiscretizationPoints_, sumlength);

    const value_type delta =
      getUpperBound (interval_) - getLowerBound (interval_);
    res[0] *= delta / (value_type)nDiscretizationPoints_;
    res[0] /= 2.;
  }

  void
  SplineCost::impl_gradient (gradient_t& grad, const argument_t& p,
			       size_type i=0)
    const
  {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
      Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

    grad.setZero ();

    trajectory_t traj = trajectory_;
    traj.setParameters (p);

    SumLengthGrad sumlengthgrad (traj, grad,blPath_);
    foreach (interval_, nDiscretizationPoints_, sumlengthgrad);
    const value_type delta =
      getUpperBound (interval_) - getLowerBound (interval_);
    grad *= delta / (value_type)nDiscretizationPoints_;
  }
  } //   namespace core
} // namespace hpp






// --------------------------------------------- OBSTACLE AVOIDANCE ----------------------------------------------------------- //
// ----------------------------------------------------------------------------------------------------------------------- //
#elif defined obstacleAvoidance

namespace hpp {
	namespace core{
		
		
static void cross (const fcl::Vec3f& v, matrix_t& m)
    {
      m (0,1) = -v [2]; m (1,0) = v [2];
      m (0,2) = v [1]; m (2,0) = -v [1];
      m (1,2) = -v [0]; m (2,1) = v [0];
    }

double computeMinDistanceFromObstacles (const CubicBSpline_t& spline, const PathVectorPtr_t& originalPath, hpp::model::DevicePtr_t device,const double& t){
	
	CollisionObjectPtr_t innerObject;
	CollisionObjectPtr_t outerObject;
	bool inner2Robot,outer2Robot;
	bool existObstacle = false;
	double dmin,currentDist;
	
	hpp::model::DevicePtr_t dev = hpp::model::Device::createCopy(device);
	
	vector_t reducedQs(spline(t));
	vector_t qs ((*originalPath)(t));
	qs[0] = reducedQs[0];
	qs[1] = reducedQs[1]; 
	dev -> currentConfiguration (qs);
	dev -> computeForwardKinematics();
	dev -> computeDistances();
	
	const model::DistanceResults_t& dr = dev->distanceResults ();
	
	// Get the min distance between a pair of obstacle/robot objects
	for (model::DistanceResults_t::const_iterator itDistance =
	       dr.begin (); itDistance != dr.end (); itDistance++) {
			   
			   innerObject = itDistance->innerObject;
			   outerObject = itDistance->outerObject;
			   currentDist = itDistance->distance();
			   
			   // Is the pair between robot and obstacle ?
			   inner2Robot = !(innerObject->joint() == NULL);
			   outer2Robot = !(outerObject->joint() == NULL);
			   
			   if ((!inner2Robot && outer2Robot) || (inner2Robot && !outer2Robot)){ // if yes
					existObstacle = true;
					if(currentDist < dmin){
					   dmin = currentDist;
				   }
			   }
	}
	
	if (existObstacle){
	} else {
		dmin = 0;
	}

	return dmin;
}

SplineCost::gradient_t derivMinDistanceFromObstacles(const CubicBSpline_t& spline, const PathVectorPtr_t& originalPath, hpp::model::DevicePtr_t device,const double& t){
	
	SplineCost::gradient_t deriv(spline.parameters().size());
	deriv.setZero ();
	
	CollisionObjectPtr_t innerObject;
	CollisionObjectPtr_t outerObject;
	fcl::Vec3f closestPointInner;
	fcl::Vec3f closestPointOuter;
	model::JointConstPtr_t jointInnerObject;
	model::JointConstPtr_t jointOuterObject;
	JointJacobian_t jacobianInnerObject;
	JointJacobian_t jacobianOuterObject;
	bool inner2Robot,outer2Robot;
	bool existObstacle = false;
	double dmin=10e6;
	double currentDist;
	hpp::model::DevicePtr_t dev = hpp::model::Device::createCopy(device);
	
	vector_t reducedQs(spline(t));
	vector_t qs ((*originalPath)(t));
	qs[0]=reducedQs[0];qs[1] = reducedQs[1];
	dev -> currentConfiguration (qs);
	dev -> computeForwardKinematics();
	dev -> computeDistances();
	
	const model::DistanceResults_t& dr = dev->distanceResults ();
	
	// Get the min distance between a pair of obstacle/robot objects
	for (model::DistanceResults_t::const_iterator itDistance =
	       dr.begin (); itDistance != dr.end (); itDistance++) {
			   
			   innerObject = itDistance->innerObject;
			   outerObject = itDistance->outerObject;
			   currentDist = itDistance->distance();
			   
			   // Is the pair between robot and obstacle ?
			   inner2Robot = !(innerObject->joint() == NULL);
			   outer2Robot = !(outerObject->joint() == NULL);
			   
			   if ((!inner2Robot && outer2Robot) || (inner2Robot && !outer2Robot)){ //if yes
					existObstacle = true;
					
				   if(currentDist < dmin){
					   dmin = currentDist;
					   
					   if(inner2Robot){ //If inner attached to robot
						   closestPointInner = itDistance->closestPointInner ();
						   closestPointOuter = itDistance->closestPointOuter ();
						   
						   jointInnerObject = innerObject->joint ();
						   jointOuterObject = outerObject->joint ();
						   
						   jacobianInnerObject = jointInnerObject->jacobian ();
//						   jacobianOuterObject = jointOuterObject->jacobian ();
						} else {
							closestPointInner = itDistance->closestPointOuter ();
							closestPointOuter = itDistance->closestPointInner ();
					   
							jointInnerObject = outerObject->joint ();
							jointOuterObject = innerObject->joint ();
					   
							jacobianInnerObject = jointInnerObject->jacobian ();
//					   		jacobianOuterObject = jointOuterObject->jacobian ();
						}
				   }
			   }
		   }
	
	if (existObstacle){

		// Computation of the jacobian 
		const fcl::Transform3f& M1 = jointInnerObject->currentTransformation ();

		fcl::Vec3f R1x1 = M1.getRotation () * closestPointInner;
		matrix_t cross1 (3,3);
		// -[R1 (q) x1]x
		cross (-R1x1, cross1);

		matrix_t jacobian = cross1 * jacobianInnerObject.bottomRows (3) + jacobianInnerObject.topRows (3);
	    

		// Computation of the derivative of the distances 
		const fcl::Vec3f OminusR = closestPointInner - closestPointOuter;
		vector_t diff(3);
		diff[0] = OminusR[0];
		diff[1] = OminusR[1];
		diff[2] = OminusR[2];
		
		deriv = 1/dmin * diff.transpose() *jacobian.leftCols(2) * spline.variationConfigWrtParam(t);
	} else {
		// No obstacle
	}

	return deriv;
}
		
  namespace
  {
    struct SumLength
    {
      SumLength (const CubicBSpline_t& traj, double& res, const PathVectorPtr_t& originalPV, hpp::model::DevicePtr_t dev)
	: traj_ (traj),
	  res_ (res),
	  pv_ (originalPV),
	  dev_ (dev)
      {}

      void operator () (const double& t)
      {
		  //compute distance min from obstacles
		  double dmin = computeMinDistanceFromObstacles(traj_,pv_,dev_,t);
		  double J1 = traj_.derivative (t, 1).dot(traj_.derivative (t, 1));
		  double J2; // penalty cost
		  
		  if (!dmin) { // If no obstacle, no penalty
			  J2 = 0; 
		  } else {
			  J2 = ALPHA * ( 1/(dmin + EPS) - 1/(DI + EPS));
		  }
		  double J_step = J1 + J2;
		  
		  res_ += J_step;
      }

    private:
      const CubicBSpline_t& traj_;
      const PathVectorPtr_t& pv_;
      DevicePtr_t dev_;
      double& res_;
    };

    struct SumLengthGrad
    {
      SumLengthGrad (const CubicBSpline_t& traj,
		     SplineCost::gradient_t& grad,const PathVectorPtr_t& originalPV, hpp::model::DevicePtr_t dev)
	: traj_ (traj),
	  grad_ (grad),
	  pv_ (originalPV),
	  dev_ (dev)
      {}

      void operator () (const double& t)
      {
			double dmin = computeMinDistanceFromObstacles(traj_,pv_,dev_,t);
			SplineCost::gradient_t du = derivMinDistanceFromObstacles(traj_,pv_,dev_,t);	  
			SplineCost::gradient_t step1 = 2 * traj_.derivative (t, 1).adjoint () * traj_.variationDerivWrtParam (t, 1);
			SplineCost::gradient_t penaltyGrad =	- ALPHA *(du)/ pow(dmin + EPS,2);
			SplineCost::gradient_t step = step1 + penaltyGrad;
											
			grad_ += step;
      }

    private:
      const CubicBSpline_t& traj_;
      const PathVectorPtr_t& pv_;
      DevicePtr_t dev_;
      SplineCost::gradient_t& grad_;
    };

  }

  SplineCost::SplineCost (const CubicBSpline_t& spline,
			      const DevicePtr_t& device,
			      const PathVectorPtr_t& straightPath,
			      size_type nDiscretizationPoints,
			      boost::optional<interval_t> interval)
    : TrajectoryCost<CubicBSpline_t> (spline, "spline length"),
      interval_ (interval ? *interval : spline.timeRange ()),
      nDiscretizationPoints_ (nDiscretizationPoints),
      blPath_ (straightPath),
      device_ (device)
  {
  }

  SplineCost::~SplineCost () throw()
  {
  }

  void
  SplineCost::impl_compute (result_t& res, const argument_t& p)
    const
  {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
      Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

    trajectory_t traj = trajectory_;
    traj.setParameters (p);

    res [0] = 0;
    SumLength sumlength (traj, res[0],blPath_,device_);
    foreach (interval_, nDiscretizationPoints_, sumlength);

    const value_type delta =
      getUpperBound (interval_) - getLowerBound (interval_);
    res[0] *= delta / (value_type)nDiscretizationPoints_;
    res[0] /= 2.;
  }

  void
  SplineCost::impl_gradient (gradient_t& grad, const argument_t& p,
			       size_type i=0)
    const
  {
#ifndef ROBOPTIM_DO_NOT_CHECK_ALLOCATION
      Eigen::internal::set_is_malloc_allowed (true);
#endif //! ROBOPTIM_DO_NOT_CHECK_ALLOCATION

    //assert (i == 0);
    grad.setZero ();

    trajectory_t traj = trajectory_;
    traj.setParameters (p);

    SumLengthGrad sumlengthgrad (traj, grad,blPath_,device_);
    foreach (interval_, nDiscretizationPoints_, sumlengthgrad);
    const value_type delta =
      getUpperBound (interval_) - getLowerBound (interval_);
    grad *= delta / (value_type)nDiscretizationPoints_;
  }
  } //   namespace core
} // namespace hpp
#endif
