//
// Copyright (c) 2016 CNRS
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

#include <pinocchio/multibody/joint/joint-generic.hpp>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/joint-collection.hh>

#include <hpp/core/distance/reeds-shepp.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method/reeds-shepp.hh>
#include <hpp/core/weighed-distance.hh>

namespace hpp {
  namespace core {
    namespace distance {

      DistancePtr_t ReedsShepp::clone () const
      {
	return createCopy (weak_.lock ());
      }

      ReedsSheppPtr_t ReedsShepp::create (const ProblemConstPtr_t& problem)
      {
	ReedsShepp* ptr (new ReedsShepp (problem));
	ReedsSheppPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }
      ReedsSheppPtr_t ReedsShepp::create
      (const ProblemConstPtr_t& problem, const value_type& turningRadius,
       JointPtr_t xyJoint, JointPtr_t rzJoint,
       std::vector <JointPtr_t> wheels)
      {
	ReedsShepp* ptr (new ReedsShepp (problem, turningRadius,
					 xyJoint, rzJoint, wheels));
	ReedsSheppPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      ReedsSheppPtr_t ReedsShepp::createCopy (const ReedsSheppPtr_t& distance)
      {
	ReedsShepp* ptr (new ReedsShepp (*distance));
	ReedsSheppPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      ReedsShepp::ReedsShepp (const ProblemConstPtr_t& problem) :
	Distance (),
	weighedDistance_(WeighedDistance::create(problem->robot())),
	device_(problem->robot()), xyId_(0), rzId_(2)
      {
        DevicePtr_t d (device_.lock());
        xy_ = d->getJointAtConfigRank(xyId_);
        rz_ = d->getJointAtConfigRank(rzId_);
	wheels_  = steeringMethod::getWheelsFromeParameter(problem, rz_);
	rho_ = problem->getParameter("SteeringMethod/Carlike/turningRadius").
	  floatValue();
      }

      ReedsShepp::ReedsShepp (const ProblemConstPtr_t& problem,
			      const value_type& turningRadius,
			      JointPtr_t xyJoint, JointPtr_t rzJoint,
			      std::vector <JointPtr_t> wheels) :
	Distance (), weighedDistance_(WeighedDistance::create
				      (problem->robot())),
	device_ (problem->robot ()), rho_ (turningRadius), xy_ (xyJoint),
	rz_ (rzJoint), xyId_ (xy_->rankInConfiguration ()), wheels_ (wheels),
	weak_ ()
      {
        if (rz_->jointModel ().shortname () == "JointModelPlanar") {
          rzId_ = rz_->rankInConfiguration () + 2;
        } else {
          rzId_ = rz_->rankInConfiguration ();
        }
      }
      
      ReedsShepp::ReedsShepp (const ReedsShepp& other) :
	Distance (other),
	weighedDistance_(WeighedDistance::createCopy(other.weighedDistance_)),
	device_(other.device_),	rho_(other.rho_), xy_(other.xy_),
	rz_(other.rz_), xyId_(other.xyId_), rzId_(other.rzId_),
	wheels_(other.wheels_), weak_()
      {
      }

      value_type ReedsShepp::impl_distance (ConfigurationIn_t q1,
					    ConfigurationIn_t q2) const
      {
        // TODO this should not be done here.
        // See todo in class ConstantCurvature
        Configuration_t qEnd (q2);
        qEnd.segment<2>(xyId_) = q1.segment<2>(xyId_);
        qEnd.segment<2>(rzId_) = q1.segment<2>(rzId_);
        // Do not take into account wheel joints in additional distance.
        for (std::vector<JointPtr_t>::const_iterator it = wheels_.begin ();
             it != wheels_.end (); ++it) {
          size_type i = (*it)->rankInConfiguration ();
          qEnd [i] = q1 [i];
        }
        // The length corresponding to the non RS DoF
        value_type extraL = (*weighedDistance_) (q1, qEnd);

	value_type distance;
        PathVectorPtr_t path
          (steeringMethod::reedsSheppPathOrDistance
	   (device_.lock (), q1, q2, extraL, rho_ , xyId_, rzId_, wheels_,
	    ConstraintSetPtr_t (), true, distance));
        return distance;
      }

      void ReedsShepp::init (const ReedsSheppWkPtr_t& weak)
      {
	weak_ = weak;
      }
			  
    } // namespace distance
  } //   namespace core
} // namespace hpp
