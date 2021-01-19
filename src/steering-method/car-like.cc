// Copyright (c) 2017, CNRS
// Authors: Florent Lamiraux
//
// This file is part of hpp-core.
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
// hpp-core. If not, see <http://www.gnu.org/licenses/>.

#include <boost/algorithm/string.hpp>

#include <hpp/core/steering-method/car-like.hh>

#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/multibody/joint/joint-generic.hpp>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/joint-collection.hh>

#include <hpp/core/problem.hh>

namespace hpp {
  namespace core {
    namespace steeringMethod {
      CarLike::CarLike (const ProblemConstPtr_t& problem) :
	SteeringMethod (problem), device_ (problem->robot ()), rho_ (1.),
        xyId_ (0), rzId_ (2)
      {
        DevicePtr_t d (device_.lock());
        xy_ = d->getJointAtConfigRank(0);
        rz_ = d->getJointAtConfigRank(2);
        wheels_  = getWheelsFromeParameter(problem, rz_);
        turningRadius(problem->getParameter("SteeringMethod/Carlike/turningRadius").
            floatValue());
      }

      CarLike::CarLike (const ProblemConstPtr_t& problem,
			const value_type turningRadius,
			JointPtr_t xyJoint, JointPtr_t rzJoint,
			std::vector <JointPtr_t> wheels) :
        SteeringMethod (problem), device_ (problem->robot ()),
        rho_ (turningRadius), xy_ (xyJoint), rz_ (rzJoint),
        xyId_ (xy_->rankInConfiguration ()), wheels_ (wheels), weak_ ()
      {
        if (rz_->jointModel ().shortname () == "JointModelPlanar") {
          rzId_ = rz_->rankInConfiguration () + 2;
        } else {
          rzId_ = rz_->rankInConfiguration ();
        }
      }

      /// Copy constructor
      CarLike::CarLike (const CarLike& other) :
        SteeringMethod (other), device_ (other.device_),
        rho_ (other.rho_), xy_ (other.xy_), rz_ (other.rz_),
        xyId_ (other.xyId_), rzId_ (other.rzId_)
      {
      }

      void CarLike::turningRadius(const value_type& rho)
      {
        if (rho <= 0)
          throw std::invalid_argument("Turning radius must be strictly positive.");
        rho_ = rho;
      }

      std::vector <JointPtr_t> getWheelsFromeParameter
      (const ProblemConstPtr_t& problem, const JointPtr_t& rz)
      {
	std::vector <JointPtr_t> wheels;
	std::string p(problem->getParameter("SteeringMethod/Carlike/wheels").
		      stringValue());
	std::vector<std::string> wheelNames;
	boost::split(wheelNames, p, [](char c){return c == ',';});

        wheels.clear();
	for(const std::string& name : wheelNames) {
	  if (name == "") continue;
	  bool found(false);
	  for (std::size_t i = 0; i < rz->numberChildJoints(); ++i) {
	    JointPtr_t j = rz->childJoint(i);
	    if (j->name() == name) {
	      found = true;
	      if (j->configSize() != 1) {
		throw std::runtime_error
		  ("Carlike: wheel joint should be of dimension 1.");
	      }
	      wheels.push_back(j);
	      hppDout(info, "wheel: " << name);
	    }
	  }
	  if (!found) {
	    std::ostringstream os;
	    os << "CarLike: no joint with name \"" << name << "\".";
	    throw std::runtime_error(os.str());
	  }
	}
	return wheels;
      }

    } // namespace steeringMethod
  } // namespace core
} // namespace hpp
