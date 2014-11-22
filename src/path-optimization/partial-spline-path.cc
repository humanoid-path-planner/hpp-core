//
// Copyright (c) 2014 CNRS
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

#include <hpp/core/path-optimization/partial-spline-path.hh>
#include <roboptim/trajectory/cubic-b-spline.hh>

namespace hpp {
  namespace core {
    namespace pathOptimization {
	/// Constructor from parameters needed for a spline
      PartialSplinePath::PartialSplinePath
      (const DevicePtr_t& robot, const PathPtr_t& path,
       const JointVector_t& joints, const std::vector <value_type>& knots) :
	parent_t (path->timeRange (), path->outputSize ()),
	robot_ (robot), splineJoints_ (), pathJoints_ ()
      {
	JointVector_t::iterator itRobot = robot_->getJointVector ().begin ();
	JointVector_t::iterator itSpline = joints.begin ();
	splineOutPutSize = 0;
	while (itRobot != robot_->getJointVector ().end) {
	  if (*itSpline == *itRobot) {
	    splineJoints_.push_back (*itRobot);
	    ++itSpline;
	    splineOutPutSize += (*itRobot)->getConfigSize ();
	  } else {
	    pathJoints_.push_back (*itRobot);
	  }
	  ++itRobot;
	}
	assert (pathJoints_.size () + splineJoints_.size () ==
		robot_->getJointVector ().size ());
	assert (splineJoints_ == joints);
	
      }

      SplinePath::SplinePath (const roboptim::trajectory::CubicBSpline& spline):
	parent_t (spline.timeRange (), spline.outputSize ())
      {
	spline_ = boost::make_shared <roboptim::trajectory::CubicBSpline>
	  (spline.timeRange (), spline.outputSize (),spline.parameters (),
	   "cubic-b-spline");
      }

      SplinePath::SplinePath (const SplinePath& path) :
	parent_t (path), spline_ (path.spline_)
      {
      }

      void SplinePath::impl_compute (ConfigurationOut_t result,
				     value_type t) const
      {
	result = (*spline_) (t);
      }
    } // namespace pathOptimization
  } // namespace core
} // namespace hpp
