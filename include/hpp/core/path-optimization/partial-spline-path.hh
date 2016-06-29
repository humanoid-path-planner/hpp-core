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

#ifndef HPP_CORE_PATH_OPTIMIZATION_PARTIAL_SPLINE_PATH_HH
# define HPP_CORE_PATH_OPTIMIZATION_PARTIAL_SPLINE_PATH_HH

# include <roboptim/trajectory/fwd.hh>
# include <hpp/model/device.hh>
# include <hpp/core/path.hh>
# include <hpp/core/path-vector.hh>
# include <boost/make_shared.hpp>

namespace hpp {
  namespace core {
    namespace pathOptimization {
      /// \addtogroup path
      /// \{

      /// Path for a robot partially defined by a cubic B-spline
      ///
      /// Some configuration variables specified at construction are 
      /// defined by an internal cubic B-spline while the other configuration
      /// variables are defined by a path given at construction.
      ///
      /// This enables users to perform numerical optimization on some joints
      /// only, keeping the trajectory of other joints constant.
      ///
      /// \todo Is this class used / useful ?
      class HPP_CORE_DLLAPI PartialSplinePath : public Path
      {
      public:
	typedef Path parent_t;
	/// \name Construction, destruction, copy
	/// \{

	/// Create instance  and return shared pointer
	static PartialSplinePathPtr_t create
	  (const PathPtr_t& path, const JointVector_t& joints,
	   const std::vector <value_type>& knots)
	{
	  PartialSplinePath* ptr = new PartialSplinePath (path, joints, knots);
	  PartialSplinePathPtr_t shPtr (ptr);
	  ptr->init (shPtr);
	  return shPtr;
	}
	/// Create instance from a CubicBSpline and return a shared pointer
	static PartialSplinePathPtr_t create
	  (const roboptim::trajectory::CubicBSpline& spline)
	{
	  PartialSplinePath* ptr = new PartialSplinePath (spline);
	  PartialSplinePathPtr_t shPtr (ptr);
	  ptr->init (shPtr);
	  return shPtr;
	}

	/// Return a shared pointer to a copy of this
	virtual PathPtr_t copy () const
	{
	  return createCopy (*this);
	}
	
	/// Destructor
	virtual ~PartialSplinePath () throw ()
	{
	}
	/// \}
      
	// TODO
	virtual std::ostream& print (std::ostream &os) const
	{
	  os << "PartialSplinePath:" << std::endl;
	  os << "interval: [ " << timeRange ().first << ", "
	     << timeRange ().second << " ]" << std::endl;
	  return os;
	}

      protected:
	/// Constructor
	PartialSplinePath (const DevicePtr_t& robot, const PathPtr_t& path,
			   const JointVector_t& joints,
			   const std::vector <value_type>& knots);
	
	/// Copy constructor
	PartialSplinePath (const PartialSplinePathPtr_t& path);
	
	void init (PartialSplinePathPtr_t self)
	{
	  parent_t::init (self);
	  weak_ = self;
	}
	virtual void impl_compute (ConfigurationOut_t result,
				   value_type t) const;

      private:
	/// Create instance and return shared pointer
	static PartialSplinePathPtr_t createCopy
	  (const PartialSplinePath& original)
	{
	  PartialSplinePath* ptr = new PartialSplinePath (original);
	  PartialSplinePathPtr_t shPtr (ptr);
	  ptr->init (shPtr);
	  return shPtr;
	}
	
	DevicePtr_t robot_;
	PathPtr_t path_;
	JointVector_t splineJoints_;
	JointVector_t pathJoints_;
	CubicBSplinePtr_t spline_;
	PartialSplinePathWkPtr_t weak_;
      }; // class PartialSplinePath      
      /// \}
    } // namespace pathOptimization
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_PATH_OPTIMIZATION_PARTIAL_SPLINE_PATH_HH
