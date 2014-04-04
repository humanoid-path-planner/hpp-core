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

#ifndef HPP_CORE_STRAIGHT_PATH_HH
# define HPP_CORE_STRAIGHT_PATH_HH

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>
# include <hpp/core/path.hh>

namespace hpp {
  namespace core {
    /// Linear interpolation between two configurations
    ///
    /// Degrees of freedom are interpolated depending on the type of
    /// \link hpp::model::Joint joint \endlink
    /// they parameterize:
    ///   \li linear interpolation for translation joints, bounded rotation
    ///       joints, and translation part of freeflyer joints,
    ///   \li angular interpolation for unbounded rotation joints,
    ///   \li constant angular velocity for SO(3) part of freeflyer joints.
    class HPP_CORE_DLLAPI StraightPath : public Path
    {
    public:
      typedef Path parent_t;
      typedef parent_t::argument_t argument_t;
      typedef parent_t::jacobian_t jacobian_t;
      typedef parent_t::size_type size_type;
      typedef parent_t::vector_t vector_t;
      typedef parent_t::value_type value_type;

      /// Destructor
      virtual ~StraightPath () throw () {}

      /// Create instance and return shared pointer
      /// \param device Robot corresponding to configurations
      /// \param init, end Start and end configurations of the path
      /// \param length Distance between the configurations.
      static StraightPathPtr_t create (const DevicePtr_t& device,
				       ConfigurationIn_t init,
				       ConfigurationIn_t end,
				       value_type length)
      {
	StraightPath* ptr = new StraightPath (device, init, end, length);
	StraightPathPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }
      /// Return a shared pointer to a copy of this
      virtual PathPtr_t copy () const
      {
	return createCopy (*this);
      }

    protected:
      /// Constructor
      StraightPath (const DevicePtr_t& robot, ConfigurationIn_t init,
		    ConfigurationIn_t end, value_type length);

      /// Copy constructor
      StraightPath (const StraightPath& path);

      void init (StraightPathPtr_t self)
      {
	parent_t::init (self);
	weak_ = self;
      }
      virtual void impl_compute (result_t& result, value_type param) const throw ();

    private:
      /// Create instance and return shared pointer
      static StraightPathPtr_t createCopy (const StraightPath& original)
      {
	StraightPath* ptr = new StraightPath (original);
	StraightPathPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      DevicePtr_t device_;
      Configuration_t initial_;
      Configuration_t end_;
      StraightPathWkPtr weak_;
    }; // class StraightPath
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_STRAIGHT_PATH_HH
