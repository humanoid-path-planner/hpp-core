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

      /// Create instance and return shared pointer
      /// \param device Robot corresponding to configurations
      /// \param init, end Start and end configurations of the path
      /// \param length Distance between the configurations.
      /// \param constraints the path is subject to
      static StraightPathPtr_t create (const DevicePtr_t& device,
				       ConfigurationIn_t init,
				       ConfigurationIn_t end,
				       value_type length,
				       ConstraintSetPtr_t constraints)
      {
	StraightPath* ptr = new StraightPath (device, init, end, length,
					      constraints);
	StraightPathPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      /// Create copy and return shared pointer
      /// \param path path to copy
      static StraightPathPtr_t createCopy (const StraightPathPtr_t& path)
      {
	StraightPath* ptr = new StraightPath (*path);
	StraightPathPtr_t shPtr (ptr);
	ptr->initCopy (shPtr);
	return shPtr;
      }

      /// Create copy and return shared pointer
      /// \param path path to copy
      /// \param constraints the path is subject to
      static StraightPathPtr_t createCopy
	(const StraightPathPtr_t& path, const ConstraintSetPtr_t& constraints)
      {
	StraightPath* ptr = new StraightPath (*path, constraints);
	StraightPathPtr_t shPtr (ptr);
	ptr->initCopy (shPtr);
	return shPtr;
      }

      /// Return a shared pointer to this
      ///
      /// As StaightPath are immutable, and refered to by shared pointers,
      /// they do not need to be copied.
      virtual PathPtr_t copy () const
      {
	return createCopy (weak_.lock ());
      }

      /// Return a shared pointer to a copy of this and set constraints
      ///
      /// \param constraints constraints to apply to the copy
      /// \precond *this should not have constraints.
      virtual PathPtr_t copy (const ConstraintSetPtr_t& constraints) const
      {
	return createCopy (weak_.lock (), constraints);
      }


      /// Extraction/Reversion of a sub-path
      /// \param subInterval interval of definition of the extract path
      /// If upper bound of subInterval is smaller than lower bound,
      /// result is reversed.
      virtual PathPtr_t extract (const interval_t& subInterval) const
        throw (projection_error);

      /// Modify initial configuration
      /// \param initial new initial configuration
      /// \pre input configuration should be of the same size as current initial
      /// configuration
      void initialConfig (ConfigurationIn_t initial)
      {
	assert (initial.size () == initial_.size ());
	initial_ = initial;
      }

      /// Modify end configuration
      /// \param end new end configuration
      /// \pre input configuration should be of the same size as current end
      /// configuration
      void endConfig (ConfigurationIn_t end)
      {
	assert (end.size () == end_.size ());
	end_ = end;
      }
      
      /// Return the internal robot.
      DevicePtr_t device () const;

      /// Get the initial configuration
      Configuration_t initial () const
      {
        return initial_;
      }

      /// Get the final configuration
      Configuration_t end () const
      {
        return end_;
      }

    protected:
      /// Print path in a stream
      virtual std::ostream& print (std::ostream &os) const
      {
	os << "StraightPath:" << std::endl;
	os << "interval: [ " << timeRange ().first << ", "
	   << timeRange ().second << " ]" << std::endl;
	os << "initial configuration: " << initial_.transpose () << std::endl;
	os << "final configuration:   " << end_.transpose () << std::endl;
	return os;
      }
      /// Constructor
      StraightPath (const DevicePtr_t& robot, ConfigurationIn_t init,
		    ConfigurationIn_t end, value_type length);

      /// Constructor with constraints
      StraightPath (const DevicePtr_t& robot, ConfigurationIn_t init,
		    ConfigurationIn_t end, value_type length,
		    ConstraintSetPtr_t constraints);

      /// Copy constructor
      StraightPath (const StraightPath& path);

      /// Copy constructor with constraints
      StraightPath (const StraightPath& path,
		    const ConstraintSetPtr_t& constraints);

      void init (StraightPathPtr_t self)
      {
	parent_t::init (self);
	weak_ = self;
        checkPath ();
      }

      void initCopy (StraightPathPtr_t self)
      {
	parent_t::initCopy (self);
	weak_ = self;
      }

      virtual bool impl_compute (ConfigurationOut_t result,
				 value_type param) const;

    private:
      DevicePtr_t device_;
      Configuration_t initial_;
      Configuration_t end_;
      StraightPathWkPtr_t weak_;
    }; // class StraightPath
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_STRAIGHT_PATH_HH
