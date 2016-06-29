// Copyright (c) 2015 CNRS
// Authors: Joseph Mirabel
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

#ifndef HPP_CORE_INTERPOLATED_PATH_HH
# define HPP_CORE_INTERPOLATED_PATH_HH

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>
# include <hpp/core/path.hh>

namespace hpp {
  namespace core {
    /// \addtogroup path
    /// \{

    /// Piecewise linear interpolation between two configurations
    ///
    /// This type of path is the return type of PathProjector algorithms.
    ///
    /// Degrees of freedom are interpolated depending on the type of
    /// \link hpp::model::Joint joint \endlink
    /// they parameterize:
    ///   \li linear interpolation for translation joints, bounded rotation
    ///       joints, and translation part of freeflyer joints,
    ///   \li angular interpolation for unbounded rotation joints,
    ///   \li constant angular velocity for SO(3) part of freeflyer joints.
    class HPP_CORE_DLLAPI InterpolatedPath : public Path
    {
    public:
      typedef std::pair <value_type, Configuration_t> InterpolationPoint_t;
      typedef std::map <value_type, Configuration_t, std::less <value_type>,
        Eigen::aligned_allocator <InterpolationPoint_t> > InterpolationPoints_t;
      typedef Path parent_t;

      /// Destructor
      virtual ~InterpolatedPath () throw () {}

      /// Create instance and return shared pointer
      /// \param device Robot corresponding to configurations
      /// \param init, end Start and end configurations of the path
      /// \param length Distance between the configurations.
      static InterpolatedPathPtr_t create (const DevicePtr_t& device,
				       ConfigurationIn_t init,
				       ConfigurationIn_t end,
				       value_type length)
      {
	InterpolatedPath* ptr = new InterpolatedPath (device, init, end, length);
	InterpolatedPathPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      /// Create instance and return shared pointer
      /// \param device Robot corresponding to configurations
      /// \param init, end Start and end configurations of the path
      /// \param length Distance between the configurations.
      /// \param constraints the path is subject to
      static InterpolatedPathPtr_t create (const DevicePtr_t& device,
				       ConfigurationIn_t init,
				       ConfigurationIn_t end,
				       value_type length,
				       ConstraintSetPtr_t constraints)
      {
	InterpolatedPath* ptr = new InterpolatedPath (device, init, end, length,
					      constraints);
	InterpolatedPathPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      /// Create copy and return shared pointer
      /// \param path path to copy
      static InterpolatedPathPtr_t createCopy (const InterpolatedPathPtr_t& path)
      {
	InterpolatedPath* ptr = new InterpolatedPath (*path);
	InterpolatedPathPtr_t shPtr (ptr);
	ptr->initCopy (shPtr);
	return shPtr;
      }

      /// Create an interpolation of this path
      /// \param path path to interpolate
      /// \param nbSamples number of samples between the initial and end
      ///        configuration
      /// \note it is assume that the constraints are constant along the path
      static InterpolatedPathPtr_t create (const PathPtr_t& path,
          const DevicePtr_t& device, const std::size_t& nbSamples);

      /// Create copy and return shared pointer
      /// \param path path to copy
      /// \param constraints the path is subject to
      static InterpolatedPathPtr_t createCopy
	(const InterpolatedPathPtr_t& path, const ConstraintSetPtr_t& constraints)
      {
	InterpolatedPath* ptr = new InterpolatedPath (*path, constraints);
	InterpolatedPathPtr_t shPtr (ptr);
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
      /// See Path::extract
      virtual PathPtr_t extract (const interval_t& subInterval) const
        throw (projection_error);

      virtual PathPtr_t reverse () const;

      /// Return the internal robot.
      DevicePtr_t device () const;

      /// Insert interpolation point
      void insert (const value_type& time, ConfigurationIn_t config)
      {
        configs_.insert (InterpolationPoint_t (time, config));
      }

      /// Get the initial configuration
      Configuration_t initial () const
      {
        return configs_.begin ()->second;
      }

      /// Get the final configuration
      Configuration_t end () const
      {
        return (--(configs_.end ()))->second;
      }

      const InterpolationPoints_t& interpolationPoints () const
      {
        return configs_;
      }

    protected:
      /// Print path in a stream
      virtual std::ostream& print (std::ostream &os) const
      {
	os << "InterpolatedPath:" << std::endl;
	os << "interval: [ " << timeRange ().first << ", "
	   << timeRange ().second << " ]" << std::endl;
	os << "initial configuration: " << initial().transpose () << std::endl;
	os << "final configuration:   " << end().transpose () << std::endl;
	return os;
      }

      /// Constructor
      InterpolatedPath (const DevicePtr_t& robot, ConfigurationIn_t init,
		    ConfigurationIn_t end, value_type length);

      /// Constructor with constraints
      InterpolatedPath (const DevicePtr_t& robot, ConfigurationIn_t init,
		    ConfigurationIn_t end, value_type length,
		    ConstraintSetPtr_t constraints);

      /// Copy constructor
      InterpolatedPath (const InterpolatedPath& path);

      /// Copy constructor with constraints
      InterpolatedPath (const InterpolatedPath& path,
		    const ConstraintSetPtr_t& constraints);

      void init (InterpolatedPathPtr_t self);

      void initCopy (InterpolatedPathPtr_t self);

      virtual bool impl_compute (ConfigurationOut_t result,
				 value_type param) const;

    private:
      DevicePtr_t device_;
      InterpolationPoints_t configs_;
      InterpolatedPathWkPtr_t weak_;
    }; // class InterpolatedPath
    /// \}
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_INTERPOLATED_PATH_HH
