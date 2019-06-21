//
// Copyright (c) 2016 CNRS
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

#ifndef HPP_CORE_REEDS_SHEPP_PATH_HH
# define HPP_CORE_REEDS_SHEPP_PATH_HH

# include <hpp/pinocchio/device.hh>

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>
# include <hpp/core/path-vector.hh>

namespace hpp {
  namespace core {
    /// \addtogroup path
    /// \{

    /// Car like motion
    ///
    /// Implement a Reeds and Shepp motion generation on the base joint.
    /// Degrees of freedom are interpolated depending on the type of
    /// \link hpp::pinocchio::Joint joint \endlink
    /// they parameterize:
    /// The following interpolation is made:
    ///   \li Reeds and Shepp interpolation for the base_joint_xy and
    ///       base_joint_rz
    ///   \li If the wheel joints are passed using setWheelJoints,
    ///       the configuration parameter of those joints are computed so that
    ///       the wheel is aligned with the velocity.
    ///   \li linear interpolation for the other joints
    class HPP_CORE_DLLAPI ReedsSheppPath : public PathVector
    {
    public:
      typedef core::PathVector parent_t;

      /// Destructor
      virtual ~ReedsSheppPath () throw () {}

      /// Create instance and return shared pointer
      /// \param device Robot corresponding to configurations
      /// \param init, end Start and end configurations of the path
      /// \param rho The radius of a turn.
      /// \param extraLength the length of the path due to the non RS DoF
      static ReedsSheppPathPtr_t create (const pinocchio::DevicePtr_t& device,
				   ConfigurationIn_t init,
				   ConfigurationIn_t end,
                                   value_type extraLength,
                                   value_type rho,
                                   size_type xyId, size_type rzId,
                                   const std::vector<JointPtr_t> wheels);

      /// Create instance and return shared pointer
      /// \param device Robot corresponding to configurations
      /// \param init, end Start and end configurations of the path
      /// \param rho The radius of a turn.
      /// \param extraLength the length of the path due to the non RS DoF
      /// \param constraints the path is subject to
      static ReedsSheppPathPtr_t create (const DevicePtr_t& device,
				   ConfigurationIn_t init,
				   ConfigurationIn_t end,
                                   value_type extraLength,
                                   value_type rho,
                                   size_type xyId, size_type rzId,
                                   const std::vector<JointPtr_t> wheels,
				   ConstraintSetPtr_t constraints);

      /// Create copy and return shared pointer
      /// \param path path to copy
      static ReedsSheppPathPtr_t createCopy (const ReedsSheppPathPtr_t& path)
      {
	ReedsSheppPath* ptr = new ReedsSheppPath (*path);
	ReedsSheppPathPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      /// Create copy and return shared pointer
      /// \param path path to copy
      /// \param constraints the path is subject to
      static ReedsSheppPathPtr_t createCopy
	(const ReedsSheppPathPtr_t& path, const ConstraintSetPtr_t& constraints)
      {
	ReedsSheppPath* ptr = new ReedsSheppPath (*path, constraints);
	ReedsSheppPathPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      /// Return a shared pointer to a copy of this object
      virtual PathPtr_t copy () const
      {
	return createCopy (weak_.lock ());
      }

      /// Return a shared pointer to a copy of this and set constraints
      ///
      /// \param constraints constraints to apply to the copy
      /// \pre *this should not have constraints.
      virtual PathPtr_t copy (const ConstraintSetPtr_t& constraints) const
      {
	return createCopy (weak_.lock (), constraints);
      }

      /// Return the internal robot.
      inline DevicePtr_t device () const
      {
        return device_;
      }

      /// Get the initial configuration
      inline Configuration_t initial () const
      {
        return initial_;
      }

      /// Get the final configuration
      inline Configuration_t end () const
      {
        return end_;
      }

    protected:
      /// Print path in a stream
      virtual std::ostream& print (std::ostream &os) const;
      /// Constructor
      ReedsSheppPath (const DevicePtr_t& robot, ConfigurationIn_t init,
                      ConfigurationIn_t end, value_type extraLength,
                      value_type rho,
                      size_type xyId, size_type rzId,
                      const std::vector<JointPtr_t> wheels);

      /// Constructor with constraints
      ReedsSheppPath (const DevicePtr_t& robot, ConfigurationIn_t init,
                      ConfigurationIn_t end, value_type extraLength,
                      value_type rho, size_type xyId, size_type rzId,
                      const std::vector<JointPtr_t> wheels,
		      ConstraintSetPtr_t constraints);

      /// Copy constructor
      ReedsSheppPath (const ReedsSheppPath& path);

      /// Copy constructor with constraints
      ReedsSheppPath (const ReedsSheppPath& path,
		    const ConstraintSetPtr_t& constraints);

      void init (ReedsSheppPathPtr_t self);

      /// Virtual implementation of derivative
      virtual void impl_derivative (vectorOut_t result, const value_type& t,
				    size_type order) const;

    private:
      typedef Eigen::Matrix<value_type, 5, 1> Lengths_t;

      // Compute path
      void buildReedsShepp (const JointPtr_t rz,
                            const std::vector<JointPtr_t> wheels);
      // Setup path
      void setupPath (const std::size_t& typeId, double t, double u=0.,
                      double v=0., double w=0., double x=0.);

      void CSC  (const vector2_t& xy, const vector2_t& csPhi, const double& phi);
      void CCC  (const vector2_t& xy, const vector2_t& csPhi, const double& phi);
      void CCCC (const vector2_t& xy, const vector2_t& csPhi, const double& phi);
      void CCSC (const vector2_t& xy, const vector2_t& csPhi, const double& phi);
      void CCSCC(const vector2_t& xy, const vector2_t& csPhi, const double& phi);


      DevicePtr_t device_;
      Configuration_t initial_;
      Configuration_t end_;
      const size_type xyId_,rzId_;
      size_type dxyId_,drzId_;
      std::size_t typeId_;
      Lengths_t lengths_;
      // Distance traveled by the center of the rear wheel axis
      value_type rsLength_;
      // Extra length to take into account the motion of other joints:
      // Length of path = rsLength_ + extraLength_
      value_type extraLength_;
      // Minimal radius of curvature
      value_type rho_;
      ReedsSheppPathWkPtr_t weak_;
    }; // class ReedsSheppPath
    /// \}
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_REEDS_SHEPP_PATH_HH
