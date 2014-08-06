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

#ifndef HPP_CORE_CONFIG_PROJECTOR_HH
# define HPP_CORE_CONFIG_PROJECTOR_HH

# include <roboptim/core/differentiable-function.hh>
# include <hpp/core/config.hh>
# include <hpp/core/constraint.hh>

namespace hpp {
  namespace core {
    /// Implicit non-linear constraint
    ///
    /// Defined by a list of vector-valued functions and solved numerically
    /// by Newton Raphson like method.
    /// Store locked degrees of freedom for performance optimisation.
    class HPP_CORE_DLLAPI ConfigProjector : public Constraint
    {
    public:
      /// Return shared pointer to new object
      /// \param robot robot the constraint applies to.
      /// \param errorThreshold norm of the value of the constraint under which
      ///        the constraint is considered satified,
      /// \param maxIterations maximal number of iteration in the resolution of
      ///                      the constraint.
      static ConfigProjectorPtr_t create (const DevicePtr_t& robot,
					  const std::string& name,
					  value_type errorThreshold,
					  size_type maxIterations);

      /// Add constraint
      void addConstraint (const DifferentiableFunctionPtr_t& constraint);

      /// Get robot
      const DevicePtr_t& robot () const
      {
	return robot_;
      }

      /// Project configuration "to" on constraint tangent space in "from"
      ///
      /// \param from configuration,
      /// \param to configuration to project
      ///
      /// \f[
      /// \textbf{q}_{res} = \textbf{q}_{from} + \left(I_n -
      /// J^{+}J(\textbf{q}_{from})\right) (\textbf{q}_{to} - \textbf{q}_{from})
      /// \f]
      void projectOnKernel (ConfigurationIn_t from,
			    ConfigurationIn_t to, ConfigurationOut_t result);

      /// Set maximal number of iterations
      void maxIterations (size_type iterations)
      {
	maxIterations_ = iterations;
      }
      /// Get maximal number of iterations in config projector
      size_type maxIterations () const
      {
	return maxIterations_;
      }

      /// Set error threshold
      void errorThreshold (const value_type& threshold)
      {
	squareErrorThreshold_ = threshold * threshold;
      }
      /// Get errorimal number of threshold in config projector
      value_type errorThreshold () const
      {
	return sqrt (squareErrorThreshold_);
      }

      value_type residualError() const
      {
        return squareNorm_;
      }

    protected:
      /// Constructor
      /// \param robot robot the constraint applies to.
      /// \param errorThreshold norm of the value of the constraint under which
      ///        the constraint is considered satified,
      /// \param maxIterations maximal number of iteration in the resolution of
      ///                      the constraint.
      ConfigProjector (const DevicePtr_t& robot, const std::string& name,
		       value_type errorThreshold, size_type maxIterations);
      /// Store weak pointer to itself
      void init (const ConfigProjectorPtr_t& self)
      {
	Constraint::init (self);
	weak_ = self;
      }
      /// Numerically solve constraint
      virtual bool impl_compute (ConfigurationOut_t configuration);
      /// Set locked degrees of freedom to their locked values
      void computeLockedDofs (ConfigurationOut_t configuration);

    private:
      virtual std::ostream& print (std::ostream& os) const;
      virtual void addToConstraintSet (const ConstraintSetPtr_t& constraintSet);
      void smallToNormal (vectorIn_t small, vectorOut_t normal);
      void normalToSmall (vectorIn_t normal, vectorOut_t small);
      struct FunctionValueAndJacobian_t {
	FunctionValueAndJacobian_t (DifferentiableFunctionPtr_t f,
				    vector_t v, matrix_t j): function (f),
							     value (v),
							     jacobian (j) {}

	DifferentiableFunctionPtr_t function;
	vector_t value;
	matrix_t jacobian;
      }; // struct FunctionValueAndJacobian_t
      typedef std::vector < FunctionValueAndJacobian_t > NumericalConstraints_t;
      void resize ();
      void computeValueAndJacobian (ConfigurationIn_t configuration);
      virtual void addLockedDof (const LockedDofPtr_t& lockedDof);
      void computeIntervals ();
      typedef std::list <LockedDofPtr_t> LockedDofs_t;
      typedef std::vector < std::pair <size_type, size_type> >Intervals_t;
      typedef std::vector <size_type> Ranks_t;
      DevicePtr_t robot_;
      NumericalConstraints_t constraints_;
      LockedDofs_t lockedDofs_;
      Intervals_t allSO3ranks_;
      Intervals_t lockedSO3ranks_;
      /// Intervals of non locked degrees of freedom
      Intervals_t  intervals_;
      value_type squareErrorThreshold_;
      size_type maxIterations_;
      mutable vector_t value_;
      /// Jacobian without locked degrees of freedom
      mutable matrix_t reducedJacobian_;
      mutable matrix_t reducedProjector_;
      mutable vector_t toMinusFrom_;
      mutable vector_t toMinusFromSmall_;
      mutable vector_t projMinusFrom_;
      mutable vector_t projMinusFromSmall_;
      mutable vector_t dq_;
      mutable vector_t dqSmall_;
      size_type nbNonLockedDofs_;
      value_type squareNorm_;
      ConfigProjectorWkPtr_t weak_;
    }; // class ConfigProjector
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_CONFIG_PROJECTOR_HH
