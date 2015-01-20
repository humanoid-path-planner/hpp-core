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

# include <hpp/core/config.hh>
# include <hpp/core/constraint.hh>
# include <hpp/core/comparison-type.hh>
# include <hpp/core/numerical-constraint.hh>
# include "hpp/core/deprecated.hh"

# include <hpp/statistics/success-bin.hh>

namespace hpp {
  namespace core {
    /// \addtogroup constraints
    /// \{

    /** Implicit non-linear constraint

     This class defines a numerical constraints on a robot configuration
     of the form:
     \f{eqnarray*}
     f_1 (\mathbf{q}) & = \mbox{or} \leq & f_1^0 \\
     & \vdots\\
     f_m (\mathbf{q}) & = \mbox{or} \leq & f_m^0
     \f}
     Functions \f$f_i\f$ are \ref constraints::DifferentiableFunction
     "differentiable functions". Vectors \f$f_i^0\f$ are called
     \b right hand side\b.

     The constraints are solved numerically by a Newton Raphson like method.
     Instances store locked degrees of freedom for performance optimisation.

     Numerical constraints can be added using method
     ConfigProjector::addFunction. Default parameter of this method define
     equality constraints, but inequality constraints can also be defined by
     passing an object of type ComparisonType to method.
    */
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

      /// Add a function
      /// \param function The function.
      /// \param comp For equality constraint, keep the default value.
      ///             For inequality constraint, it does a comparison to
      ///             whether the constraint is active.
      /// \deprecated Use add(NumericalConstraintPtr_t) instead.
      void  addFunction (const DifferentiableFunctionPtr_t& function,
          ComparisonTypePtr_t comp = ComparisonType::createDefault())
        HPP_CORE_DEPRECATED
      {
        add (NumericalConstraint::create (function, comp));
      }

      /// Add constraint
      /// \param constraint The function.
      /// \param comp For equality constraint, keep the default value.
      ///             For inequality constraint, it does a comparison to
      ///             whether the constraint is active.
      /// \deprecated Use addFunction instead.
      void addConstraint (const DifferentiableFunctionPtr_t& constraint,
			  ComparisonTypePtr_t comp = ComparisonType::createDefault ())
	HPP_CORE_DEPRECATED
      {
        add (NumericalConstraint::create (constraint, comp));
      }

      /// Add a numerical constraint
      /// \param numericalConstraint The numerical constraint.
      void add (const NumericalConstraintPtr_t& numericalConstraint);

      /// Add a locked joint.
      /// \param lockedJoint The locked joint.
      void add (const LockedJointPtr_t& lockedJoint);

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
      virtual void projectOnKernel (ConfigurationIn_t from,
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

      /// \name Right hand side of equalities - inequalities
      /// @{

      /// Set the right hand side from a configuration
      ///
      /// in such a way that the configuration satisfies the numerical
      /// constraints
      /// \param config the input configuration.
      /// \return the right hand side
      ///
      /// \warning Only values of the right hand side corresponding to 
      /// \link Equality "equality constraints" \endlink are set. As a
      /// result, the input configuration may not satisfy the other constraints.
      /// The rationale is the following. Equality constraints define a
      /// foliation of the configuration space. Leaves of the foliation are
      /// defined by the value of the right hand side of the equality
      /// constraints. This method is mainly used in manipulation planning
      /// to retrieve the leaf a configuration lies on.
      vector_t rightHandSideFromConfig (ConfigurationIn_t config);

      /// Set the level set parameter.
      /// \param param the level set parameter.
      void rightHandSide (const vector_t& param);

      /// Get the level set parameter.
      /// \return the parameter.
      vector_t rightHandSide () const;

      /// @}

      /// Check whether a configuration statisfies the constraint.
      virtual bool isSatisfied (ConfigurationIn_t config);

      /// Get the statistics
      ::hpp::statistics::SuccessStatistics& statistics()
      {
        return statistics_;
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
      typedef std::vector < NumericalConstraintPtr_t > NumericalConstraints_t;
      void resize ();
      void computeValueAndJacobian (ConfigurationIn_t configuration);
      void computeIntervals ();
      typedef std::list <LockedJointPtr_t> LockedJoints_t;
      DevicePtr_t robot_;
      NumericalConstraints_t functions_;
      LockedJoints_t lockedJoints_;
      /// Intervals of non locked degrees of freedom
      SizeIntervals_t  intervals_;
      value_type squareErrorThreshold_;
      size_type maxIterations_;
      vector_t rightHandSide_;
      size_type rhsReducedSize_;
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
      size_type nbLockedDofs_;
      value_type squareNorm_;
      ConfigProjectorWkPtr_t weak_;

      ::hpp::statistics::SuccessStatistics statistics_;
    }; // class ConfigProjector
    /// \}
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_CONFIG_PROJECTOR_HH
