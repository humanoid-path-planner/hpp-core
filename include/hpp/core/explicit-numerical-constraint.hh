// Copyright (c) 2015, LAAS-CNRS
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

#ifndef HPP_CORE_EXPLICIT_NUMERICAL_CONSTRAINT_HH
# define HPP_CORE_EXPLICIT_NUMERICAL_CONSTRAINT_HH

# include <hpp/core/numerical-constraint.hh>

namespace hpp {
  namespace core {
    /// \addtogroup constraints
    /// \{

    /** Explicit numerical constraint

        An explicit numerical constraint is a constraint such that some
        configuration variables called \b output are function of the
        others called \b input.

        Let
         \li \f$(ic_{1}, \cdots, ic_{n_{ic}})\f$ be the list of indices
             corresponding to ordered input configuration variables,
         \li \f$(oc_{1}, \cdots, oc_{n_{oc}})\f$ be the list of indices
             corresponding to ordered output configuration variables,
         \li \f$(iv_{1}, \cdots, iv_{n_{iv}})\f$ be the list of indices
             corresponding to ordered input degrees of freedom,
         \li \f$(ov_{1}, \cdots, ov_{n_{ov}})\f$ be the list of indices
             corresponding to ordered output degrees of freedom.

        Recall that degrees of freedom refer to velocity vectors.

        Let us notice that \f$n_{ic} + n_{oc}\f$ is equal to the robot
        configuration size, and \f$n_{iv} + n_{ov}\f$ is equal to the velocity
        size.

        Then the differential function is of the form
        \f{equation*}
        \left(\begin{array}{c}
        q_{oc_{1}} \\ \vdots \\ q_{oc_{n_{oc}}}
        \end{array}\right) -
        f \left((q_{ic_{1}} \cdots q_{ic_{n_{ic}}})^T\right)
        \f}

        It is straightforward that equality constraint with this function can
        solved explicitely:

        \f{align*}
        \left(\begin{array}{c}
        q_{oc_{1}} \\ \vdots \\ q_{oc_{n_{oc}}}
        \end{array}\right) &-
        f \left((q_{ic_{1}} \cdots q_{ic_{n_{ic}}})^T\right) = rhs \\
        & \mbox {if and only if}\\
        \left(\begin{array}{c}
        q_{oc_{1}} \\ \vdots \\ q_{oc_{n_{oc}}}
        \end{array}\right) &=
        f \left((q_{ic_{1}} \cdots q_{ic_{n_{ic}}})^T\right) + rhs \\
        \f}

	\note If function \f$f\f$ takes values in a sub-manifold, the above
	impicit formulation does not make sense. In this case, users may
	derive method \code solve \endcode to implement its own resolution.

        Considered as a numerical constraint, the Jacobian of the differentiable
        function above is built as follows:

        for any index \f$i\f$ between 0 and the size of velocity vectors, either
        \li \f$\dot{q}_i\f$ is an input degree of freedom:
        \f$\exists j\in n_{iv}\f$ such that \f$i=iv_{j}\f$, or
        \li \f$\dot{q}_i\f$ is an output degree of freedom:
        \f$\exists j\in n_{ov}\f$ such that \f$i=ov_{j}\f$.

        \f{equation*}
        J = \left(\begin{array}{cccccccccccc}
        \cdots & ov_1 & \cdots & iv_{1} & \cdots & ov_2 & \cdots & iv_2 & \cdots & ov_{n_{ov}} & \cdots \\
               &  1   &        &        &        &  0   &        &      &        &             &        \\
               &  0   &        &        &        &  1   &        &      &        &             &        \\
               &       &        &  -\frac{\partial f}{q_1} & & &   & -\frac{\partial f}{q_2} \\
          &&&&&\\
               & 0    &        &       &         &  0   &        &      &        &  1
        \end{array}\right)
        \f}
    **/
    class HPP_CORE_DLLAPI ExplicitNumericalConstraint :
      public NumericalConstraint
    {
    public:
      /// Copy object and return shared pointer to copy
      virtual EquationPtr_t copy () const;
      /// Create instance and return shared pointer
      ///
      /// function relation between input configuration variables and output
      ///          configuration variables,
      /// outputConf set of integer intervals defining indices
      ///            \f$(oc_{1}, \cdots, oc_{n_{oc}})\f$,
      /// outputVeclocity set of integer defining indices
      ///            \f$(ov_{1}, \cdots, ov_{n_{ov}})\f$.
      /// \note comparison type for this constraint is always equality
      static ExplicitNumericalConstraintPtr_t create
        (const DifferentiableFunctionPtr_t& implicitFunction,
         const DifferentiableFunctionPtr_t& explicitFunction,
	 const SizeIntervals_t& inputConf,
	 const SizeIntervals_t& inputVelocity,
	 const SizeIntervals_t& outputConf,
	 const SizeIntervals_t& outputVelocity);

      /// Create instance and return shared pointer
      ///
      /// \param robot Robot for which the constraint is defined.
      /// \param function relation between input configuration variables and
      ///        output configuration variables,
      /// \param outputConf set of integer intervals defining indices
      ///            \f$(oc_{1}, \cdots, oc_{n_{oc}})\f$,
      /// \param outputVeclocity set of integer defining indices
      ///            \f$(ov_{1}, \cdots, ov_{n_{ov}})\f$.
      /// \param rhs        right hand side.
      /// \note comparison type for this constraint is always equality
      static ExplicitNumericalConstraintPtr_t create
        (const DevicePtr_t& robot, const DifferentiableFunctionPtr_t& explicitFunction,
	 const SizeIntervals_t& inputConf,
	 const SizeIntervals_t& inputVelocity,
	 const SizeIntervals_t& outputConf,
	 const SizeIntervals_t& outputVelocity);

      /// Create a copy and return shared pointer
      static ExplicitNumericalConstraintPtr_t createCopy
	(const ExplicitNumericalConstraintPtr_t& other);

      virtual DifferentiableFunctionPtr_t explicitFunction() const
      {
        return inputToOutput_;
      }

      /// Get output configuration variables
      const SizeIntervals_t& outputConf () const
      {
	return outputConf_;
      }
      /// Get output degrees of freedom
      const SizeIntervals_t& outputVelocity () const
      {
	return outputVelocity_;
      }
      /// Get input configuration variables
      const SizeIntervals_t& inputConf () const
      {
	return inputConf_;
      }
      /// Get input degrees of freedom
      const SizeIntervals_t& inputVelocity () const
      {
	return inputVelocity_;
      }
    protected:
      /// Constructor
      ///
      /// \param implicitFunction the implicit formulation of the constraint,
      /// \param explicitFunction relation between input configuration variables and
      ///          output configuration variables,
      /// \param outputConf set of integer intervals defining indices
      ///            \f$(oc_{1}, \cdots, oc_{n_{oc}})\f$,
      /// \param outputVeclocity set of integer defining indices
      ///            \f$(ov_{1}, \cdots, ov_{n_{ov}})\f$.
      /// \note comparison type for this constraint is always equality
      ExplicitNumericalConstraint
	(const DifferentiableFunctionPtr_t& implicitFunction,
         const DifferentiableFunctionPtr_t& explicitFormulation,
	 const SizeIntervals_t& inputConf,
	 const SizeIntervals_t& inputVelocity,
	 const SizeIntervals_t& outputConf,
	 const SizeIntervals_t& outputVelocity);

      /// Constructor
      ///
      /// \param robot Robot for which the constraint is defined.
      /// \param explicitFunction relation between input configuration variables and
      ///        output configuration variables,
      /// \param outputConf set of integer intervals defining indices
      ///            \f$(oc_{1}, \cdots, oc_{n_{oc}})\f$,
      /// \param outputVeclocity set of integer defining indices
      ///            \f$(ov_{1}, \cdots, ov_{n_{ov}})\f$.
      /// \param rhs        right hand side.
      /// \note comparison type for this constraint is always equality
      ExplicitNumericalConstraint
	(const DevicePtr_t& robot, const DifferentiableFunctionPtr_t& explicitFunction,
	 const SizeIntervals_t& inputConf,
	 const SizeIntervals_t& inputVelocity,
	 const SizeIntervals_t& outputConf,
	 const SizeIntervals_t& outputVelocity);

      /// Constructor
      ///
      /// \param implicitConstraint Function that is used when this constraint
      ///        is handled as an implicit constraint
      ///
      /// This constructor is aimed at specializing this class, in order to
      /// provide a function in case the default implicit function is not
      /// appropriate.
      ExplicitNumericalConstraint
	(const DifferentiableFunctionPtr_t& implicitConstraint,
	 const SizeIntervals_t& inputConf,
	 const SizeIntervals_t& inputVelocity,
	 const SizeIntervals_t& outputConf,
	 const SizeIntervals_t& outputVelocity);

      /// Constructor
      ///
      /// \param robot Robot for which the constraint is defined.
      /// \param function relation between input configuration variables and
      ///        output configuration variables,
      /// \param outputConf set of integer intervals defining indices
      ///            \f$(oc_{1}, \cdots, oc_{n_{oc}})\f$,
      /// \param outputVeclocity set of integer defining indices
      ///            \f$(ov_{1}, \cdots, ov_{n_{ov}})\f$.
      /// \param rhs        right hand side.
      /// \note comparison type for this constraint is always equality
      ExplicitNumericalConstraint (const ExplicitNumericalConstraint& other);

      // Store weak pointer to itself
      void init (const ExplicitNumericalConstraintWkPtr_t& weak)
	{
	  NumericalConstraint::init (weak);
	  weak_ = weak;
	}
    private:
      // Relation between input and output configuration variables
      DifferentiableFunctionPtr_t inputToOutput_;
      SizeIntervals_t inputConf_;
      SizeIntervals_t inputVelocity_;
      SizeIntervals_t outputConf_;
      SizeIntervals_t outputVelocity_;
      ExplicitNumericalConstraintWkPtr_t weak_;
    }; // class ExplicitNumericalConstraint
    /// \}
  } // namespace core
} // namespace core

#endif // HPP_CORE_EXPLICIT_NUMERICAL_CONSTRAINT_HH
