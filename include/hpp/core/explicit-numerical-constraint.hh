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

        Let us notice that \f$n_{ic} + n_{oc}\f$ is less than the robot
        configuration size, and \f$n_{iv} + n_{ov}\f$ is less than the velocity
        size. Some degrees of freedom may indeed be neither input nor output.

        Then the differential function is of the form
        \f{equation*}{
        \left(\begin{array}{c}
        q_{oc_{1}} \\ \vdots \\ q_{oc_{n_{oc}}}
        \end{array}\right) -
        f \left((q_{ic_{1}} \cdots q_{ic_{n_{ic}}})^T\right)
        \f}
        It is straightforward that an equality constraint with this function can
        solved explicitely:
        \f{align*}{
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
	If function \f$f\f$ takes values in a Lie group (SO(2), SO(3)),
        the above "+" between a Lie group element and a tangent vector
        has to be undestood as the integration of the constant velocity from
        the Lie group element:
        \f{equation*}{
        \mathbf{q} + \mathbf{v} = \mathbf{q}.\exp (\mathbf{v})
        \f}
        where \f$\mathbf{q}\f$ is a Lie group element and \f$\mathbf{v}\f$ is a
        tangent vector.

        Considered as a NumericalConstraint, the expression of the Jacobian of
        the DifferentiableFunction above depends on the output space of function
        \f$f\f$. The rows corresponding to values in a vector space are
        expressed as follows.

        for any index \f$i\f$ between 0 and the size of velocity vectors, either
        \li \f$\dot{q}_i\f$ is an input degree of freedom:
        \f$\exists j\f$ integer, \f$1 \leq j \leq n_{iv}\f$ such that
        \f$i=iv_{j}\f$,
        \li \f$\dot{q}_i\f$ is an output degree of freedom:
        \f$\exists j\f$ integer, \f$1\leq j \leq n_{ov}\f$ such that
        \f$i=ov_{j}\f$, or
        \li \f$\dot{q}_i\f$ neither input nor output. In this case, the
        corresponding column is equal to 0.
        \f{equation*}{
        J = \left(\begin{array}{cccccccccccc}
        \cdots & ov_1 & \cdots & iv_{1} & \cdots & ov_2 & \cdots & iv_2 & \cdots & ov_{n_{ov}} & \cdots \\
               &  1   &        &        &        &  0   &        &      &        &             &        \\
               &  0   &        &        &        &  1   &        &      &        &             &        \\
               &       &        &  -\frac{\partial f}{q_1} & & &   & -\frac{\partial f}{q_2} \\
          &&&&&\\
               & 0    &        &       &         &  0   &        &      &        &  1
        \end{array}\right)
        \f}
        The rows corresponding to values in SO(3) have the following expression.
        \f{equation*}{
        J = \left(\begin{array}{cccccccccccc}
        ov_1 \ ov_2 \ ov_3 & iv_1 \cdots  iv_{n_{iv}} \\
        J_{log}(R_{f}^T R_{out}) & -J_{log}(R_{f}^T R_{out})R_{out}^T \frac{\partial f}{\partial q_{in}}
        \end{array}\right)
        \f}
        where
        \li \f$R_{out}\f$ is the rotation matrix corresponding to unit
        quaternion \f$(q_{oc1},q_{oc2},q_{oc3},q_{oc4})\f$,
        \li \f$R_{f}\f$ is the rotation matrix corresponding to the part of the
        output value of \f$f\f$ corresponding to SO(3),
        \li \f$J_{log}\f$ is the Jacobian matrix of function that associates
        to a rotation matrix \f$R\f$ the vector \f$\omega\f$ such that
        \f{equation*}{
        R = \exp (\left[\omega\right]_{\times})
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
        (const DevicePtr_t& robot,
         const DifferentiableFunctionPtr_t& function,
	 const segments_t& inputConf,
	 const segments_t& inputVelocity,
	 const segments_t& outputConf,
	 const segments_t& outputVelocity);

      /// Create a copy and return shared pointer
      static ExplicitNumericalConstraintPtr_t createCopy
	(const ExplicitNumericalConstraintPtr_t& other);

      virtual DifferentiableFunctionPtr_t explicitFunction() const
      {
        return inputToOutput_;
      }

      /// Get output configuration variables
      const segments_t& outputConf () const
      {
	return outputConf_;
      }
      /// Get output degrees of freedom
      const segments_t& outputVelocity () const
      {
	return outputVelocity_;
      }
      /// Get input configuration variables
      const segments_t& inputConf () const
      {
	return inputConf_;
      }
      /// Get input degrees of freedom
      const segments_t& inputVelocity () const
      {
	return inputVelocity_;
      }
    protected:
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
      ExplicitNumericalConstraint
	(const DevicePtr_t& robot, const DifferentiableFunctionPtr_t& function,
	 const segments_t& inputConf,
	 const segments_t& inputVelocity,
	 const segments_t& outputConf,
	 const segments_t& outputVelocity);

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
	 const segments_t& inputConf,
	 const segments_t& inputVelocity,
	 const segments_t& outputConf,
	 const segments_t& outputVelocity);

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
      segments_t inputConf_;
      segments_t inputVelocity_;
      segments_t outputConf_;
      segments_t outputVelocity_;
      ExplicitNumericalConstraintWkPtr_t weak_;
    }; // class ExplicitNumericalConstraint
    /// \}
  } // namespace core
} // namespace core

#endif // HPP_CORE_EXPLICIT_NUMERICAL_CONSTRAINT_HH
