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

# include <Eigen/SVD>

# include <hpp/statistics/success-bin.hh>

# include <hpp/core/config.hh>
# include <hpp/core/constraint.hh>
# include <hpp/core/deprecated.hh>

namespace hpp {
  namespace core {
    typedef constraints::solver::BySubstitution BySubstitution;

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
     <b>right hand side</b>.

     The constraints are solved numerically by a Newton Raphson like method.

     Numerical constraints can be added using method
     ConfigProjector::add. Default parameter of this method define
     equality constraints, but inequality constraints can also be defined by
     passing an object of type ComparisonType to method.
    */
    class HPP_CORE_DLLAPI ConfigProjector : public Constraint
    {
    public:
      enum LineSearchType {
        Backtracking,
        ErrorNormBased,
        FixedSequence,
        Constant
      };

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

      /// Return shared pointer to copy
      /// \param cp shared pointer to object to copy
      static ConfigProjectorPtr_t createCopy (const ConfigProjectorPtr_t cp);

      /// return shared pointer to copy
      virtual ConstraintPtr_t copy () const;

      /// Destructor
      virtual ~ConfigProjector ();

      /// Check that numerical constraint is in config projector
      /// \param numericalConstraint numerical constraint
      /// \return true if numerical constraint is already in config projector
      ///         whatever the passive dofs are.
      bool contains (const constraints::ImplicitPtr_t& numericalConstraint)
        const;

      /// Add a numerical constraint
      /// \param numericalConstraint The numerical constraint.
      /// \param passiveDofs column indices of the jacobian vector that will be
      ///        set to zero when solving.
      /// \param priority priority of the function. The last level might be
      ///        optional.
      /// \return false if numerical constraint had already been inserted.
      ///
      /// \note The intervals are interpreted as a list of couple
      /// (index_start, length) and NOT as (index_start, index_end).
      bool add (const constraints::ImplicitPtr_t& numericalConstraint,
		const segments_t& passiveDofs = segments_t (0),
		const std::size_t priority = 0);

      void lastIsOptional (bool optional);

      bool lastIsOptional () const;

      /// Optimize the configuration while respecting the constraints
      /// The input configuration must already satisfy the constraints.
      /// \return true if the configuration was optimized.
      /// \param maxIter if 0, use maxIterations().
      bool optimize (ConfigurationOut_t config,
          std::size_t maxIter = 0);

      /// Get robot
      const DevicePtr_t& robot () const
      {
	return robot_;
      }

      /// Project velocity on constraint tangent space in "from"
      ///
      /// \param from configuration,
      /// \param velocity velocity to project
      ///
      /// \f[
      /// \textbf{q}_{res} = \left(I_n -
      /// J^{+}J(\textbf{q}_{from})\right) (\textbf{v})
      /// \f]
      void projectVectorOnKernel (ConfigurationIn_t from,
			    vectorIn_t velocity, vectorOut_t result);

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

      /// Compute value and reduced jacobian at a given configuration
      ///
      /// \param configuration input configuration
      /// \retval value values of the differentiable functions stacked in a
      ///         vector,
      /// \retval reducedJacobian Reduced Jacobian of the differentiable
      ///         functions stacked in a matrix. Reduced Jacobian is defined
      ///         as the Jacobian to which columns corresponding to explicit
      ///         constraints have been removed and to which columns corresponding
      ///         to passive dofs are set to 0.
      void computeValueAndJacobian (ConfigurationIn_t configuration,
				    vectorOut_t value,
				    matrixOut_t reducedJacobian);

      /// Execute one iteration of the projection algorithm
      /// \return true if the constraints are satisfied
      /// \deprecated use solver().oneStep is needed
      bool oneStep (ConfigurationOut_t config, vectorOut_t dq,
          const value_type& alpha) HPP_CORE_DEPRECATED;

      /// \name Compression of locked degrees of freedom
      ///
      /// Degrees of freedom related to locked joint are not taken into
      /// account in numerical constraint resolution. The following methods
      /// Compress or uncompress vectors or matrices by removing lines and
      /// columns corresponding to locked degrees of freedom.
      /// \{

      /// Return the number of free variables
      size_type numberFreeVariables () const;

      /// Get number of non-locked degrees of freedom
      /// \deprecated Call numberFreeVariables instead
      size_type numberNonLockedDof () const HPP_CORE_DEPRECATED;

      /// Get constraint dimension
      size_type dimension () const;

      /// Compress Velocity vector by removing output of explicit constraints
      ///
      /// \param normal input velocity vector
      /// \retval small compressed velocity vectors
      void compressVector (vectorIn_t normal, vectorOut_t small) const;

      /// Expand compressed velocity vector
      ///
      /// \param small compressed velocity vector without output of explicit
      ///              constraints
      /// \retval normal uncompressed velocity vector.
      void uncompressVector (vectorIn_t small, vectorOut_t normal) const;

      /// Compress matrix
      ///
      /// \param normal input matrix
      /// \retval small compressed matrix
      /// \param rows whether to compress rows and colums or only columns
      void compressMatrix (matrixIn_t normal, matrixOut_t small,
			   bool rows = true) const;

      /// Uncompress matrix
      ///
      /// \param small input matrix
      /// \retval normal uncompressed matrix
      /// \param rows whether to uncompress rows and colums or only columns
      void uncompressMatrix (matrixIn_t small, matrixOut_t normal,
			     bool rows = true) const;

      /// \}

      /// Set maximal number of iterations
      void maxIterations (size_type iterations);
      /// Get maximal number of iterations in config projector
      size_type maxIterations () const;

      /// Set error threshold
      void errorThreshold (const value_type& threshold);
      /// Get errorimal number of threshold in config projector
      value_type errorThreshold () const;

      value_type residualError() const;

      const value_type& sigma() const;

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

      /// Same as rightHandSideFromConfig(ConfigurationIn_t) but only for
      /// the specified constraints::Implicit
      void rightHandSideFromConfig (const constraints::ImplicitPtr_t& nm,
                                    ConfigurationIn_t config);

      /// Set the level set parameter.
      /// \param param the level set parameter.
      void rightHandSide (const vector_t& param);

      /// Same as rightHandSide(vectorIn_t) but only for
      /// the specified constraints::Implicit
      void rightHandSide (const constraints::ImplicitPtr_t& nm, vectorIn_t rhs);

      /// Get the level set parameter.
      /// \return the parameter.
      vector_t rightHandSide () const;

      /// Update the right hand side using Implicit::rightHandSideAt
      void rightHandSideAt (const value_type& s);

      /// @}

      /// Check whether a configuration statisfies the constraint.
      virtual bool isSatisfied (ConfigurationIn_t config);

      /// Check whether a configuration statisfies the constraint.
      ///
      /// \param config the configuration to check
      /// \retval error concatenation of differences between value of numerical constraints and
      ///         right hand side.
      virtual bool isSatisfied (ConfigurationIn_t config, vector_t& error);

      /// Get the statistics
      ::hpp::statistics::SuccessStatistics& statistics()
      {
        return statistics_;
      }

      /// Get the numerical constraints of the config-projector (and so of the
      /// Constraint Set)
      const NumericalConstraints_t& numericalConstraints () const;

      const BySubstitution& solver () const
      {
        return *solver_;
      }

      static void defaultLineSearch (LineSearchType ls);

    protected:
      /// Constructor
      /// \param robot robot the constraint applies to.
      /// \param errorThreshold norm of the value of the constraint under which
      ///        the constraint is considered satified,
      /// \param maxIterations maximal number of iteration in the resolution of
      ///                      the constraint.
      ConfigProjector (const DevicePtr_t& robot, const std::string& name,
		       value_type errorThreshold, size_type maxIterations);
      /// Copy constructor
      ConfigProjector (const ConfigProjector& cp);

      /// Store weak pointer to itself
      void init (const ConfigProjectorPtr_t& self)
      {
	Constraint::init (self);
	weak_ = self;
      }
      /// Numerically solve constraint
      virtual bool impl_compute (ConfigurationOut_t configuration);

    private:
      virtual std::ostream& print (std::ostream& os) const;
      virtual void addToConstraintSet (const ConstraintSetPtr_t& constraintSet);

      DevicePtr_t robot_;
      static LineSearchType defaultLineSearch_;
      LineSearchType lineSearchType_;
      BySubstitution* solver_;

      bool solverOneStep (ConfigurationOut_t config) const;
      int  solverSolve   (ConfigurationOut_t config) const;

      ConfigProjectorWkPtr_t weak_;
      ::hpp::statistics::SuccessStatistics statistics_;
    }; // class ConfigProjector
    /// \}
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_CONFIG_PROJECTOR_HH
