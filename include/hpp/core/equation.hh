// Copyright (c) 2015, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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


#ifndef HPP_CORE_EQUATION_HH
# define HPP_CORE_EQUATION_HH

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>
# include <hpp/core/comparison-type.hh>

namespace hpp {
  namespace core {
    /// \addtogroup constraints
    /// \{

    /// This class represents an equation with the following format
    /// \f$ f(q) = \textbf{or} \le rhs \f$.
    class HPP_CORE_DLLAPI Equation {
      public:
        /// Copy object and return shared pointer to copy
        virtual EquationPtr_t copy () const = 0;
	/// Operator equality
	bool operator== (const Equation& other) const
	{
	  return isEqual (other, true);
	}
        /// Set the right hand side from a configuration
        ///
        /// in such a way that the configuration satisfies the numerical
        /// constraints
        /// \param config the input configuration.
        ///
        /// \warning Only values of the right hand side corresponding to 
        /// \link Equality "equality constraints" \endlink are set. As a
        /// result, the input configuration may not satisfy the other constraints.
        /// The rationale is the following. Equality constraints define a
        /// foliation of the configuration space. Leaves of the foliation are
        /// defined by the value of the right hand side of the equality
        /// constraints. This method is mainly used in manipulation planning
        /// to retrieve the leaf a configuration lies on.
        virtual void rightHandSideFromConfig (ConfigurationIn_t config) = 0;

        /// Set the right hand side of the equation.
        /// \param rhs the right hand side.
        void rightHandSide (vectorIn_t rhs);

        /// Return the right hand side of the equation.
        vectorIn_t rightHandSide () const;

        /// Return the size of the right hand side constraint.
        size_type rhsSize () const;

        /// Return the ComparisonType
        const ComparisonTypePtr_t& comparisonType () const;

	/// Set the comparison type
	void comparisonType (const ComparisonTypePtr_t& comp);

        /// Return the right hand side of the equation.
        vectorOut_t nonConstRightHandSide ();

      protected:
        Equation (const ComparisonTypePtr_t& comp, vectorIn_t rhs);
	//Copy constructor
	Equation (const Equation& other);
	/// Test equality with other instance
	/// \param other object to copy
	/// \param swapAndTest whether we should also check other == this
	virtual bool isEqual (const Equation& other, bool swapAndTest) const;
	// Store weak pointer to itself
	void init (const EquationWkPtr_t& weak)
	{
	  weak_ = weak;
	}
      private:
        ComparisonTypePtr_t comparison_;
        vector_t rhs_;
        size_type rhsRealSize_;
	EquationWkPtr_t weak_;
    };
    /// \}
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_EQUATION_HH
