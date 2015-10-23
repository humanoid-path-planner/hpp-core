// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr),
//          Florent Lamiraux
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

#ifndef HPP_CORE_COMPARISON_TYPE_HH
# define HPP_CORE_COMPARISON_TYPE_HH

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>

namespace hpp {
  namespace core {
    /// \addtogroup constraints
    /// \{

    /// Abstract class defining the comparison for a function value
    /// and doing a saturation, necessary for inequality constraint.
    /// used to compare two vector.
    /// They are used in ConfigProjector to implement inequality
    /// constraint.
    class HPP_CORE_DLLAPI ComparisonType
    {
      public:
        enum Type {
          Equality,
          EqualToZero,
          Superior,
          Inferior,
          DoubleInequality
        };
        typedef std::vector <Type> VectorOfTypes;

        /// Return the result of the comparison.
        /// \param[in,out] value the value that will be compared and saturated.
        /// \param[in,out] jacobian the jacobian to be saturated depending on the value.
        /// \return true is the constraint is - at least partially - active
        virtual bool operator () (vectorOut_t value, matrixOut_t jacobian)
	  const = 0;

        /// Return the result of the comparison.
        /// \param[in,out] value the value that will be compared and saturated.
        /// \return true is the constraint is - at least partially - active
        virtual bool operator () (vectorOut_t value) const = 0;
	/// Return whether the right hand side of the comparison is constant
	virtual bool constantRightHandSide () const
	{
	  return true;
	}
	static ComparisonTypePtr_t createDefault ();
    }; // class ComparisonType

    /// Implementation of equality.
    ///
    /// \f{eqnarray*}
    /// f (\mathbf{q}) = f^0 \in \mathbb{R}^m
    /// \f}
    class HPP_CORE_DLLAPI Equality : public ComparisonType
    {
      public:
        /// \return Always true.
        inline bool operator () (vectorOut_t, matrixOut_t) const
        {
          return true;
        }

        /// \return Always true.
        inline bool operator () (vectorOut_t) const
        {
          return true;
        }

        static EqualityPtr_t create ()
        {
          if (!unique_)
            unique_ = EqualityPtr_t (new Equality());
          return unique_;
        }
	virtual bool constantRightHandSide () const
	{
	  return false;
	}

        static EqualityPtr_t unique_;
    }; // class Equality

    /// Implementation of equality to zero.
    ///
    /// \f{eqnarray*}
    /// f (\mathbf{q}) = 0 \in \mathbb{R}^m
    /// \f}
    ///
    /// right hand side of this constraint cannot be modified.
    class HPP_CORE_DLLAPI EqualToZero : public ComparisonType
    {
      public:
        /// \return Always true.
        inline bool operator () (vectorOut_t, matrixOut_t) const
        {
          return true;
        }

        /// \return Always true.
        inline bool operator () (vectorOut_t) const
        {
          return true;
        }

        static EqualToZeroPtr_t create ()
        {
          if (!unique_)
            unique_ = EqualToZeroPtr_t (new EqualToZero());
          return unique_;
        }

        static EqualToZeroPtr_t unique_;
    }; // class EqualToZero

    /// Implementation of various equation types
    ///
    /// This class enable users to define different types of comparison
    /// for each component of the function. Comparator for each component
    /// is defined by input parameter of ComparisonTypes::create.
    ///
    /// \note DoubleInequality cannot be used by this class.
    class HPP_CORE_DLLAPI ComparisonTypes : public ComparisonType
    {
      public:
        virtual bool operator () (vectorOut_t value, matrixOut_t jacobian) const;

        virtual bool operator () (vectorOut_t value) const;

	/// Create a vector of comparisons
	///
	/// \param dim size of the function output
        static ComparisonTypesPtr_t create (size_t dim);

	/// Create a vector of comparisons
	///
	/// \param types vector of comparisons
        static ComparisonTypesPtr_t create
	  (const std::vector <ComparisonType::Type> types);

        ComparisonTypePtr_t& at (const std::size_t index)
        {
          assert (index < inequalities_.size());
          return inequalities_[index];
        }

      protected:
        ComparisonTypes (const std::vector <ComparisonType::Type> types);

      private:
        typedef std::vector <ComparisonTypePtr_t> Container_t;
        typedef Container_t::iterator iterator;
        typedef Container_t::const_iterator const_iterator;

        Container_t inequalities_;
    }; // class ComparisonType

    template < ComparisonType::Type T > class Inequality;
    typedef Inequality < ComparisonType::Superior > SuperiorIneq;
    typedef Inequality < ComparisonType::Inferior > InferiorIneq;
    typedef boost::shared_ptr < SuperiorIneq > SuperiorPtr_t;
    typedef boost::shared_ptr < InferiorIneq > InferiorPtr_t;

    /// Implementation of inequality
    ///
    /// \f{eqnarray*}
    /// f (\mathbf{q}) \leq 0 \in \mathbb{R}^m
    /// \f}
    /// if T is ComparisonType::Inferior
    ///
    /// \f{eqnarray*}
    /// f (\mathbf{q}) \geq 0 \in \mathbb{R}^m
    /// \f}
    /// if T is ComparisonType::Superior
    ///
    /// \sa SuperiorIneq InferiorIneq
    template < ComparisonType::Type T >
    class HPP_CORE_DLLAPI Inequality : public ComparisonType
    {
      public:
        virtual bool operator () (vectorOut_t value, matrixOut_t jacobian) const;

        virtual bool operator () (vectorOut_t value) const;

        void threshold (const value_type& t);

        static ComparisonTypePtr_t create (const value_type& threshold = 1e-3);

      protected:
        Inequality (const value_type& threshold);

      private:
        value_type threshold_;
    }; // class Inequality

    /// Implementation of double inequality
    ///
    /// \f{eqnarray*}
    /// -f^0/2 \leq f (\mathbf{q}) \leq f^0/2 \in \mathbb{R}^m
    /// \f}
    /// \f$f^0\f$ is set at construction by parameter width of
    /// DoubleInequality::create.
    class HPP_CORE_DLLAPI DoubleInequality : public ComparisonType
    {
      public:
        virtual bool operator () (vectorOut_t value, matrixOut_t jacobian) const;

        virtual bool operator () (vectorOut_t value) const;

        void threshold (const value_type& t);

        static ComparisonTypePtr_t create (const value_type width, const value_type& threshold = 1e-3);

      protected:
        DoubleInequality (const value_type width, const value_type& threshold);

      private:
        value_type threshold_;
        value_type left_, right_;
    };// class DoubleInequality
    /// \}
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_COMPARISON_TYPE_HH
