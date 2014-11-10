// Copyright (c) 2014, LAAS-CNRS
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

#ifndef HPP_CORE_INEQUALITY_HH
# define HPP_CORE_INEQUALITY_HH

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>

namespace hpp {
  namespace core {
    /// Abstract class defining the comparison for a function value
    /// and doing a saturation, necessary for inequality constraint.
    /// used to compare two vector.
    /// They are used in ConfigProjector to implement inequality
    /// constraint.
    class HPP_CORE_DLLAPI EquationType
    {
      public:
        enum Type {
          Equality,
          Superior,
          Inferior,
          DoubleInequality
        };
        typedef std::vector <Type> VectorOfTypes;

        /// Return the result of the comparison.
        /// \param[in,out] value the value that will be compared and saturated.
        /// \param[in,out] jacobian the jacobian to be saturated depending on the value.
        /// \return true is the constraint is - at least partially - active
        virtual bool operator () (vectorOut_t value, matrixOut_t jacobian) const = 0;

        /// Return the result of the comparison.
        /// \param[in,out] value the value that will be compared and saturated.
        /// \return true is the constraint is - at least partially - active
        virtual bool operator () (vectorOut_t value) const = 0;
    };

    /// Implement the comparison for equality constraint.
    class HPP_CORE_DLLAPI Equality : public EquationType
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

        static EqualityPtr_t unique_;
    };

    /// Implementation of Inequality that compare the value to
    /// a reference with the possibility of inverting axis.
    class HPP_CORE_DLLAPI EquationTypes : public EquationType
    {
      public:
        virtual bool operator () (vectorOut_t value, matrixOut_t jacobian) const;

        virtual bool operator () (vectorOut_t value) const;

        static EquationTypesPtr_t create (size_t dim);

        static EquationTypesPtr_t create (const std::vector <EquationType::Type> types);

      protected:
        EquationTypes (const std::vector <EquationType::Type> types);

      private:
        typedef std::vector <EquationTypePtr_t> Container_t;
        typedef Container_t::iterator iterator;
        typedef Container_t::const_iterator const_iterator;

        Container_t inequalities_;
    };

    template < EquationType::Type T > class Inequality;
    typedef Inequality < EquationType::Superior > Superior;
    typedef Inequality < EquationType::Inferior > Inferior;
    typedef boost::shared_ptr < Superior > SuperiorPtr_t;
    typedef boost::shared_ptr < Inferior > InferiorPtr_t;

    template < EquationType::Type T >
    class HPP_CORE_DLLAPI Inequality : public EquationType
    {
      public:
        virtual bool operator () (vectorOut_t value, matrixOut_t jacobian) const;

        virtual bool operator () (vectorOut_t value) const;

        void threshold (const value_type& t);

        static EquationTypePtr_t create (const value_type& threshold = 1e-3);

      protected:
        Inequality (const value_type& threshold);

      private:
        value_type threshold_;
    };

    class HPP_CORE_DLLAPI DoubleInequality : public EquationType
    {
      public:
        virtual bool operator () (vectorOut_t value, matrixOut_t jacobian) const;

        virtual bool operator () (vectorOut_t value) const;

        void threshold (const value_type& t);

        static EquationTypePtr_t create (const value_type width, const value_type& threshold = 1e-3);

      protected:
        DoubleInequality (const value_type width, const value_type& threshold);

      private:
        value_type threshold_;
        value_type left_, right_;
    };
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_INEQUALITY_HH
