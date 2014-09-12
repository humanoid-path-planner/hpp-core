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
    class HPP_CORE_DLLAPI Inequality
    {
      public:
        /// Return the result of the comparison.
        /// \param[in,out] value the value that will be compared and saturated.
        /// \param[in,out] jacobian the jacobian to be saturated depending on the value.
        /// \return true is the constraint is - at least partially - active
        virtual bool operator () (vector_t& value, matrix_t& jacobian) const = 0;
    };

    /// Implement the comparison for equality constraint.
    class HPP_CORE_DLLAPI Equality : public Inequality
    {
      public:
        /// \return Always true.
        inline bool operator () (vector_t&, matrix_t&) const
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
    class HPP_CORE_DLLAPI InequalityVector : public Inequality
    {
      public:
        InequalityVector (size_type dim) :
          invert_ (vector_t::Ones (dim))
        {}

        InequalityVector (const vector_t& invert) :
          invert_ (invert)
        {}

        virtual bool operator () (vector_t& value, matrix_t& jacobian) const
        {
          bool isActive = true;
          for (size_type i = 0; i < invert_.size (); i++)
            if (invert_[i] * value[i]> 0) {
              value[i] = 0;
              jacobian.row (i).setZero ();
              isActive = false;
            }
          return isActive;
        }

      private:
        /// A vector of +/- 1 depending on whether the direction
        /// is inverted along the corresponding axis.
        vector_t invert_;
    };
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_INEQUALITY_HH
