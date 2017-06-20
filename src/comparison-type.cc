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

#include <stdexcept>
#include "eigen3/Eigen/Core"
#include <hpp/core/comparison-type.hh>

namespace hpp {
  namespace core {
    EqualityPtr_t Equality::unique_ = Equality::create ();
    EqualToZeroPtr_t EqualToZero::unique_ = EqualToZero::create ();

    ComparisonTypePtr_t ComparisonType::createDefault ()
    {
      return EqualToZero::unique_;
    }

    // -------------------------------------------------------------------

    template <>
    Inequality < ComparisonType::Superior >::Inequality (const value_type& thr):
      threshold_ (thr)
    {}

    template <>
    ComparisonTypePtr_t Inequality < ComparisonType::Superior >::create (const value_type& thr)
    {
      return SuperiorPtr_t (new Inequality < ComparisonType::Superior > (thr));
    }

    template <>
    void Inequality< ComparisonType::Superior >::threshold (const value_type& t)
    {
      assert (t >= 0);
      threshold_ = t;
    }

    template <>
    bool Inequality< ComparisonType::Superior >::operator () (vectorOut_t value) const
    {
      if (value[0] <= 0) {
        value[0] = value[0] - threshold_;
        return true;
      } else if (value[0] < threshold_) {
        value[0] -= threshold_;
      } else {
        value[0] = 0;
      }
      return false;
    }

    template <>
    bool Inequality< ComparisonType::Superior >::operator () (vectorOut_t value, matrixOut_t jacobian) const
    {
      if (value[0] <= 0) {
        value[0] = value[0] - threshold_;
        return true;
      } else if (value[0] < threshold_) {
        value[0] -= threshold_;
      } else {
        value[0] = 0;
        jacobian.row (0).setZero ();
      }
      return false;
    }

    // -------------------------------------------------------------------

    template <>
    Inequality < ComparisonType::Inferior >::Inequality (const value_type& thr):
      threshold_ (-thr)
    {}

    template <>
    ComparisonTypePtr_t Inequality < ComparisonType::Inferior >::create (const value_type& thr)
    {
      return InferiorPtr_t (new Inequality < ComparisonType::Inferior > (thr));
    }

    template <>
    void Inequality< ComparisonType::Inferior >::threshold (const value_type& t)
    {
      assert (t >= 0);
      threshold_ = -t;
    }

    template <>
    bool Inequality< ComparisonType::Inferior >::operator () (vectorOut_t value) const
    {
      if (value[0] >= 0) {
        value[0] = value[0] - threshold_;
        return true;
      } else if (value[0] > threshold_) { // threshold_ < 0
        value[0] -= threshold_;
      } else {
        value[0] = 0;
      }
      return false;
    }

    template <>
    bool Inequality< ComparisonType::Inferior >::operator () (vectorOut_t value, matrixOut_t jacobian) const
    {
      if (value[0] >= 0) {
        value[0] = value[0] - threshold_;
        return true;
      } else if (value[0] > threshold_) {
        value[0] -= threshold_;
      } else {
        value[0] = 0;
        jacobian.row (0).setZero ();
      }
      return false;
    }

    // -------------------------------------------------------------------

    DoubleInequality::DoubleInequality (const value_type w, const value_type& t):
      threshold_ (t), left_ (-w/2), right_ (w/2)
    {
      assert (w > 0);
    }

    ComparisonTypePtr_t DoubleInequality::create (const value_type w, const value_type& thr)
    {
      return DoubleInequalityPtr_t (new DoubleInequality (w, thr));
    }

    void DoubleInequality::threshold (const value_type& t)
    {
      threshold_ = t;
    }

    bool DoubleInequality::operator () (vectorOut_t value) const
    {
      if (value[0] <= left_) {
        value[0] = value[0] - threshold_;
        return true;
      } else if (value[0] >= right_) {
        value[0] = value[0] + threshold_;
        return true;
      } else if (value[0] < left_ - threshold_) {
        value[0] = value[0] - threshold_;
      } else if (value[0] > right_ - threshold_) {
        value[0] = value[0] + threshold_;
      } else {
        value[0] = 0;
      }
      return false;
    }

    bool DoubleInequality::operator () (vectorOut_t value, matrixOut_t jacobian) const
    {
      if (value[0] <= left_) {
        value[0] = value[0] - threshold_;
        return true;
      } else if (value[0] >= right_) {
        value[0] = value[0] + threshold_;
        return true;
      } else if (value[0] < left_ - threshold_) {
        value[0] = value[0] - threshold_;
      } else if (value[0] > right_ - threshold_) {
        value[0] = value[0] + threshold_;
      } else {
        value[0] = 0;
        jacobian.row (0).setZero ();
      }
      return false;
    }

    // -------------------------------------------------------------------

    bool ComparisonTypes::operator () (vectorOut_t value, matrixOut_t jacobian) const
    {
      bool val = true;
      for (size_t i = 0; i < inequalities_.size (); i++)
        val = (*inequalities_[i])(value.segment (i, 1), jacobian.block (i, 0, 1, jacobian.cols())) && val;
      return val;
    }

    bool ComparisonTypes::operator () (vectorOut_t value) const
    {
      bool val = true;
      for (size_t i = 0; i < inequalities_.size (); i++)
        val = (*inequalities_[i])(value.segment (i, 1)) && val;
      return val;
    }

    ComparisonTypes::ComparisonTypes (const std::vector <ComparisonType::Type> types)
    {
      for (size_t i = 0; i < types.size (); i++) {
        switch (types[i]) {
          case ComparisonType::Equality:
            inequalities_.push_back (Equality::create ());
            break;
          case ComparisonType::Superior:
            inequalities_.push_back (Inequality <ComparisonType::Superior>::create ());
            break;
          case ComparisonType::Inferior:
            inequalities_.push_back (Inequality <ComparisonType::Inferior>::create ());
            break;
          case ComparisonType::EqualToZero:
            inequalities_.push_back (EqualToZero::create ());
            break;
          default:
            throw std::logic_error ("ComparisonType::Type not known.");
        }
      }
    }

    ComparisonTypesPtr_t ComparisonTypes::create (size_t dim)
    {
      return create (std::vector <ComparisonType::Type> (dim, ComparisonType::Default));
    }

    ComparisonTypesPtr_t ComparisonTypes::create (const std::vector <ComparisonType::Type> types)
    {
      ComparisonTypesPtr_t shPtr (new ComparisonTypes (types));
      return shPtr;
    }

    template class Inequality < ComparisonType::Superior >;
    template class Inequality < ComparisonType::Inferior >;
  } // namespace core
} // namespace hpp
