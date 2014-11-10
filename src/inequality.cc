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

#include "eigen3/Eigen/Core"
#include "hpp/core/inequality.hh"

namespace hpp {
  namespace core {
    EqualityPtr_t Equality::unique_ = Equality::create ();

    // -------------------------------------------------------------------

    template <>
    Inequality < EquationType::Superior >::Inequality (const value_type& thr):
      threshold_ (thr)
    {}

    template <>
    EquationTypePtr_t Inequality < EquationType::Superior >::create (const value_type& thr)
    {
      return SuperiorPtr_t (new Inequality < EquationType::Superior > (thr));
    }

    template <>
    void Inequality< EquationType::Superior >::threshold (const value_type& t)
    {
      assert (t >= 0);
      threshold_ = t;
    }

    template <>
    bool Inequality< EquationType::Superior >::operator () (vectorOut_t value) const
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
    bool Inequality< EquationType::Superior >::operator () (vectorOut_t value, matrixOut_t jacobian) const
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
    Inequality < EquationType::Inferior >::Inequality (const value_type& thr):
      threshold_ (-thr)
    {}

    template <>
    EquationTypePtr_t Inequality < EquationType::Inferior >::create (const value_type& thr)
    {
      return InferiorPtr_t (new Inequality < EquationType::Inferior > (thr));
    }

    template <>
    void Inequality< EquationType::Inferior >::threshold (const value_type& t)
    {
      assert (t >= 0);
      threshold_ = -t;
    }

    template <>
    bool Inequality< EquationType::Inferior >::operator () (vectorOut_t value) const
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
    bool Inequality< EquationType::Inferior >::operator () (vectorOut_t value, matrixOut_t jacobian) const
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

    EquationTypePtr_t DoubleInequality::create (const value_type w, const value_type& thr)
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

    bool EquationTypes::operator () (vectorOut_t value, matrixOut_t jacobian) const
    {
      bool val = true;
      for (size_t i = 0; i < inequalities_.size (); i++)
        val = (*inequalities_[i])(value.segment (i, 1), jacobian.block (i, 0, 1, jacobian.cols())) && val;
      return val;
    }

    bool EquationTypes::operator () (vectorOut_t value) const
    {
      bool val = true;
      for (size_t i = 0; i < inequalities_.size (); i++)
        val = (*inequalities_[i])(value.segment (i, 1)) && val;
      return val;
    }

    EquationTypes::EquationTypes (const std::vector <EquationType::Type> types)
    {
      for (size_t i = 0; i < types.size (); i++) {
        switch (types[i]) {
          case EquationType::Equality:
            inequalities_.push_back (Equality::create ());
            break;
          case EquationType::Superior:
            inequalities_.push_back (Inequality <EquationType::Superior>::create ());
            break;
          case EquationType::Inferior:
            inequalities_.push_back (Inequality <EquationType::Inferior>::create ());
            break;
          default:
            throw std::logic_error ("EquationType::Type not known.");
        }
      }
    }

    EquationTypesPtr_t EquationTypes::create (size_t dim)
    {
      return create (std::vector <EquationType::Type> (dim, EquationType::Superior));
    }

    EquationTypesPtr_t EquationTypes::create (const std::vector <EquationType::Type> types)
    {
      EquationTypesPtr_t shPtr (new EquationTypes (types));
      return shPtr;
    }

    template class Inequality < EquationType::Superior >;
    template class Inequality < EquationType::Inferior >;
  } // namespace core
} // namespace hpp
