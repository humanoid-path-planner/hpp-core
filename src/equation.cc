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

#include "hpp/core/equation.hh"

namespace hpp {
  namespace core {
    void Equation::rightHandSide (vectorIn_t rhs)
    {
      rhs_ = rhs;
    }

    vectorIn_t Equation::rightHandSide () const
    {
      return rhs_;
    }

    vectorOut_t Equation::nonConstRightHandSide ()
    {
      return rhs_;
    }

    size_type Equation::rhsSize () const
    {
      return rhs_.size ();
    }

    const ComparisonTypePtr_t& Equation::comparisonType () const
    {
      return comparison_;
    }

    void Equation::comparisonType (const ComparisonTypePtr_t& comp)
    {
      comparison_ = comp;
      if (comparison_->constantRightHandSide ())
        rhs_ = vector_t ();
      else
        rhs_ = vector_t::Zero (rhsRealSize_);
    }

    Equation::Equation (const ComparisonTypePtr_t& comp, vectorIn_t rhs) :
      comparison_ (comp), rhs_ (rhs), rhsRealSize_ (rhs.size())
    {
      if (comparison_->constantRightHandSide ())
        rhs_ = vector_t ();
    }

    Equation::Equation (const Equation& other) :
      comparison_ (other.comparison_), rhs_ (other.rhs_)
    {
    }

    bool Equation::isEqual (const Equation& other, bool swapAndTest) const
    {
      if (comparison_ != other.comparison_) return false;
      if (rhs_ != other.rhs_) return false;
      if (swapAndTest)
	return other.isEqual (*this, false);
      return true;
    }

  } // namespace core
} // namespace hpp
