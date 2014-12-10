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

#define BOOST_TEST_MODULE DifferentiableFunction
#include <boost/test/included/unit_test.hpp>

#include <hpp/constraints/differentiable-function.hh>
#include <hpp/core/fwd.hh>

namespace hpp {
  namespace core {
    class DiffFuncTest : public DifferentiableFunction
    {
      public:
        DiffFuncTest (size_type inputSize,
            size_type inputDerivativeSize,
            size_type outputSize,
            std::string name = std::string ()) :
          DifferentiableFunction (inputSize, inputDerivativeSize, outputSize, name)
      {}

      protected:
        void impl_compute (vectorOut_t /* result */,
            ConfigurationIn_t /* argument */) const
        {}

        void impl_jacobian (matrixOut_t jacobian,
            ConfigurationIn_t /* arg */) const
        {
          jacobian.setOnes ();
        }
    };
  }
}

typedef hpp::core::DifferentiableFunction::Intervals_t Intervals_t;
typedef hpp::core::DifferentiableFunction::Interval_t  Interval_t;
using hpp::core::DiffFuncTest;
using hpp::core::size_type;

template < class T >
inline std::ostream& operator << (std::ostream& os, const std::vector<T>& v) 
{
    os << "[";
    typedef typename std::vector<T>::const_iterator const_iterator;
    for (const_iterator ii = v.begin(); ii != v.end(); ++ii)
    {
        os << " " << *ii;
    }
    os << " ]";
    return os;
}

BOOST_AUTO_TEST_CASE (DifferentiableFunction)
{
  DiffFuncTest df (9, 8, 1, "test");
  Intervals_t dofsIntervals;
  dofsIntervals.push_back (Interval_t (0, 1));
  dofsIntervals.push_back (Interval_t (2, 3));
  dofsIntervals.push_back (Interval_t (7, 1));
  std::vector <size_type> dofs;
  for (size_t i = 0; i < dofsIntervals.size (); i++)
    for (size_type j = 0; j < dofsIntervals[i].second; j++)
      dofs.push_back (dofsIntervals [i].first + j);
  Intervals_t ret = df.passiveDofs (dofs);
  BOOST_CHECK_MESSAGE (ret.size () == dofsIntervals.size (),
      ret.size () << " intervals found instead of " << dofsIntervals.size ());
  for (size_t i = 0; i < dofsIntervals.size (); i++) {
    BOOST_CHECK_MESSAGE (ret[i].first == dofsIntervals[i].first && ret[i].second == dofsIntervals[i].second,
        "Interval " << i << " is " << ret[i].first << ", " << ret[i].second
        << " and should be " << dofsIntervals[i].first << ", " << dofsIntervals[i].second);
  }

  std::vector <size_type> dofs_shuffle (dofs);
  dofs_shuffle.push_back (dofsIntervals[0].first);
  std::random_shuffle (dofs_shuffle.begin (), dofs_shuffle.end());
  std::cout << dofs_shuffle << std::endl;
  ret = df.passiveDofs (dofs_shuffle);
  BOOST_CHECK_MESSAGE (ret.size () == dofsIntervals.size (),
      "After random_shuffle: " << ret.size () << " intervals found instead of " << dofsIntervals.size ());
  for (size_t i = 0; i < dofsIntervals.size (); i++) {
    BOOST_CHECK_MESSAGE (ret[i].first == dofsIntervals[i].first && ret[i].second == dofsIntervals[i].second,
        "After random_shuffle: Interval " << i << " is " << ret[i].first << ", " << ret[i].second
        << " and should be " << dofsIntervals[i].first << ", " << dofsIntervals[i].second);
  }

  hpp::core::matrix_t jacobian (df.outputSize (), df.inputDerivativeSize ()); jacobian.setZero ();
  hpp::core::vector_t arg (df.inputSize ()); arg.setZero ();
  df.jacobian (jacobian, arg);
  for (size_t i = 0; i < dofs.size (); i++)
    BOOST_CHECK_MESSAGE (jacobian.col (dofs[i]).isZero (),
        "Column " << dofs[i] << " of jacobian is not zero");
}
