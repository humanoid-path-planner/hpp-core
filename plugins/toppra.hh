// Copyright (c) 2020, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//          Olivier Roussel (olivier.roussel@laas.fr)
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

#ifndef HPP_CORE_PATH_OPTIMIZATION_TOPPRA_HH
#define HPP_CORE_PATH_OPTIMIZATION_TOPPRA_HH

#include <hpp/core/path-optimizer.hh>

namespace hpp {
namespace core {
namespace pathOptimization {

class TOPPRA;
typedef boost::shared_ptr<TOPPRA> TOPPRAPtr_t;

class TOPPRA : public PathOptimizer
{
  public:
    static TOPPRAPtr_t create(const Problem &p)
    {
      return TOPPRAPtr_t(new TOPPRA(p));
    }

    PathVectorPtr_t optimize(const PathVectorPtr_t &path);

protected:
    using PathOptimizer::PathOptimizer;
}; // class TOPPRA

} // namespace pathOptimization
} // namespace core
} // namespace hpp

#endif // HPP_CORE_PATH_OPTIMIZATION_TOPPRA_HH
