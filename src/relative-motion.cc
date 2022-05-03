// Copyright (c) 2016, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#include <hpp/core/relative-motion.hh>

#include <pinocchio/multibody/model.hpp>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/joint-collection.hh>

#include <hpp/constraints/generic-transformation.hh>
#include <hpp/constraints/explicit/relative-transformation.hh>

#include <hpp/core/constraint-set.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/constraints/implicit.hh>
#include <hpp/constraints/locked-joint.hh>

#include <hpp/constraints/explicit/implicit-function.hh>

namespace hpp {
  namespace core {
    namespace {
      inline void symSet (RelativeMotion::matrix_type& m, size_type i0, size_type i1, RelativeMotion::RelativeMotionType t)
      {
        m(i0,i1) = m(i1,i0) = t;
      }

      /// Considering the order Constrained(0) < Parameterized(1) < Unconstrained(2)
      /// Change m(i0,i1) only if it decreases.
      inline void symSetNoDowngrade (RelativeMotion::matrix_type& m, size_type i0, size_type i1, RelativeMotion::RelativeMotionType t)
      {
        assert (RelativeMotion::Constrained < RelativeMotion::Parameterized
            && RelativeMotion::Parameterized < RelativeMotion::Unconstrained);
        assert (m(i0,i1) == m(i1,i0));
        if (t < m(i0,i1)) m(i0,i1) = m(i1,i0) = t;
      }
    }

    RelativeMotion::matrix_type RelativeMotion::matrix (const DevicePtr_t& dev)
    {
      assert (dev);
      const size_type N = dev->model().joints.size();
      matrix_type matrix (N, N);
      matrix.setConstant (Unconstrained);
      matrix.diagonal().setConstant(Constrained);
      return matrix;
    }

    void RelativeMotion::fromConstraint (matrix_type& matrix,
        const DevicePtr_t& robot,
        const ConstraintSetPtr_t& c)
    {
      assert (robot);
      assert (c);

      const size_type N = robot->model().joints.size();
      if (matrix.rows() != N || matrix.cols() != N)
        throw std::invalid_argument ("Wrong RelativeMotion::matrix_type size");

      ConfigProjectorPtr_t proj = c->configProjector();
      if (!proj) return;

      // Loop over the constraints
      const NumericalConstraints_t& ncs = proj->numericalConstraints ();
      for (NumericalConstraints_t::const_iterator _ncs = ncs.begin();
          _ncs != ncs.end(); ++_ncs) {
        constraints::ImplicitConstPtr_t nc = *_ncs;
        size_type i1, i2;
        std::pair <JointConstPtr_t, JointConstPtr_t> joints =
          nc->doesConstrainRelPoseBetween(robot);
        i1 = Joint::index(joints.first);
        i2 = Joint::index(joints.second);
        if (i1 == i2) {
          hppDout (info, "Constraint " << nc->function().name ()
                  << " does not fully constrain any pair of joints.");
          continue;
        }
        bool cstRHS (nc->parameterSize () == 0);
        recurseSetRelMotion (matrix, i1, i2, (cstRHS ? Constrained : Parameterized));
      }
    }

    void RelativeMotion::recurseSetRelMotion(matrix_type& matrix,
        const size_type& i1, const size_type& i2,
        const RelativeMotion::RelativeMotionType& type)
    {
      assert (Constrained < Parameterized && Parameterized < Unconstrained);
      bool param = (type == Parameterized);
      RelativeMotionType t = Unconstrained;
      // Constrained(0) < Parameterized(1) < Unconstrained(2)
      // If the current value is more constraining, then do not change it.
      if (matrix(i1,i2) <= type) return;
      symSet(matrix, i1, i2, type);

      // i1 to i3
      for (size_type i3 = 0; i3 < matrix.rows(); ++i3) {
        if (i3 == i1) continue;
        if (i3 == i2) continue;
        if (matrix(i2,i3) != Unconstrained) {
          t = (!param && matrix(i2,i3) == Constrained) ? Constrained : Parameterized;
          symSetNoDowngrade(matrix, i1, i3, t);
        }
      }
      for (size_type i0 = 0; i0 < matrix.rows(); ++i0) {
        if (i0 == i2) continue;
        if (i0 == i1) continue;
        // i0 to i2
        if (matrix(i0,i1) != Unconstrained) {
          t = (!param && matrix(i0,i1) == Constrained) ? Constrained : Parameterized;
          symSetNoDowngrade (matrix, i0, i2 ,t);
        }

        // from i0 to i3
        if (matrix(i0,i1) == Unconstrained) continue;
        for (size_type i3 = 0; i3 < matrix.rows(); ++i3) {
          if (i3 == i2) continue;
          if (i3 == i1) continue;
          if (i3 == i0) continue;
          if (matrix(i2,i3) == Unconstrained) continue;
          // If motion is already constrained, continue.
          t = (!param && matrix(i0,i1) == Constrained && matrix(i2,i3) == Constrained)
            ? Constrained : Parameterized;
          symSetNoDowngrade (matrix, i0, i3, t);
        }
      }
    }
  } // namespace core
} // namespace hpp
