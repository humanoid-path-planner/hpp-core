// Copyright (c) 2016, Joseph Mirabel
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

#include <hpp/core/relative-motion.hh>
#include <hpp/constraints/explicit/relative-pose.hh>

#include <pinocchio/multibody/model.hpp>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>

#include <hpp/constraints/generic-transformation.hh>
#include <hpp/constraints/implicit/relative-pose.hh>
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

      template <typename T, typename Ptr_t = boost::shared_ptr<T> > struct check {
        static bool is (const DifferentiableFunctionPtr_t& f, size_type& i1, size_type& i2)
        {
          Ptr_t t = HPP_DYNAMIC_PTR_CAST(T, f);
          if (t) {
            i1 = RelativeMotion::idx(t->joint1());
            i2 = RelativeMotion::idx(t->joint2());
            return true;
          }
          return false;
        }
      };

      using constraints::explicit_::Function;
      template <bool GisIdentity> struct check <Function<GisIdentity> > {
        static bool is (const DifferentiableFunctionPtr_t& f, size_type& i1,
                        size_type& i2)
        {
          typename Function<GisIdentity>::Ptr_t
            implicit (HPP_DYNAMIC_PTR_CAST (Function<GisIdentity>, f));
          if (implicit)
            return check<ExplicitRelativeTransformation>::is (implicit->inputToOutput(), i1, i2);
          return false;
        }
      };
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
      using constraints::Transformation;
      using constraints::RelativeTransformation;
      assert (robot);
      assert (c);

      const size_type N = robot->model().joints.size();
      if (matrix.rows() != N || matrix.cols() != N)
        throw std::invalid_argument ("Wrong RelativeMotion::matrix_type size");

      ConfigProjectorPtr_t proj = c->configProjector();
      if (!proj) return;

      // Loop over the LockedJoint
      const pinocchio::Model& model = robot->model();
      const LockedJoints_t& lj = proj->lockedJoints ();
      for (LockedJoints_t::const_iterator it = lj.begin ();
          it != lj.end (); ++it) {
        const std::string& jointName = (*it)->jointName();
        if (!model.existJointName(jointName)) {
          // Extra dofs and partial locked joints have a name that won't be
          // recognized by Device::getJointByName. So they can be filtered
          // this way.
          hppDout (info, "Joint of locked joint not found: " << **it);
          continue;
        }
        bool cstRHS = (*it)->constantRightHandSide();

        // JointPtr_t j = robot->getJointByName ((*it)->jointName());
        // const size_type i1 = idx(j),
        const size_type i1 = model.getJointId(jointName),
                        i2 = model.parents[i1];
        recurseSetRelMotion (matrix, i1, i2, (cstRHS ? Constrained : Parameterized));
      }

      // Loop over the DifferentiableFunction
      const NumericalConstraints_t& ncs = proj->numericalConstraints ();
      for (NumericalConstraints_t::const_iterator _ncs = ncs.begin();
          _ncs != ncs.end(); ++_ncs) {
        using hpp::constraints::explicit_::BasicFunction;
        using hpp::constraints::explicit_::GenericFunction;
        const constraints::Implicit& nc = **_ncs;
        size_type i1, i2;

        if (nc.functionPtr()->outputSize() != 6) continue;
        if (   !check< RelativeTransformation>::is (nc.functionPtr(), i1, i2)
            && !check<         Transformation>::is (nc.functionPtr(), i1, i2)
            && !check<          BasicFunction>::is (nc.functionPtr(), i1, i2)
            && !check<        GenericFunction>::is (nc.functionPtr(), i1, i2))
          continue;

        bool cstRHS = nc.constantRightHandSide();
        recurseSetRelMotion (matrix, i1, i2, (cstRHS ? Constrained : Parameterized));
      }
    }

    void RelativeMotion::recurseSetRelMotion(matrix_type& matrix,
        const size_type& i1, const size_type& i2,
        const RelativeMotion::RelativeMotionType& type)
    {
      bool param = (type == Parameterized);
      RelativeMotionType t = Unconstrained;
      if (type == Unconstrained) return;
      symSet(matrix, i1, i2, type);

      // i1 to i3
      for (size_type i3 = 0; i3 < matrix.rows(); ++i3) {
        if (i3 == i1) continue;
        if (i3 == i2) continue;
        if (matrix(i2,i3) != Unconstrained) {
          t = (!param && matrix(i2,i3) == Constrained) ? Constrained : Parameterized;
          symSet(matrix, i1, i3, t);
        }
      }
      for (size_type i0 = 0; i0 < matrix.rows(); ++i0) {
        if (i0 == i2) continue;
        if (i0 == i1) continue;
        // i0 to i2
        if (matrix(i0,i1) != Unconstrained) {
          t = (!param && matrix(i0,i1) == Constrained) ? Constrained : Parameterized;
          symSet (matrix, i0, i2 ,t);
        }

        // from i0 to i3
        if (matrix(i0,i1) == Unconstrained) continue;
        for (size_type i3 = 0; i3 < matrix.rows(); ++i3) {
          if (i3 == i2) continue;
          if (i3 == i1) continue;
          if (i3 == i0) continue;
          if (matrix(i2,i3) == Unconstrained) continue;
          t = (!param && matrix(i0,i1) == Constrained && matrix(i2,i3) == Constrained)
            ? Constrained : Parameterized;
          symSet (matrix, i0, i3, t);
        }
      }
    }
  } // namespace core
} // namespace hpp
