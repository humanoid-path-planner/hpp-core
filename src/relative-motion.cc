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

#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>

#include <hpp/constraints/generic-transformation.hh>

#include <hpp/core/constraint-set.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/locked-joint.hh>

namespace hpp {
  namespace core {
    namespace {
      inline void symSet (RelativeMotion::matrix_type& m, size_type i0, size_type i1, RelativeMotion::RelativeMotionType t)
      {
        m(i0,i1) = m(i1,i0) = t;
      }

      inline JointConstPtr_t getNonAnchorParent (const JointConstPtr_t& j)
      {
        if (j==NULL) return NULL;
        JointConstPtr_t parent = j;
        // Find the closest non-fixed parent in the kinematic chain
        while (
            ((parent = parent->parentJoint()) != NULL) 
            && parent->numberDof() == 0) {}
        return parent;
      }
    }

    RelativeMotion::matrix_type RelativeMotion::matrix (const DevicePtr_t& dev)
    {
      assert (dev);
      const size_type N = dev->numberDof () + 1;
      matrix_type matrix (N, N);
      matrix.setConstant (Unconstrained);
      matrix.diagonal().setConstant(Constrained);
      return matrix;
    }

    void RelativeMotion::fromConstraint (matrix_type& matrix,
        const DevicePtr_t& robot,
        const ConstraintSetPtr_t& c)
    {
      using constraints::RelativeTransformation;
      using constraints::RelativeTransformationPtr_t;
      assert (robot);
      assert (c);

      const size_type N = robot->numberDof () + 1;
      if (matrix.rows() != N || matrix.cols() != N)
        throw std::invalid_argument ("Wrong RelativeMotion::matrix_type size");

      ConfigProjectorPtr_t proj = c->configProjector();
      if (!proj) return;

      // Loop over the LockedJoint
      const LockedJoints_t& lj = proj->lockedJoints ();
      for (LockedJoints_t::const_iterator it = lj.begin ();
          it != lj.end (); ++it) {
        JointPtr_t j;
        try {
          // Extra dofs and partial locked joints have a name that won't be
          // recognized by Device::getJointByName. So they can be filtered
          // this way.
          j = robot->getJointByName ((*it)->jointName());
        } catch (const std::runtime_error& e) {
          hppDout (info, "Joint not found:" << e.what ());
          continue;
        }
        bool cstRHS = (*it)->comparisonType()->constantRightHandSide();

        const size_type i1 = idx(j),
                        i2 = idx(getNonAnchorParent(j));
        recurseSetRelMotion (matrix, i1, i2, (cstRHS ? Constrained : Parameterized));
      }

      // Loop over the DifferentiableFunction
      const NumericalConstraints_t& ncs = proj->numericalConstraints ();
      for (NumericalConstraints_t::const_iterator _ncs = ncs.begin();
          _ncs != ncs.end(); ++_ncs) {
        const NumericalConstraint& nc = **_ncs;
        if (nc.comparisonType()->constantRightHandSide()) {
          RelativeTransformationPtr_t rt =
            HPP_DYNAMIC_PTR_CAST(RelativeTransformation,
                nc.functionPtr());
          if (!rt || rt->outputSize() != 6) continue;
          const size_type i1 = idx(rt->joint1()),
                          i2 = idx(rt->joint2());

          bool cstRHS = nc.comparisonType()->constantRightHandSide();
          recurseSetRelMotion (matrix, i1, i2, (cstRHS ? Constrained : Parameterized));
        }
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

    size_type RelativeMotion::idx(const JointConstPtr_t& joint)
    {
      if (joint == NULL) return 0;
      if (joint->numberDof() == 0) {
        const JointConstPtr_t j = getNonAnchorParent(joint);
        return (j == NULL ? 0 : j->rankInVelocity() + 1);
      }
      return joint->rankInVelocity() + 1;
    }
  } // namespace core
} // namespace hpp
