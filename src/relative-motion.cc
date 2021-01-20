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

      /// Check that a differentiable function defines a constraint of
      /// relative pose between two joints
      template <typename T > struct check {
        typedef boost::shared_ptr<T> Ptr_t;
        /// Check that function defines relative pose constraint
        /// \param f Differentiable function,
        /// \return whether f defines a relative pose constraint,
        /// \retval i1, i2 indices of joints that are constrained if so.
        static bool is (const DifferentiableFunctionPtr_t& f,
                        size_type& i1, size_type& i2)
        {
          Ptr_t t = HPP_DYNAMIC_PTR_CAST(T, f);
          if (t) {
            i1 = RelativeMotion::idx(t->joint1());
            i2 = RelativeMotion::idx(t->joint2());
            hppDout (info, "function " << f->name ()
                     << " is a (relative) transformation. i1="
                     << i1 << ", i2=" << i2);
            return true;
          }
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
      using constraints::TransformationR3xSO3;
      using constraints::RelativeTransformationR3xSO3;
      assert (robot);
      assert (c);

      const size_type N = robot->model().joints.size();
      if (matrix.rows() != N || matrix.cols() != N)
        throw std::invalid_argument ("Wrong RelativeMotion::matrix_type size");

      ConfigProjectorPtr_t proj = c->configProjector();
      if (!proj) return;

      // Loop over the LockedJoint
      const pinocchio::Model& model = robot->model();
      // Loop over the constraints
      const NumericalConstraints_t& ncs = proj->numericalConstraints ();
      for (NumericalConstraints_t::const_iterator _ncs = ncs.begin();
          _ncs != ncs.end(); ++_ncs) {
        constraints::ImplicitConstPtr_t nc = *_ncs;
        size_type i1, i2;
        // Detect locked joints
        LockedJointConstPtr_t lj (HPP_DYNAMIC_PTR_CAST (const LockedJoint, nc));
        if (lj) {
          const std::string& jointName = lj->jointName();
          if (!model.existJointName(jointName)) {
            // Extra dofs and partial locked joints have a name that won't be
            // recognized by Device::getJointByName. So they can be filtered
            // this way.
            hppDout (info, "Joint of locked joint not found: " << *lj);
            continue;
          }
          bool cstRHS (lj->parameterSize () == 0);

          i1 = model.getJointId(jointName); i2 = model.parents[i1];
          recurseSetRelMotion (matrix, i1, i2, (cstRHS ? Constrained :
                                                Parameterized));
          hppDout (info, "Locked joint found: " << lj->jointName ());
          continue;
        }
        // Detect relative pose constraints
        if ((nc->functionPtr()->outputSize() != 6) &&
	    (nc->functionPtr()->outputSize()) != 7) {
          hppDout (info, "Constraint " << nc->functionPtr()->name ()
                   << " is neither of dimension 6 nor 7.");
          continue;
        }

        if (!check <Transformation>::is (nc->functionPtr (), i1, i2)) {
          hppDout (info, "Constraint function " << nc->functionPtr()->name ()
                   << " is not of type Transformation");
	  if (!check <TransformationR3xSO3>::is (nc->functionPtr (), i1, i2)) {
	    hppDout (info, "Constraint function " << nc->functionPtr()->name ()
		     << " is not of type TransformationR3xSO3");
	    if (!check <RelativeTransformation>::is
		(nc->functionPtr (), i1, i2)) {
	      hppDout (info, "Constraint function "
		       << nc->functionPtr()->name ()
		       << " is not of type RelativeTransformation");
	      if (!check <RelativeTransformationR3xSO3>::is
		  (nc->functionPtr (), i1, i2)) {
		hppDout (info, "Constraint function "
			 << nc->functionPtr()->name ()
			 << " is not of type RelativeTransformationR3xSO3");
		continue;
	      }
	    }
	  }
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
