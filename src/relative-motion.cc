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

#include <hpp/core/constraint-set.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/locked-joint.hh>

namespace hpp {
  namespace core {
    RelativeMotion::matrix_type RelativeMotion::matrix (const DevicePtr_t& dev)
    {
      assert (dev);
      const size_type N = dev->numberDof () + 1;
      matrix_type matrix (N, N);
      matrix.setConstant (Unconstrained);
      return matrix;
    }

    void RelativeMotion::fromConstraint (matrix_type& matrix,
        const DevicePtr_t& robot,
        const ConstraintSetPtr_t& c)
    {
      assert (robot);
      assert (c);

      const size_type N = robot->numberDof () + 1;
      if (matrix.rows() != N || matrix.cols() != N)
        throw std::invalid_argument ("Wrong RelativeMotion::matrix_type size");

      ConfigProjectorPtr_t proj = c->configProjector();
      if (!proj) return;
      const LockedJoints_t& lj = proj->lockedJoints ();

      typedef std::map <std::string, std::size_t> StringIndex_m;
      typedef std::pair<JointPtr_t, bool> JointAndCstRhs_t;
      typedef std::list<JointAndCstRhs_t> Joints_t;
      typedef std::vector<Joints_t> JointLists_t;

      StringIndex_m nameToIndex;
      // A vector of (list of sequential locked joints)
      JointLists_t jls;
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
        JointPtr_t parent = j->parentJoint ();

        bool cstRHS = (*it)->comparisonType()->constantRightHandSide();

        // Find the closest non-fixed parent in the kinematic chain
        while (parent != NULL && parent->numberDof() == 0)
          parent = parent->parentJoint();

        if (parent == NULL ||
            nameToIndex.find (parent->name()) == nameToIndex.end()) {
          // Create a new list
          nameToIndex[j->name()] = jls.size();
          Joints_t js;
          if (parent == NULL) js.push_back (JointAndCstRhs_t (NULL, true));
          js.push_back (JointAndCstRhs_t (j, cstRHS));
          jls.push_back (js);
        } else {
          std::size_t i = nameToIndex[parent->name()];
          jls[i].push_back (JointAndCstRhs_t (j, cstRHS));
          nameToIndex[j->name()] = i;
        }
      }

      // Build the matrix of joint pairs to be disabled.
      size_type i1, i2;
      for (JointLists_t::const_iterator _jls = jls.begin();
          _jls != jls.end(); ++_jls) {
        for (Joints_t::const_iterator _js1 = _jls->begin();
            _js1 != _jls->end(); ++_js1) {
          RelativeMotionType type = Constrained;
          const JointPtr_t j1 = _js1->first;
          i1 = ((j1 == NULL) ? N - 1 : j1->rankInVelocity());
          matrix(i1, i1) = Constrained;
          for (Joints_t::const_iterator _js2 = ++Joints_t::const_iterator(_js1);
              _js2 != _jls->end(); ++_js2) {
            // If one LockedJoint in the chain has non-constant RHS,
            // then the relative motion between the joints is parameterized.
            if (!_js2->second) type = Parameterized;
            const JointPtr_t j2 = _js2->first;
            i2 = ((j2 == NULL) ? N - 1 : j2->rankInVelocity());
            matrix(i1, i2) = type;
            matrix(i2, i1) = type;
          }
        }
      }
    }
  } // namespace core
} // namespace hpp
