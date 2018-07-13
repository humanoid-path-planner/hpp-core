// Copyright (c) 2017, Joseph Mirabel
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

#ifndef HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_JOINT_BOUNDS_HH
#define HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_JOINT_BOUNDS_HH

#include <pinocchio/algorithm/joint-configuration.hpp>

#include <hpp/pinocchio/liegroup.hh>

#include <hpp/core/path/spline.hh>

namespace hpp {
  namespace core {
    namespace pathOptimization {
      using hpp::pinocchio::RnxSOnLieGroupMap;
      using hpp::pinocchio::liegroup::CartesianProductOperation;
      using hpp::pinocchio::liegroup::SpecialOrthogonalOperation;
      using hpp::pinocchio::liegroup::VectorSpaceOperation;

      template <typename LieGroup>
      struct JointBoundConstraintAlgo {};

      template<int Size, bool rot>
      struct JointBoundConstraintAlgo<VectorSpaceOperation<Size, rot> >
      {
        static void run (const size_type& idxV,
                         vectorIn_t low,
                         vectorIn_t up ,
                         vectorIn_t neutral,
                         matrix_t& A,
                         vector_t& b,
                         size_type& row)
        {
          // size_type row;
          for (std::size_t i = 0; i < Size; ++i) {
            if (!std::isinf (low(i))) {
              // row = A.rows();
              // A.conservativeResize(A.rows() + 1, A.cols());
              // b.conservativeResize(b.rows() + 1);
              A.row(row).setZero();
              A(row, idxV + i) = -1;
              b(row) = -(low(i) - neutral(i));
              row++;
            }
            if (!std::isinf (up(i))) {
              // row = A.rows();
              // A.conservativeResize(A.rows() + 1, A.cols());
              // b.conservativeResize(b.rows() + 1);
              A.row(row).setZero();
              A(row, idxV + i) = 1;
              b(row) = up(i) - neutral(i);
              row++;
            }
          }
        }
      };

      template<int N>
      struct JointBoundConstraintAlgo<SpecialOrthogonalOperation<N> >
      {
        static void run (const size_type&,
                         vectorIn_t,
                         vectorIn_t,
                         vectorIn_t,
                         matrix_t&,
                         vector_t&,
                         size_type&)
        {}
      };

      template<typename LieGroup1, typename LieGroup2>
      struct JointBoundConstraintAlgo<CartesianProductOperation <LieGroup1, LieGroup2> >
      {
        static void run (const size_type& idxV,
                         vectorIn_t low,
                         vectorIn_t up ,
                         vectorIn_t neutral,
                         matrix_t& A,
                         vector_t& b,
                         size_type& row)
        {
          JointBoundConstraintAlgo<LieGroup1>::run (idxV,
                                                    low.head<LieGroup1::NQ>(),
                                                    up .head<LieGroup1::NQ>(),
                                                    neutral.head<LieGroup1::NQ>(),
                                                    A, b, row);
          JointBoundConstraintAlgo<LieGroup2>::run (idxV + LieGroup1::NV,
                                                    low.tail<LieGroup2::NQ>(),
                                                    up .tail<LieGroup2::NQ>(),
                                                    neutral.tail<LieGroup1::NQ>(),
                                                    A, b, row);
        }
      };

      struct JointBoundConstraintStep : public se3::fusion::JointModelVisitor<JointBoundConstraintStep>
      {
        typedef boost::fusion::vector<vectorIn_t,
                                      vectorIn_t,
                                      vectorIn_t,
                                      matrix_t&,
                                      vector_t&,
                                      size_type&> ArgsType;

        JOINT_MODEL_VISITOR_INIT(JointBoundConstraintStep);

        template<typename JointModel>
        static void algo(const se3::JointModelBase<JointModel> & jmodel,
            vectorIn_t low,
            vectorIn_t up,
            vectorIn_t neutral,
            matrix_t& A,
            vector_t& b,
            size_type& row)
        {
          typedef typename RnxSOnLieGroupMap::operation<JointModel>::type LG_t;
          JointBoundConstraintAlgo<LG_t>::run (
              jmodel.idx_v(),
              jmodel.jointConfigSelector(low),
              jmodel.jointConfigSelector(up),
              jmodel.jointConfigSelector(neutral),
              A, b, row);
        }
      };

      template<>
      void JointBoundConstraintStep::algo<se3::JointModelComposite>(
          const se3::JointModelBase<se3::JointModelComposite> & jmodel,
          vectorIn_t low,
          vectorIn_t up,
          vectorIn_t neutral,
          matrix_t& A,
          vector_t& b,
          size_type& row)
      {
        se3::details::Dispatch<JointBoundConstraintStep>::run(jmodel,
            JointBoundConstraintStep::ArgsType(low, up, neutral, A, b, row));
      }

      // The bounds are satisfied iif A * v <= b, where v is the velocity from
      // the neutral configuration.
      // We consider only vector-space.
      // \retval A, b must be of the correct size. If you do not know the number
      //              of rows, set it to 2 * robot->numberDof()
      // \return the number of rows that were set in A and b. The other rows are
      //         left untouched.
      size_type jointBoundMatrices (const DevicePtr_t& d, ConfigurationIn_t neutral,
          matrix_t& A, vector_t& b)
      {
        size_type row = 0;
        const se3::Model& model = d->model();
        for (std::size_t i = 1; i < model.joints.size(); ++i) {
          JointBoundConstraintStep::run(model.joints[i],
              JointBoundConstraintStep::ArgsType(
                model.lowerPositionLimit, model.upperPositionLimit,
                neutral, A, b, row)
              );
        }
        return row;
        // TODO handle extra dof
     }
    } // namespace pathOptimization
  }  // namespace core
} // namespace hpp

#endif // HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_COST_HH
