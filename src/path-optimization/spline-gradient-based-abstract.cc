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

#include <hpp/core/path-optimization/spline-gradient-based-abstract.hh>

#include <hpp/util/exception-factory.hh>
#include <hpp/util/timer.hh>

#include <hpp/pinocchio/device.hh>

#include <hpp/constraints/svd.hh>

#include <hpp/core/config-projector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/interpolated-path.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/collision-path-validation-report.hh>

#include <path-optimization/spline-gradient-based/joint-bounds.hh>

namespace hpp {
  namespace core {
    using pinocchio::Device;

    namespace pathOptimization {
      typedef Eigen::Matrix<value_type, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMajorMatrix_t;
      typedef Eigen::Map<const vector_t> ConstVectorMap_t;
      typedef Eigen::Map<      vector_t>      VectorMap_t;

      typedef Eigen::BlockIndex BlockIndex;

      HPP_DEFINE_TIMECOUNTER(SGB_validatePath);

      template <int NbRows>
      VectorMap_t reshape (Eigen::Matrix<value_type, NbRows, Eigen::Dynamic, Eigen::RowMajor>& parameters)
      {
        return VectorMap_t (parameters.data(), parameters.size());
      }

      template <int NbRows>
      ConstVectorMap_t reshape (const Eigen::Matrix<value_type, NbRows, Eigen::Dynamic, Eigen::RowMajor>& parameters)
      {
        return ConstVectorMap_t (parameters.data(), parameters.size());
      }

      template <int _PB, int _SO>
      SplineGradientBasedAbstract<_PB, _SO>::SplineGradientBasedAbstract (const Problem& problem)
        : PathOptimizer (problem),
        steeringMethod_(SSM_t::create(problem)),
        robot_ (problem.robot())
      {}

      // ----------- Convenience class -------------------------------------- //

      /// \param lc the constraint to be updated.
      /// \param row the row into \c lc.J and \c lc.b where to write.
      /// \param col the index of the first DoF.
      /// \param rDoF the number of DoF of the robot.
      /// \param select the DoF indices onto which we optimize (the others
      ///               being explicitely computed.)
      /// \param Bl, Br the Spline::basisFunctionDerivative on the left and right.
      /// \param splineL, splineR the left and right spline.
      /// \param Czero true for continuity of the path, false for any of its
      ///              derivative.
      ///
      /// \c select.nbIndices() rows will be written in \c lc.J and \c lc.b .
      /// \c select.nbIndices() rows will be written in \c lc.J and \c lc.b .
      template <typename SplineType>
      static inline void setContinuityConstraint (LinearConstraint& lc, const size_type& row,
          const size_type& col,
          const size_type& rDof,
          const Eigen::RowBlockIndices select,
          const typename SplineType::BasisFunctionVector_t& Bl,
          const typename SplineType::BasisFunctionVector_t& Br,
          const typename SplineType::Ptr_t& splineL,
          const typename SplineType::Ptr_t& splineR,
          bool Czero)
      {
        const size_type& rows = select.nbIndices();
        size_type c = col;
        lc.b.segment(row, rows).setZero();
        if (splineL) {
          for (size_type i = 0; i < SplineType::NbCoeffs; ++i) {
            lc.J.block (row, c, rows, rDof).noalias()
              = select.rview ( - Bl(i) * matrix_t::Identity(rDof, rDof));
            c += rDof;
          }
          if (Czero)
            lc.b.segment(row, rows).noalias()
              = select.rview (splineL->parameters ().transpose() * (-Bl));
        }
        if (splineR) {
          for (size_type i = 0; i < SplineType::NbCoeffs; ++i) {
            lc.J.block (row, c, rows, rDof).noalias()
              = select.rview ( Br(i) * matrix_t::Identity(rDof, rDof));
            c += rDof;
          }
          if (Czero)
            lc.b.segment(row, rows).noalias()
              += select.rview (splineR->parameters ().transpose() * Br).eval();
        }
      }

      // ----------- Resolution steps --------------------------------------- //

      template <int _PB, int _SO>
      void SplineGradientBasedAbstract<_PB, _SO>::appendEquivalentSpline
      (const StraightPathPtr_t& path, Splines_t& splines) const
      {
        PathPtr_t s =
          steeringMethod_->impl_compute (path->initial(), path->end());
        if (path->constraints()) {
          splines.push_back(
              HPP_DYNAMIC_PTR_CAST(Spline, s->copy (path->constraints()))
              );
        } else {
          splines.push_back(HPP_DYNAMIC_PTR_CAST(Spline, s));
        }
      }

      template <int _PB, int _SO>
      void SplineGradientBasedAbstract<_PB, _SO>::appendEquivalentSpline
      (const InterpolatedPathPtr_t& path, Splines_t& splines) const
      {
        if (path->interpolationPoints().size() > 2) {
          HPP_THROW (std::logic_error,
              "Projected path with more than 2 IPs are not supported. "
              << path->interpolationPoints().size() << ").");
        }
        PathPtr_t s =
          steeringMethod_->impl_compute (path->initial(), path->end());
        if (path->constraints()) {
          splines.push_back(
              HPP_DYNAMIC_PTR_CAST(Spline, s->copy (path->constraints()))
              );
        } else {
          splines.push_back(HPP_DYNAMIC_PTR_CAST(Spline, s));
        }
      }

      template <int _PB, int _SO>
      void SplineGradientBasedAbstract<_PB, _SO>::appendEquivalentSpline
      (const PathVectorPtr_t& path, Splines_t& splines) const
      {
        for (std::size_t i = 0; i < path->numberPaths(); ++i)
        {
          PathPtr_t p = path->pathAtRank(i);
          StraightPathPtr_t straight (HPP_DYNAMIC_PTR_CAST(StraightPath, p));
          PathVectorPtr_t pvect      (HPP_DYNAMIC_PTR_CAST(PathVector, p));
          InterpolatedPathPtr_t intp (HPP_DYNAMIC_PTR_CAST(InterpolatedPath, p));
          SplinePtr_t spline         (HPP_DYNAMIC_PTR_CAST(Spline, p));
          if (straight   ) appendEquivalentSpline(straight, splines);
          else if (pvect ) appendEquivalentSpline(pvect   , splines);
          else if (intp  ) appendEquivalentSpline(intp    , splines);
          else if (spline) splines.push_back(HPP_STATIC_PTR_CAST(Spline, spline->copy()));
          else // if TODO check if path is another type of spline.
            throw std::logic_error("Unknown type of path");
        }
      }

      template <int _PB, int _SO>
      void SplineGradientBasedAbstract<_PB, _SO>::initializePathValidation
      (const Splines_t& splines)
      {
        validations_.resize(splines.size());
        for (std::size_t i = 0; i < splines.size(); ++i) {
          validations_[i] = problem ().pathValidation();
        }
      }

      template <int _PB, int _SO>
      typename SplineGradientBasedAbstract<_PB, _SO>::Reports_t
      SplineGradientBasedAbstract<_PB, _SO>::validatePath
      (const Splines_t& splines, bool stopAtFirst) const
      {
        assert (validations_.size() == splines.size());
        HPP_SCOPE_TIMECOUNTER(SGB_validatePath);
	PathPtr_t validPart;
	PathValidationReportPtr_t report;
	Reports_t reports;
        for (std::size_t i = 0; i < splines.size(); ++i) {
	  if (!validations_[i]->validate (splines[i], false, validPart, report)) {
	    HPP_STATIC_CAST_REF_CHECK (CollisionPathValidationReport, *report);
	    reports.push_back
	      (std::make_pair (HPP_STATIC_PTR_CAST
			       (CollisionPathValidationReport, report), i));
            if (stopAtFirst) break;
	  }
	}
        HPP_DISPLAY_TIMECOUNTER(SGB_validatePath);
        return reports;
      }

      template <int _PB, int _SO>
      void SplineGradientBasedAbstract<_PB, _SO>::addContinuityConstraints
      (const Splines_t& splines, const size_type maxOrder,
       const SplineOptimizationDatas_t& sods, LinearConstraint& lc)
      {
        typename Spline::BasisFunctionVector_t B0, B1;
        enum { NbCoeffs = Spline::NbCoeffs };

        const size_type rDof = robot_->numberDof();

        // Compute which continuity constraint are necessary
        size_type nbRows = 0;
        std::vector<RowBlockIndices> rbis (splines.size() + 1);
        BlockIndex::segment_t space (0, rDof);

        // 1. Constrain the starting position.
        rbis[0] = sods[0].activeParameters;
        nbRows += rbis[0].nbIndices();

        // 2. For each consecutive splines, constrain the intersection.
        for (std::size_t i = 1; i < sods.size(); ++i) {
          // Compute union between A = sods[i-1].activeParameters.indices()
          // and B = sods[i].activeParameters.indices()
          BlockIndex::segments_t v (sods[i-1].activeParameters.indices());
          v.insert(v.end(), sods[i].activeParameters.indices().begin(), sods[i].activeParameters.indices().end());
          rbis[i] = RowBlockIndices (v);
          rbis[i].updateRows<true, true, true>();
          hppDout (info, "Optimize waypoint " << i << " over " << rbis[i]);
          nbRows += rbis[i].nbIndices();
        }

        // 3. Constrain the final position.
        rbis[sods.size()] = sods[sods.size()-1].activeParameters;
        nbRows += rbis[sods.size()].nbIndices();

        nbRows *= (maxOrder + 1);

        // Create continuity constraint
        size_type row = lc.J.rows(), paramSize = rDof * Spline::NbCoeffs;
        lc.addRows (nbRows);

        for (size_type k = 0; k <= maxOrder; ++k) {
          bool Czero = (k == 0);

          // Continuity at the beginning
          splines[0]->basisFunctionDerivative(k, 0, B0);
          size_type indexParam = 0;

          setContinuityConstraint <Spline>
            (lc, row, indexParam, rDof, rbis[0],
             B1, B0, SplinePtr_t(), splines[0], Czero);
          row += rbis[0].nbIndices();

          for (std::size_t j = 0; j < splines.size() - 1; ++j) {
            splines[j  ]->basisFunctionDerivative(k, 1, B1);
            splines[j+1]->basisFunctionDerivative(k, 0, B0);

            // Continuity between spline i and j
            setContinuityConstraint <Spline>
              (lc, row, indexParam, rDof, rbis[j+1],
               B1, B0, splines[j], splines[j+1], Czero);

            row += rbis[j+1].nbIndices();
            indexParam += paramSize;
          }

          // Continuity at the end
          splines.back()->basisFunctionDerivative(k, 1, B1);
          setContinuityConstraint <Spline>
            (lc, row, indexParam, rDof, rbis.back(),
             B1, B0, splines.back(), SplinePtr_t(), Czero);
          row += rbis.back().nbIndices();

          assert (indexParam + paramSize == lc.J.cols());
        }
        assert(row == lc.J.rows());
      }

      template <int _PB, int _SO>
      typename SplineGradientBasedAbstract<_PB, _SO>::Indices_t
      SplineGradientBasedAbstract<_PB, _SO>::validateBounds
      (const Splines_t& splines, const LinearConstraint& lc) const
      {
        Indices_t violated;

        const size_type rDof = robot_->numberDof();
        const size_type cols = rDof * Spline::NbCoeffs;
        const size_type rows = lc.J.rows() / splines.size();
        const size_type nBoundedDof = rows / (2*Spline::NbCoeffs);
        assert (lc.J.rows() % splines.size() == 0);
        assert (rows % (2*Spline::NbCoeffs) == 0);

        const value_type thr = Eigen::NumTraits<value_type>::dummy_precision();
        vector_t err;
        std::vector<bool> dofActive (nBoundedDof, false);

        for (std::size_t i = 0; i < splines.size(); ++i) {
          const size_type row = i * rows;
          const size_type col = i * cols;

          err = lc.J.block(row, col, rows, cols) * splines[i]->rowParameters()
            - lc.b.segment(row, rows);

          // Find the maximum per parameter
          for (size_type j = 0; j < nBoundedDof; ++j) {
            if (dofActive[j]) continue;
            value_type errM = -thr;
            size_type iErrM = -1;
            for (size_type k = 0; k < Spline::NbCoeffs; ++k) {
              const size_type low = 2*j + k * 2*nBoundedDof;
              const size_type up  = 2*j + k * 2*nBoundedDof + 1;
              if (err(low) < errM) {
                iErrM = low;
                errM = err(low);
              }
              if (err(up) < errM) {
                iErrM = up;
                errM = err(up);
              }
            }
            if (iErrM >= 0) {
              violated.push_back(row + iErrM);
              dofActive[j] = true;
              hppDout(info, "Bound violation at spline " << i << ", param " << (iErrM - 2*j) / nBoundedDof
                  << ", iDof " << j << ": " << errM);
            }
          }
	}
        if (!violated.empty()) {
          hppDout (info, violated.size() << " bounds violated." );
        }
        return violated;
      }

      template <int _PB, int _SO>
      std::size_t SplineGradientBasedAbstract<_PB, _SO>::addBoundConstraints
      (const Indices_t& bci, const LinearConstraint& bc,
       Bools_t& activeConstraint, LinearConstraint& constraint) const
      {
        std::size_t nbC = 0;
        for (std::size_t i = 0; i < bci.size(); i++) {
          if (!activeConstraint[bci[i]]) {
            ++nbC;
            constraint.addRows (1);
            constraint.J.template bottomRows<1>() = bc.J.row(bci[i]);
            constraint.b(constraint.b.size() - 1) = bc.b (bci[i]);
          }
        }
        return nbC;
      }

      template <int _PB, int _SO>
      PathVectorPtr_t SplineGradientBasedAbstract<_PB, _SO>::buildPathVector
      (const Splines_t& splines) const
      {
        PathVectorPtr_t pv =
          PathVector::create (robot_->configSize(), robot_->numberDof());

        for (std::size_t i = 0; i < splines.size(); ++i)
          pv->appendPath (splines[i]);
        return pv;
      }

      // ----------- Convenience functions ---------------------------------- //

      template <int _PB, int _SO>
      void SplineGradientBasedAbstract<_PB, _SO>::jointBoundConstraint
      (const Splines_t& splines, LinearConstraint& lc) const
      {
        const size_type rDof = robot_->numberDof();
        const size_type cols = Spline::NbCoeffs * rDof;

        matrix_t A (2*rDof, rDof);
        vector_t b (2*rDof);
        const size_type rows = jointBoundMatrices (robot_, robot_->neutralConfiguration(), A, b);

        A.resize(rows, rDof);
        b.resize(rows);

        const size_type size = Spline::NbCoeffs * rows;
        lc.J.resize(splines.size() * size, splines.size() * cols);
        lc.b.resize(splines.size() * size);

        lc.J.setZero();
        lc.b.setZero();

        for (std::size_t i = 0; i < splines.size(); ++i) {
          jointBoundMatrices (robot_, splines[i]->base(), A, b);
          for (size_type k = 0; k < Spline::NbCoeffs; ++k) {
            const size_type row = i * size + k * rows;
            const size_type col = i * cols + k * rDof;
            lc.J.block (row, col, rows, rDof) = -A;
            lc.b.segment (row, rows)          = -b;
          }
        }
      }

      template <int _PB, int _SO>
      void SplineGradientBasedAbstract<_PB, _SO>::copy
      (const Splines_t& in, Splines_t& out)
      {
        out.resize(in.size());
        for (std::size_t i = 0; i < in.size(); ++i)
          out[i] = HPP_STATIC_PTR_CAST(Spline, in[i]->copy());
      }

      template <int _PB, int _SO>
      void SplineGradientBasedAbstract<_PB, _SO>::updateSplines
      (Splines_t& splines, const vector_t& param) const
      {
        size_type row = 0, size = robot_->numberDof() * Spline::NbCoeffs;
        for (std::size_t i = 0; i < splines.size(); ++i) {
          splines[i]->rowParameters(param.segment(row, size));
          row += size;
        }
      }

      template <int _PB, int _SO>
      void SplineGradientBasedAbstract<_PB, _SO>::interpolate
      (const Splines_t& a, const Splines_t& b, const value_type& alpha, Splines_t& res)
      {
        assert (a.size() == b.size() && b.size() == res.size());
        assert (alpha >= 0 && alpha <= 1);

        for (std::size_t i = 0; i < a.size(); ++i)
          res[i]->rowParameters(
              (1 - alpha) * a[i]->rowParameters()
              + alpha     * b[i]->rowParameters());
      }

      // ----------- Instanciate -------------------------------------------- //

      // template class SplineGradientBased<path::CanonicalPolynomeBasis, 1>; // equivalent to StraightPath
      // template class SplineGradientBased<path::CanonicalPolynomeBasis, 2>;
      // template class SplineGradientBased<path::CanonicalPolynomeBasis, 3>;
      template class SplineGradientBasedAbstract<path::BernsteinBasis, 1>; // equivalent to StraightPath
      // template class SplineGradientBased<path::BernsteinBasis, 2>;
      template class SplineGradientBasedAbstract<path::BernsteinBasis, 3>;
    } // namespace pathOptimization
  }  // namespace core
} // namespace hpp
