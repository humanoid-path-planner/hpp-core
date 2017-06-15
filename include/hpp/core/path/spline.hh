// Copyright (c) 2017 CNRS
// Authors: Joseph Mirabel
//
// This file is part of hpp-core
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
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_CORE_PATH_SPLINE_HH
# define HPP_CORE_PATH_SPLINE_HH

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>
# include <hpp/core/path.hh>

# include <hpp/constraints/matrix-view.hh>

namespace hpp {
  namespace core {
    namespace path {
      /// \addtogroup path
      /// \{

      enum SplineType {
        Canonical
      };

      /// Base class for spline paths
      template <SplineType type, int _Order>
      class HPP_CORE_DLLAPI Spline : public Path
      {
        public:
          enum { Order = _Order };

          void velocity (vectorOut_t res, const value_type& t) const
          {
            assert (outputDerivativeSize() == res.size());
            impl_velocity (res, t);
          }

          size_type parameterSize () const
          {
            return parameterSize_;
          }

          size_type parameterDerivativeSize () const
          {
            return parameterDerivativeSize_;
          }

          size_type order () const
          {
            return order_;
          }

          void parameterDerivative (matrixOut_t res, const value_type& t) const
          {
            assert (outputDerivativeSize()   == res.rows());
            assert (parameterDerivativeSize_ == res.cols());
            impl_paramDerivative (res, t);
          }

          void parameterIntegrate (vectorIn_t dParam)
          {
            assert (parameterDerivativeSize_ == dParam.size());
            impl_paramIntegrate (dParam);
          }

        protected:
          Spline (const DevicePtr_t& robot,
              const interval_t& interval,
              const ConstraintSetPtr_t& constraints)
            : Path (interval, robot->configSize(), robot->numberDof(), constraints),
            robot_ (robot)
          {}

          Spline (const Spline& path) : Path (path) {}

          Spline (const Spline& path, const ConstraintSetPtr_t& constraints)
            : Path (path, constraints)
          {}

          void init (const SplinePtr_t& self) { Path::init(self); weak_ = self; }

          void initCopy (const SplinePtr_t& self) { Path::initCopy(self); weak_ = self; }

          virtual void impl_velocity (vectorOut_t res, const value_type& t) const;

          virtual void impl_paramDerivative (vectorOut_t res, const value_type& t) const;

          virtual void impl_paramIntegrate (vectorIn_t dParam);

          virtual void impl_basisFunctionDerivative (const size_type order, const value_type& t, matrixOut_t res) const = 0;

          size_type parameterSize_, parameterDerivativeSize_, order_;
          DevicePtr_t robot_;

        private:
          SplineWkPtr_t weak_;
      }; // class Spline
      /// \}
    } //   namespace path
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_PATH_SPLINE_HH
