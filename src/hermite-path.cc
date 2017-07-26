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

#include <hpp/util/debug.hh>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/liegroup.hh>

#include <hpp/core/config-projector.hh>
#include <hpp/core/hermite-path.hh>
#include <hpp/core/projection-error.hh>

namespace hpp {
  namespace core {
    namespace {
      typedef Eigen::Matrix<value_type, 3, 3> matrix3_t;
      typedef Eigen::Matrix<value_type, 3, 1> vector3_t;
      typedef Eigen::DiagonalMatrix<value_type, 3> DiagonalMatrix_t;
      const Eigen::Matrix<value_type, 3, 3> HermiteCoeffs =
        (matrix3_t() << 0,  3, -2,
                        1, -2,  1,
                        0, -1,  1).finished();

      vector3_t powsOfT (const value_type& t) {
        vector3_t pows;
        pows(0) = t;
        pows(1) = pows(0) * t;
        pows(2) = pows(1) * t;
        return pows;
      }

      vector3_t powsOfTprime (const value_type& t) {
        vector3_t pows;
        pows(0) = 1;
        pows(1) = 2 * t;
        pows(2) = 3 * t * t;
        return pows;
      }
    }

    HermitePath::HermitePath (const DevicePtr_t& device,
                              ConfigurationIn_t init,
                              ConfigurationIn_t end) :
      parent_t (interval_t (0, 1), device->configSize (),
		device->numberDof ()),
      device_ (device),
      initial_ (init), end_ (end),
      vs_ (device->numberDof(), 3),
      hermiteLength_ (-1)
    {
      assert (init.size() == device_->configSize ());
      assert (device);
      assert (!constraints ());

      computeVelocities();
    }

    HermitePath::HermitePath (const DevicePtr_t& device,
				ConfigurationIn_t init,
				ConfigurationIn_t end,
				ConstraintSetPtr_t constraints) :
      parent_t (interval_t (0, 1), device->configSize (),
		device->numberDof (), constraints),
      device_ (device),
      initial_ (init), end_ (end),
      vs_ (device->numberDof(), 3),
      hermiteLength_ (-1)
    {
      assert (init.size() == device_->configSize ());
      assert (device);

      computeVelocities();
    }

    HermitePath::HermitePath (const HermitePath& path) :
      parent_t (path), device_ (path.device_),
      initial_ (path.initial_), end_(path.end_),
      vs_ (path.vs_), hermiteLength_ (-1)
    {
      assert (initial().size() == device_->configSize ());
    }

    HermitePath::HermitePath (const HermitePath& path,
                              const ConstraintSetPtr_t& constraints) :
      parent_t (path, constraints), device_ (path.device_),
      initial_ (path.initial_), end_(path.end_),
      hermiteLength_ (-1)
    {
      computeVelocities();
    }

    void HermitePath::init (HermitePathPtr_t self)
    {
      parent_t::init (self);
      weak_ = self;
      checkPath ();
    }

    bool HermitePath::impl_compute (ConfigurationOut_t result,
				     value_type param) const
    {
      assert (timeRange().first == 0 && timeRange().second == 1);
      assert (0 <= param && param <= 1);
      if (param == timeRange ().first) {
	result.noalias () = initial();
	return true;
      }
      if (param >= timeRange ().second) {
	result.noalias () = end();
	return true;
      }

      pinocchio::integrate<true, hpp::pinocchio::LieGroupTpl> (device_, initial_, delta(param), result);
      return true;
    }

    vector_t HermitePath::delta (const value_type& t) const
    {
      assert (0 <= t && t <= 1);
      return vs_ * (HermiteCoeffs * powsOfT (t));
    }

    void HermitePath::computeVelocities ()
    {
      pinocchio::difference<hpp::pinocchio::LieGroupTpl> (device_, end_, initial_, vs_.col(0));
      if (constraints() && constraints()->configProjector()) {
        ConfigProjectorPtr_t proj = constraints()->configProjector();
        vs_.rightCols<2>().setZero();
        proj->projectVectorOnKernel (initial_, vs_.col(0), vs_.col(1));
        proj->projectVectorOnKernel (end_    , vs_.col(0), vs_.col(2));
      } else {
        vs_.col(1) = vs_.col(0);
        vs_.col(2) = vs_.col(0);
      }
      assert (!vs_.hasNaN());
    }

    void HermitePath::computeHermiteLength ()
    {
      hermiteLength_ =
        (vs_.col(1).norm() + vs_.col(2).norm()) / 3
        + (- vs_.col(0) + (vs_.col(1) + vs_.col(2)) / 3).norm();
    }

    vector_t HermitePath::velocity (const value_type& param) const
    {
      const value_type t = param;
      vector_t v = vs_ * (HermiteCoeffs * powsOfTprime (t)); 
      if (constraints() && constraints()->configProjector()) {
        ConfigProjectorPtr_t proj = constraints()->configProjector();
        Configuration_t q (outputSize());
        if (!(*this) (q, t))
          throw projection_error ("Configuration does not satisfy the constraints");
        proj->projectVectorOnKernel (q, v, v);
      }
      return v;
    }

    DevicePtr_t HermitePath::device () const
    {
      return device_;
    }
  } //   namespace core
} // namespace hpp


