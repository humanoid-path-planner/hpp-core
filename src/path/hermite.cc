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

#include <hpp/core/path/hermite.hh>

#include <hpp/util/debug.hh>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/liegroup.hh>

#include <hpp/core/config-projector.hh>
#include <hpp/core/projection-error.hh>

namespace hpp {
  namespace core {
    namespace path {
      Hermite::Hermite (const DevicePtr_t& device,
                        ConfigurationIn_t init,
                        ConfigurationIn_t end,
                        ConstraintSetPtr_t constraints) :
        parent_t (device, interval_t (0, 1), constraints),
        init_ (init), end_ (end),
        hermiteLength_ (-1)
      {
        assert (init.size() == robot_->configSize ());
        assert (device);

        base (init);
        parameters_.row(0).setZero();
        pinocchio::difference<hpp::pinocchio::RnxSOnLieGroupMap>
          (robot_, init, end, parameters_.row(3));

        projectVelocities(init, end);
      }

      Hermite::Hermite (const Hermite& path) :
        parent_t (path),
        init_ (path.init_), end_ (path.end_),
        hermiteLength_ (-1)
      {}

      Hermite::Hermite (const Hermite& path,
                        const ConstraintSetPtr_t& constraints) :
        parent_t (path, constraints),
        init_ (path.init_), end_ (path.end_),
        hermiteLength_ (-1)
      {
        projectVelocities(init_, end_);
      }

      void Hermite::init (HermitePtr_t self)
      {
        parent_t::init (self);
        weak_ = self;
        checkPath ();
      }

      // void Hermite::computeVelocities (ConfigurationIn_t qi, ConfigurationIn_t qe)
      void Hermite::projectVelocities (ConfigurationIn_t qi, ConfigurationIn_t qe)
      {
        // vector_t v_i2e (outputDerivativeSize());
        // pinocchio::difference<hpp::pinocchio::LieGroupTpl> (robot_, qe, qi, v_i2e);
        if (constraints() && constraints()->configProjector()) {
          ConfigProjectorPtr_t proj = constraints()->configProjector();
          vector_t v (outputDerivativeSize());
          // Compute v0
          // proj->projectVectorOnKernel (qi, v_i2e, v);
          proj->projectVectorOnKernel (qi, parameters_.row(3), v);
          v0 (v);
          // Compute v1
          proj->projectVectorOnKernel (qe, parameters_.row(3), v);
          v1 (v);
        } else {
          v0(parameters_.row(3));
          v1(parameters_.row(3));
        }
        assert (!parameters_.hasNaN());
      }

      void Hermite::computeHermiteLength ()
      {
        hermiteLength_ = (parameters_.bottomRows<3>() - parameters_.topRows<3>()).rowwise().norm().sum();
      }

      vector_t Hermite::velocity (const value_type& param) const
      {
        vector_t v (outputDerivativeSize());
        derivative (v, param, 1);
        if (constraints() && constraints()->configProjector()) {
          ConfigProjectorPtr_t proj = constraints()->configProjector();
          Configuration_t q (outputSize());
          if (!(*this) (q, param))
            throw projection_error ("Configuration does not satisfy the constraints");
          proj->projectVectorOnKernel (q, v, v);
        }
        return v;
      }

      DevicePtr_t Hermite::device () const
      {
        return robot_;
      }
    } //   namespace path
  } //   namespace core
} // namespace hpp


