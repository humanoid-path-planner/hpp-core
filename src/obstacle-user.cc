// Copyright (c) 2019 CNRS
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

#include <hpp/core/obstacle-user.hh>

#include <hpp/util/exception-factory.hh>
#include <hpp/fcl/collision.h>

#include <pinocchio/multibody/geometry.hpp>

#include <hpp/pinocchio/body.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/collision-object.hh>

namespace hpp {
  namespace core {
    bool ObstacleUser::collide (const CollisionPairs_t& pairs,
        const CollisionRequests_t& reqs,
        fcl::CollisionResult& res,
        std::size_t& i,
        pinocchio::DeviceData& data)
    {
      for (i = 0; i < pairs.size(); ++i) {
        res.clear();
        assert(!pairs[i].first ->fcl (data)->getTranslation().hasNaN());
        assert(!pairs[i].first ->fcl (data)->getRotation().hasNaN());
        assert(!pairs[i].second->fcl (data)->getTranslation().hasNaN());
        assert(!pairs[i].second->fcl (data)->getRotation().hasNaN());
        if (fcl::collide (
              pairs[i].first ->fcl (data),
              pairs[i].second->fcl (data),
              reqs[i], res) != 0)
          return true;
      }
      return false;
    }

    void ObstacleUser::addObstacle (const CollisionObjectConstPtr_t& object)
    {
      for (size_type j = 0; j < robot_->nbJoints(); ++j) {
        JointPtr_t joint = robot_->jointAt (j);
        addObstacleToJoint (object, joint, false);
      }
    }

    void ObstacleUser::addObstacleToJoint (
        const CollisionObjectConstPtr_t& object,
        const JointPtr_t& joint, const bool includeChildren)
    {
        BodyPtr_t body = joint->linkedBody ();
        if (body) {
            for (size_type o = 0; o < body->nbInnerObjects(); ++o) {
              // TODO: check the objects are not in same joint
              cPairs_.push_back (CollisionPair_t (body->innerObjectAt(o), object));
              cRequests_.push_back (defaultRequest_);
            }
        }
        if(includeChildren) {
            for(std::size_t i=0; i<joint->numberChildJoints(); ++i){
                addObstacleToJoint (object, joint->childJoint(i),includeChildren);
            }
        }
    }

    struct CollisionPairComparision {
      CollisionPair_t a;
      CollisionPairComparision (const CollisionPair_t& p) : a (p) {}
      bool operator() (const CollisionPair_t& b)
      {
        return (&(a.first ->pinocchio()) == &(b.first ->pinocchio()))
          &&   (&(a.second->pinocchio()) == &(b.second->pinocchio()));
      }
    };

    void ObstacleUser::removeObstacleFromJoint
    (const JointPtr_t& joint, const CollisionObjectConstPtr_t& obstacle)
    {
      BodyPtr_t body = joint->linkedBody ();
      if (body) {
        for (size_type o = 0; o < body->nbInnerObjects(); ++o) {
          CollisionPair_t colPair (body->innerObjectAt(o), obstacle);
          CollisionPairComparision compare (colPair);
          std::size_t nbDelPairs = 0;
          for (std::size_t i = 0; i < cPairs_.size();) {
            if (compare(cPairs_[i])) {
              cPairs_.erase(cPairs_.begin()+i);
              cRequests_.erase(cRequests_.begin()+i);
              ++nbDelPairs;
            } else ++i;
          }
          if (nbDelPairs == 0) {
            std::ostringstream oss;
            oss << "ObstacleUser::removeObstacleFromJoint: obstacle \""
                << obstacle->name () <<
                "\" is not registered as obstacle for joint \"" << joint->name ()
                << "\".";
            throw std::runtime_error (oss.str ());
          } else if (nbDelPairs >= 2) {
            hppDout (error, "obstacle "<< obstacle->name () <<
                     " was registered " << nbDelPairs
                     << " times as obstacle for joint " << joint->name ()
                     << ".");
          }
        }
      }
    }

    void ObstacleUser::filterCollisionPairs (const RelativeMotion::matrix_type& matrix)
    {
      // Loop over collision pairs and remove disabled ones.
      pinocchio::JointIndex j1, j2;
      fcl::CollisionResult unused;
      for (std::size_t i = 0; i < cPairs_.size();) {
        const CollisionPair_t& pair = cPairs_[i];

        j1 = pair.first ->jointIndex();
        j2 = pair.second->jointIndex();

        switch (matrix(j1, j2)) {
          case RelativeMotion::Parameterized:
              hppDout(info, "Parameterized collision pairs between "
                  << pair.first ->name() << " and "
                  << pair.second->name());
              pPairs_.push_back (pair);
              pRequests_.push_back (cRequests_[i]);
              cPairs_.erase(cPairs_.begin()+i);
              cRequests_.erase(cRequests_.begin()+i);
              break;
          case RelativeMotion::Constrained:
              hppDout(info, "Disabling collision between "
                  << pair.first ->name() << " and "
                  << pair.second->name());
              if (fcl::collide (pair.first ->fcl (), pair.second->fcl (),
                    cRequests_[i], unused) != 0) {
                hppDout(warning, "Disabling collision detection between two "
                    "bodies in collision.");
              }
              dPairs_.push_back (pair);
              dRequests_.push_back (cRequests_[i]);
              cPairs_.erase(cPairs_.begin()+i);
              cRequests_.erase(cRequests_.begin()+i);
              break;
          case RelativeMotion::Unconstrained: ++i; break;
          default:
            hppDout (warning, "RelativeMotionType not understood");
            ++i;
            break;
        }
      }
    }

    void ObstacleUser::setSecurityMargins(const matrix_t& securityMatrix)
    {
      if (   securityMatrix.rows() != robot_->nbJoints()+1
          || securityMatrix.cols() != robot_->nbJoints()+1)
      {
        HPP_THROW(std::invalid_argument, "Wrong size of security margin matrix."
            " Expected " << robot_->nbJoints()+1 << 'x' << robot_->nbJoints()+1
            << ". Got " << securityMatrix.rows() << 'x' << securityMatrix.cols()
            );
      }
      pinocchio::JointIndex j1, j2;
      fcl::CollisionResult unused;
      for (std::size_t i = 0; i < cPairs_.size(); ++i) {
        const CollisionPair_t& pair = cPairs_[i];
        j1 = pair.first ->jointIndex();
        j2 = pair.second->jointIndex();
        cRequests_[i].security_margin = securityMatrix(j1, j2);
      }
      for (std::size_t i = 0; i < pPairs_.size(); ++i) {
        const CollisionPair_t& pair = pPairs_[i];
        j1 = pair.first ->jointIndex();
        j2 = pair.second->jointIndex();
        pRequests_[i].security_margin = securityMatrix(j1, j2);
      }
      for (std::size_t i = 0; i < dPairs_.size(); ++i) {
        const CollisionPair_t& pair = dPairs_[i];
        j1 = pair.first ->jointIndex();
        j2 = pair.second->jointIndex();
        dRequests_[i].security_margin = securityMatrix(j1, j2);
      }
    }

    void ObstacleUser::addRobotCollisionPairs ()
    {
      const pinocchio::GeomModel& model = robot_->geomModel();
      const pinocchio::GeomData & data  = robot_->geomData();

      for (std::size_t i = 0; i < model.collisionPairs.size(); ++i)
        if (data.activeCollisionPairs[i]) {
          CollisionObjectConstPtr_t o1 (new pinocchio::CollisionObject(robot_,
                model.collisionPairs[i].first));
          CollisionObjectConstPtr_t o2 (new pinocchio::CollisionObject(robot_,
                model.collisionPairs[i].second));
          cPairs_.push_back(CollisionPair_t (o1, o2));
          cRequests_.push_back (defaultRequest_);
        }
    }

    void ObstacleUser::setRequests (const fcl::CollisionRequest& r)
    {
      for (std::size_t i = 0; i < cRequests_.size(); ++i) cRequests_[i] = r;
      for (std::size_t i = 0; i < pRequests_.size(); ++i) pRequests_[i] = r;
      for (std::size_t i = 0; i < dRequests_.size(); ++i) dRequests_[i] = r;
    }
  } // namespace core
} // namespace hpp
