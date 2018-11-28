//
// Copyright (c) 2014, 2015, 2016, 2017, 2018 CNRS
// Authors: Florent Lamiraux, Joseph Mirabel, Diane Bury
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

#include <hpp/core/continuous-validation.hh>

#include <limits>
#include <pinocchio/multibody/geometry.hpp>
#include <hpp/util/debug.hh>
#include <hpp/core/collision-path-validation-report.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/continuous-validation/initializer.hh>
#include <hpp/core/continuous-validation/solid-solid-collision.hh>

#include <iterator>
namespace hpp {
  namespace core {
    using continuousValidation::BodyPairCollisions_t;
    using continuousValidation::SolidSolidCollision;

    /// Validate interval centered on a path parameter
    /// \param bodyPairCollisions a reference to the pair with smallest interval.
    /// \param config Configuration at abscissa tmin on the path.
    /// \param t parameter value in the path interval of definition
    /// \retval interval interval validated for all validation elements
    /// \retval report reason why the interval is not valid,
    /// \return true if the configuration is collision free for this parameter
    ///         value, false otherwise.
    bool ContinuousValidation::validateConfiguration(BodyPairCollisions_t& bodyPairCollisions,
        const Configuration_t &config, const value_type &t,
        interval_t &interval, PathValidationReportPtr_t &report)
    {
      interval.first = -std::numeric_limits <value_type>::infinity ();
      interval.second = std::numeric_limits <value_type>::infinity ();
      pinocchio::DeviceSync robot (robot_);
      robot.currentConfiguration (config);
      robot.computeForwardKinematics();
      robot.updateGeometryPlacements();
      BodyPairCollisions_t::iterator smallestInterval = bodyPairCollisions.begin();
      if (!validateIntervals<BodyPairCollisions_t, CollisionValidationReportPtr_t>
            (bodyPairCollisions, t, interval, report,
             smallestInterval, robot.d()))
        return false;
      // Put the smallest interval first so that, at next iteration,
      // collision pairs with large interval are not computed.
      if (bodyPairCollisions.size() > 1 && smallestInterval != bodyPairCollisions.begin())
        std::iter_swap (bodyPairCollisions.begin(), smallestInterval);
      return true;
    }


    bool ContinuousValidation::validate(const PathPtr_t &path, bool reverse, PathPtr_t &validPart,
                                              PathValidationReportPtr_t &report)
    {
      if (PathVectorPtr_t pv = HPP_DYNAMIC_PTR_CAST(PathVector, path))
      {
        PathVectorPtr_t validPathVector = PathVector::create(path->outputSize(), path->outputDerivativeSize());
        validPart = validPathVector;
        PathPtr_t localValidPart;
        if (reverse)
        {
          value_type param = path->length();
          std::deque<PathPtr_t> paths;
          for (std::size_t i = pv->numberPaths() + 1; i != 0; --i)
          {
            PathPtr_t localPath(pv->pathAtRank(i - 1));
            if (validate(localPath, reverse, localValidPart, report))
            {
              paths.push_front(localPath->copy());
              param -= localPath->length();
            }
            else
            {
              report->parameter += param - localPath->length();
              paths.push_front(localValidPart->copy());
              for (std::deque<PathPtr_t>::const_iterator it = paths.begin();
                  it != paths.end(); ++it)
              {
                validPathVector->appendPath(*it);
              }
              return false;
            }
          }
          return true;
        }
        else
        {
          value_type param = 0;
          for (std::size_t i = 0; i < pv->numberPaths(); ++i)
          {
            PathPtr_t localPath(pv->pathAtRank(i));
            if (validate(localPath, reverse, localValidPart, report))
            {
              validPathVector->appendPath(localPath->copy());
              param += localPath->length();
            }
            else
            {
              report->parameter += param;
              validPathVector->appendPath(localValidPart->copy());
              return false;
            }
          }
          return true;
        }
      }
      return validateStraightPath(bodyPairCollisions_, path, reverse, validPart, report);
    }

    void ContinuousValidation::addObstacle(const CollisionObjectConstPtr_t &object)
    {
      for (size_type idx = 0; idx < robot_->nbJoints(); ++idx) {
        JointPtr_t joint = robot_->jointAt (idx);
      BodyPtr_t body = joint->linkedBody ();
        if (body)
        {
          ConstObjectStdVector_t objects;
          objects.push_back(object);
          bodyPairCollisions_.push_back(SolidSolidCollision::create (joint, objects, tolerance_));
        }
      }
    }

    void ContinuousValidation::setPath(BodyPairCollisions_t& bodyPairCollisions,
        const PathPtr_t &path, bool reverse)
    {
      for (BodyPairCollisions_t::iterator itPair = bodyPairCollisions.begin ();
      itPair != bodyPairCollisions.end (); ++itPair) {
        (*itPair)->path (path, reverse);
      }
    }

    void ContinuousValidation::removeObstacleFromJoint(const JointPtr_t &joint, const CollisionObjectConstPtr_t &obstacle)
    {
      assert (joint);
      bool removed = false;
      for (BodyPairCollisions_t::iterator itPair = bodyPairCollisions_.begin();
          itPair != bodyPairCollisions_.end(); ++itPair)
      {
        // If jointA == joint and jointB is the root joint.
        if ((*itPair)->indexJointA() == (size_type)joint->index()
            && (*itPair)->indexJointB() == 0)
        {
          if ((*itPair)->removeObjectTo_b(obstacle))
          {
            removed = true;
            if ((*itPair)->pairs().empty())
            {
              bodyPairCollisions_.erase(itPair);
            }
          }
        }
      }
      if (!removed)
      {
        std::ostringstream oss;
        oss << "ContinuousValidation::removeObstacleFromJoint: obstacle \""
            << obstacle->name() << "\" is not registered as obstacle for joint \"" << joint->name()
            << "\".";
        throw std::runtime_error(oss.str());
      }
    }

    void ContinuousValidation::filterCollisionPairs(const RelativeMotion::matrix_type &relMotion)
    {
      // Loop over collision pairs and remove disabled ones.
      size_type ia, ib;
      for (BodyPairCollisions_t::iterator _colPair = bodyPairCollisions_.begin();
          _colPair != bodyPairCollisions_.end();)
      {
        ia = (*_colPair)->indexJointA ();
        ib = (*_colPair)->indexJointB ();
        if (ia < 0 || ib < 0) continue;
        switch (relMotion(ia, ib))
        {
        case RelativeMotion::Parameterized:
          hppDout(info, "Parameterized collision pairs treated as Constrained");
        case RelativeMotion::Constrained:
          hppDout(info, "Disabling collision pair " << **_colPair);
          disabledBodyPairCollisions_.push_back(*_colPair);
          _colPair = bodyPairCollisions_.erase(_colPair);
          break;
        case RelativeMotion::Unconstrained:
          ++_colPair;
          break;
        default:
          hppDout(warning, "RelativeMotionType not understood");
          ++_colPair;
          break;
        }
      }
    }

    void ContinuousValidation::init (ContinuousValidationWkPtr_t weak)
    {
      weak_ = weak;
      initializer_->initContinuousValidation(weak_);
    }

    void ContinuousValidation::changeInitializer (continuousValidation::InitializerPtr_t initializer)
    {
      initializer_ = initializer;
      initializer_->initContinuousValidation(weak_);
      initializer_->reset();
      initializer_->initialize();
    }

    ContinuousValidation::~ContinuousValidation()
    {
    }

    ContinuousValidation::ContinuousValidation(const DevicePtr_t &robot, const value_type &tolerance):
      robot_(robot), tolerance_(tolerance), bodyPairCollisions_(),
      weak_()
    {
      initializer_ = continuousValidation::Initializer::create();
      if (tolerance < 0) {
        throw std::runtime_error ("tolerance should be non-negative.");
      }
    }
  } // namespace core
} // namespace hpp
