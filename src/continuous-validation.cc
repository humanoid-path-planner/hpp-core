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
#include <hpp/core/continuous-validation/solid-solid-collision.hh>

#include <iterator>
namespace hpp {
  namespace core {
    using continuousValidation::IntervalValidations_t;
    using continuousValidation::SolidSolidCollision;
    using continuousValidation::BodyPairCollisionPtr_t;
    using continuousValidation::BodyPairCollision;

    ContinuousValidation::Initialize::Initialize
    (ContinuousValidation& owner):
      owner_(owner)
    {
    }

    typedef std::pair<pinocchio::JointIndex, pinocchio::JointIndex>
    JointIndexPair_t;
    struct JointIndexPairCompare_t
    {
      bool operator()(const JointIndexPair_t &p0, const JointIndexPair_t &p1)
        const
      {
        if (p0.first < p1.first)
          return true;
        if (p0.first > p1.first)
          return false;
        return (p0.second < p1.second);
      }
    };
    typedef std::map <JointIndexPair_t, BodyPairCollisionPtr_t,
                      JointIndexPairCompare_t> BodyPairCollisionMap_t;

    void ContinuousValidation::Initialize::doExecute() const
    {
      DevicePtr_t robot = owner_.robot();
      const pinocchio::GeomModel &gmodel = robot->geomModel();
      const pinocchio::GeomData  &gdata  = robot->geomData();
      JointPtr_t joint1, joint2;
      BodyPairCollisionMap_t bodyPairMap;
      for (std::size_t i = 0; i < gmodel.collisionPairs.size(); ++i)
      {
        if (!gdata.activeCollisionPairs[i]) continue;

        const ::pinocchio::CollisionPair &cp = gmodel.collisionPairs[i];
        JointIndexPair_t jp(gmodel.geometryObjects[cp.first].parentJoint,
                            gmodel.geometryObjects[cp.second].parentJoint);

        // Ignore pairs of bodies that are in the same joint.
        if (jp.first == jp.second)
          continue;

        BodyPairCollisionMap_t::iterator _bp = bodyPairMap.find(jp);

        if (_bp == bodyPairMap.end())
        {
          joint1 = Joint::create (robot, jp.first );
          joint2 = Joint::create (robot, jp.second);
          if (!joint2) joint2.swap (joint1);
          assert(joint2);
          continuousValidation::SolidSolidCollisionPtr_t ss
            (SolidSolidCollision::create(joint2, joint1, owner_.tolerance_));
          owner_.addIntervalValidation(ss);
          bodyPairMap[jp] = ss;
        }
        CollisionObjectConstPtr_t co1
          (new pinocchio::CollisionObject(robot, cp.first));
        CollisionObjectConstPtr_t co2
          (new pinocchio::CollisionObject(robot, cp.second));
        bodyPairMap[jp]->addCollisionPair( co1,co2 );
      }
    }

    ContinuousValidation::AddObstacle::AddObstacle
    (ContinuousValidation& owner):
      owner_(owner), robot_ (owner.robot())
    {
    }

    void ContinuousValidation::AddObstacle::doExecute
      (const CollisionObjectConstPtr_t &object) const
    {
      DevicePtr_t robot(robot_.lock());
      for (size_type idx = 0; idx < robot->nbJoints(); ++idx) {
        JointPtr_t joint = robot->jointAt (idx);
      BodyPtr_t body = joint->linkedBody ();
        if (body)
        {
          ConstObjectStdVector_t objects;
          objects.push_back(object);
          owner_.addIntervalValidation
            (SolidSolidCollision::create (joint, objects,
                                          owner_.tolerance()));
        }
      }
    }

    /// Validate interval centered on a path parameter
    /// \param intervalValidations a reference to the pair with smallest interval.
    /// \param config Configuration at abscissa tmin on the path.
    /// \param t parameter value in the path interval of definition
    /// \retval interval interval validated for all validation elements
    /// \retval report reason why the interval is not valid,
    /// \return true if the configuration is collision free for this parameter
    ///         value, false otherwise.
    bool ContinuousValidation::validateConfiguration
    (IntervalValidations_t& intervalValidations,
        const Configuration_t &config, const value_type &t,
        interval_t &interval, PathValidationReportPtr_t &report)
    {
      interval.first = -std::numeric_limits <value_type>::infinity ();
      interval.second = std::numeric_limits <value_type>::infinity ();
      hpp::pinocchio::DeviceSync robot (robot_);
      robot.currentConfiguration (config);
      robot.computeForwardKinematics();
      robot.updateGeometryPlacements();
      IntervalValidations_t::iterator smallestInterval
        (intervalValidations.begin());
      if (!validateIntervals
            (intervalValidations, t, interval, report,
             smallestInterval, robot.d()))
        return false;
      // Put the smallest interval first so that, at next iteration,
      // collision pairs with large interval are not computed.
      if (intervalValidations.size() > 1 &&
          smallestInterval != intervalValidations.begin())
        std::iter_swap (intervalValidations.begin(), smallestInterval);
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
      // Copy list of BodyPairCollision instances in a pool for thread safety.
      IntervalValidations_t* bpc;
      if (!bodyPairCollisionPool_.available()) {
        // Add an element
        bpc = new IntervalValidations_t(intervalValidations_.size());
        for (std::size_t i = 0; i < bpc->size(); ++i)
          (*bpc)[i] = intervalValidations_[i]->copy();
        bodyPairCollisionPool_.push_back (bpc);
      }
      bpc = bodyPairCollisionPool_.acquire();
      bool ret = validateStraightPath(*bpc, path, reverse, validPart, report);
      bodyPairCollisionPool_.release (bpc);
      return ret;
    }

    void ContinuousValidation::addObstacle(const CollisionObjectConstPtr_t &object)
    {
      for(std::vector<AddObstacle>::const_iterator it(addObstacle_.begin());
          it != addObstacle_.end(); ++it) {
        it->doExecute(object);
      }
    }

    void ContinuousValidation::setPath
    (IntervalValidations_t& intervalValidations,
        const PathPtr_t &path, bool reverse)
    {
      for (IntervalValidations_t::iterator itPair(intervalValidations.begin ());
      itPair != intervalValidations.end (); ++itPair) {
        (*itPair)->path (path, reverse);
      }
    }

    void ContinuousValidation::removeObstacleFromJoint(const JointPtr_t &joint, const CollisionObjectConstPtr_t &obstacle)
    {
      assert (joint);
      bool removed = false;
      for (IntervalValidations_t::iterator itPair(intervalValidations_.begin());
          itPair != intervalValidations_.end(); ++itPair)
      {
        BodyPairCollisionPtr_t bpc(HPP_DYNAMIC_PTR_CAST(BodyPairCollision,
                                                        *itPair));
        if (!bpc) continue;
        // If jointA == joint and jointB is the root joint.
        if (bpc->indexJointA() == (size_type)joint->index()
            && bpc->indexJointB() == 0)
        {
          if (bpc->removeObjectTo_b(obstacle))
          {
            removed = true;
            if (bpc->pairs().empty())
            {
              intervalValidations_.erase(itPair);
              bodyPairCollisionPool_.clear();
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
      for (IntervalValidations_t::iterator _colPair
             (intervalValidations_.begin());
          _colPair != intervalValidations_.end();)
      {
        BodyPairCollisionPtr_t bpc(HPP_DYNAMIC_PTR_CAST(BodyPairCollision,
                                                        *_colPair));
        if (!bpc) continue;
        ia = bpc->indexJointA ();
        ib = bpc->indexJointB ();
        if (ia < 0 || ib < 0) {
          ++_colPair;
          continue;
        }
        switch (relMotion(ia, ib))
        {
        case RelativeMotion::Parameterized:
          hppDout(info, "Parameterized collision pairs treated as Constrained");
        case RelativeMotion::Constrained:
          hppDout(info, "Disabling collision pair " << **_colPair);
          disabledBodyPairCollisions_.push_back(bpc);
          _colPair = intervalValidations_.erase(_colPair);
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
      bodyPairCollisionPool_.clear();
    }

    void ContinuousValidation::setSecurityMargins
    (const matrix_t& securityMatrix)
    {
      // Loop over collision pairs and remove disabled ones.
      size_type ia, ib;
      for (IntervalValidations_t::iterator _colPair
             (intervalValidations_.begin());
          _colPair != intervalValidations_.end(); ++_colPair)
      {
        BodyPairCollisionPtr_t bpc(HPP_DYNAMIC_PTR_CAST(BodyPairCollision,
                                                        *_colPair));
        if (!bpc) continue;
        ia = bpc->indexJointA ();
        ib = bpc->indexJointB ();
        value_type margin(securityMatrix(ia, ib));
        bpc->securityMargin(margin);
      }
      bodyPairCollisionPool_.clear();
    }

    template <>
    void ContinuousValidation::add<ContinuousValidation::AddObstacle>
    (const AddObstacle& delegate)
    {
      addObstacle_.push_back(delegate);
    }

    template <>
    void ContinuousValidation::reset<ContinuousValidation::AddObstacle>()
    {
      addObstacle_.clear();
    }

    template <>
    void ContinuousValidation::add<ContinuousValidation::Initialize>
    (const Initialize& delegate)
    {
      initialize_.push_back(delegate);
    }

    template <>
    void ContinuousValidation::reset<ContinuousValidation::Initialize>()
    {
      initialize_.clear();
    }

    void ContinuousValidation::init (ContinuousValidationWkPtr_t weak)
    {
      weak_ = weak;
    }

    void ContinuousValidation::addIntervalValidation
    (const IntervalValidationPtr_t& intervalValidation)
    {
      intervalValidations_.push_back(intervalValidation);
      bodyPairCollisionPool_.clear();
    }

    void ContinuousValidation::initialize()
    {
      for(std::vector<Initialize>::const_iterator it(initialize_.begin());
          it != initialize_.end(); ++it) {
        it->doExecute();
      }
    }

    ContinuousValidation::~ContinuousValidation()
    {
    }

    ContinuousValidation::ContinuousValidation(const DevicePtr_t &robot, const value_type &tolerance):
      robot_(robot), tolerance_(tolerance), intervalValidations_(),
      weak_()
    {
      if (tolerance < 0) {
        throw std::runtime_error ("tolerance should be non-negative.");
      }
      add<Initialize>(Initialize(*this));
      add<AddObstacle>(AddObstacle(*this));
    }
  } // namespace core
} // namespace hpp
