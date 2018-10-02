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

#include <iterator>
namespace hpp
{
namespace core
{
namespace
{
using continuousValidation::IntervalValidation;
using continuousValidation::IntervalValidationPtr_t;
using continuousValidation::IntervalValidations_t;
using continuousValidation::BodyPairCollisionPtr_t;
using continuousValidation::SolidSolidCollision;
using continuousValidation::BodyPairCollisions_t;

typedef std::pair<se3::JointIndex, se3::JointIndex> JointIndexPair_t;

struct JointIndexPairCompare_t
{
  bool operator()(const JointIndexPair_t &p0, const JointIndexPair_t &p1) const
  {
    if (p0.first < p1.first)
      return true;
    if (p0.first > p1.first)
      return false;
    return (p0.second < p1.second);
  }
};

// typedef std::set<JointIndexPair_t, JointIndexPairCompare_t> JointIndexPairSet_t;
typedef std::map<JointIndexPair_t, BodyPairCollisionPtr_t, JointIndexPairCompare_t> BodyPairCollisionMap_t;
} // namespace

/// Validate interval centered on a path parameter
/// \param config Configuration at abscissa tmin on the path.
/// \param tmin parameter value in the path interval of definition
/// \retval tmin lower bound of valid interval if reverve is true,
///              upper bound of valid interval if reverse is false.
///              other bound of valid interval is input tmin.
/// \return true if the configuration is collision free for this parameter
///         value, false otherwise.
bool ContinuousValidation::validateConfiguration(const Configuration_t &config, const value_type &t,
                                                        interval_t &interval, PathValidationReportPtr_t &report)
{
  interval.first = -std::numeric_limits<value_type>::infinity();
  interval.second = std::numeric_limits<value_type>::infinity();
  interval_t tmpInt;
  robot_->currentConfiguration(config);
  robot_->computeForwardKinematics();
  robot_->updateGeometryPlacements();
  bool itMinIsInCollisions = true;
  BodyPairCollisions_t::iterator itMin1 = bodyPairCollisions_.begin();
  IntervalValidations_t::iterator itMin2 = intervalValidations_.begin();
  value_type distance1 = 0;
  value_type distance2 = 0;
  if (checkCollisionFirst)
  {
    // check first for collision validations, then for other validations
    if (!validateIntervals<BodyPairCollisions_t, CollisionValidationReportPtr_t>
          (bodyPairCollisions_, t, interval, tmpInt, report, distance1))
      return false;
    value_type collisionUpperBound = interval.second;
    if (!validateIntervals<IntervalValidations_t, ValidationReportPtr_t>
          (intervalValidations_, t, interval, tmpInt, report, distance2))
      return false;
    itMinIsInCollisions = (interval.second >= collisionUpperBound);
  }
  else
  {
    // check first for validations other than collisions, then for collisions
    if (!validateIntervals<IntervalValidations_t, ValidationReportPtr_t>
          (intervalValidations_, t, interval, tmpInt, report, distance2))
      return false;
    value_type validationUpperBound = interval.second;
    if (!validateIntervals<BodyPairCollisions_t, CollisionValidationReportPtr_t>
          (bodyPairCollisions_, t, interval, tmpInt, report, distance1))
      return false;
    itMinIsInCollisions = (interval.second < validationUpperBound);
  }
  std::advance(itMin1, distance1);
  std::advance(itMin2, distance2);
  if (itMinIsInCollisions)
  {
    checkCollisionFirst = true;
    if (bodyPairCollisions_.size() > 1 && itMin1 != bodyPairCollisions_.begin())
      std::iter_swap(bodyPairCollisions_.begin(), itMin1);
  }
  else
  {
    checkCollisionFirst = false;
    if (intervalValidations_.size() > 1 && itMin2 != intervalValidations_.begin())
      std::iter_swap(intervalValidations_.begin(), itMin2);
  }
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
  bool testres = validateStraightPath(path, reverse, validPart, report);
  return testres;
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

void ContinuousValidation::setPath(const PathPtr_t &path, bool reverse)
{
  for (BodyPairCollisions_t::iterator itPair = bodyPairCollisions_.begin ();
	 itPair != bodyPairCollisions_.end (); ++itPair) {
	  (*itPair)->path (path, reverse);
	}
  for (IntervalValidations_t::iterator itVal = intervalValidations_.begin ();
	 itVal != intervalValidations_.end (); ++itVal) {
	  (*itVal)->path (path, reverse);
	}
}

void ContinuousValidation::removeObstacleFromJoint(const JointPtr_t &joint, const CollisionObjectConstPtr_t &obstacle)
{
  bool removed = false;
  for (BodyPairCollisions_t::iterator itPair =
           bodyPairCollisions_.begin();
       itPair != bodyPairCollisions_.end(); ++itPair)
  {
    if (!(*itPair)->joint_b() && (*itPair)->joint_a() == joint)
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
    const JointConstPtr_t &ja = (*_colPair)->joint_a(),
                          jb = (*_colPair)->joint_b();
    ia = RelativeMotion::idx(ja);
    ib = RelativeMotion::idx(jb);
    switch (relMotion(ia, ib))
    {
    case RelativeMotion::Parameterized:
      hppDout(info, "Parameterized collision pairs treated as Constrained");
    case RelativeMotion::Constrained:
      hppDout(info, "Disabling collision between joint "
                        << (ja ? ja->name() : "universe") << " and "
                        << (jb ? jb->name() : "universe"));
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

void ContinuousValidation::generateAutoCollisions()
{
  // Build body pairs for validation
  // First auto-collision
  const se3::GeometryModel &gmodel = robot_->geomModel();
  JointPtr_t joint1, joint2;
  BodyPairCollisionMap_t bodyPairMap;
  for (std::size_t i = 0; i < gmodel.collisionPairs.size(); ++i)
  {
    const se3::CollisionPair &cp = gmodel.collisionPairs[i];
    JointIndexPair_t jp(gmodel.geometryObjects[cp.first].parentJoint,
                        gmodel.geometryObjects[cp.second].parentJoint);

    // Ignore pairs of bodies that are in the same joint.
    if (jp.first == jp.second)
      continue;

    BodyPairCollisionMap_t::iterator _bp = bodyPairMap.find(jp);

    if (_bp == bodyPairMap.end())
    {
      joint1 = JointPtr_t(new Joint(robot_, jp.first));
      joint2 = JointPtr_t(new Joint(robot_, jp.second));
      bodyPairCollisions_.push_back(
          SolidSolidCollision::create(joint2, joint1, tolerance_));
      bodyPairMap[jp] = bodyPairCollisions_.back();
    }
    bodyPairMap[jp]->addCollisionPair(
        CollisionObjectConstPtr_t(new pinocchio::CollisionObject(robot_, cp.first)),
        CollisionObjectConstPtr_t(new pinocchio::CollisionObject(robot_, cp.second)));
  }
}

ContinuousValidation::~ContinuousValidation()
{
}

ContinuousValidation::ContinuousValidation(const DevicePtr_t &robot, const value_type &tolerance):
  robot_(robot), tolerance_(tolerance), bodyPairCollisions_(), intervalValidations_(), checkCollisionFirst(true)
{
  generateAutoCollisions();
}
} // namespace core
} // namespace hpp
