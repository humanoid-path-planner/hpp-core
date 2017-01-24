//
// Copyright (c) 2016 CNRS
// Authors: Florent Lamiraux
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

#define BOOST_TEST_MODULE path_extraction

#include <boost/test/included/unit_test.hpp>
#include <hpp/fcl/math/transform.h>

#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>

#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/steering-method-straight.hh>

BOOST_AUTO_TEST_SUITE( test_hpp_core )

using hpp::model::ConfigurationOut_t;
using hpp::model::Configuration_t;
using hpp::model::Device;
using hpp::model::DevicePtr_t;
using hpp::model::JointPtr_t;
using hpp::model::JointConstPtr_t;

using hpp::core::ConfigurationPtr_t;
using hpp::core::ConstraintSetPtr_t;
using hpp::core::interval_t;
using hpp::core::Path;
using hpp::core::PathPtr_t;
using hpp::core::PathVector;
using hpp::core::PathVectorPtr_t;
using hpp::core::Problem;
using hpp::core::ProblemPtr_t;
using hpp::core::SteeringMethodStraight;
using hpp::core::SteeringMethodStraightPtr_t;
using hpp::core::value_type;

HPP_PREDEF_CLASS (LocalPath);
typedef boost::shared_ptr <LocalPath> LocalPathPtr_t;

class LocalPath : public Path
{
public:
  virtual ~LocalPath () throw () {}
  virtual bool impl_compute
  (ConfigurationOut_t configuration, value_type t) const
  {
    value_type alpha (t-timeRange ().first/
		      (timeRange ().second - timeRange ().first));
    configuration = (1-alpha)*q1_ + alpha * q2_;
    return true;
  }

  static LocalPathPtr_t create (const interval_t& interval)
  {
    LocalPath* ptr (new LocalPath (interval));
    LocalPathPtr_t shPtr (ptr);
    ptr->init (shPtr);
    return shPtr;
  }

  static LocalPathPtr_t createCopy (const LocalPathPtr_t& other)
  {
    LocalPath* ptr (new LocalPath (*other));
    LocalPathPtr_t shPtr (ptr);
    ptr->init (shPtr);
    return shPtr;
  }

  virtual Configuration_t initial () const
  {
    return q1_;
  }

  virtual Configuration_t end () const
  {
    return q2_;
  }

  virtual PathPtr_t copy () const
  {
    return createCopy (weak_.lock ());
  }

  virtual PathPtr_t copy (const ConstraintSetPtr_t&) const
  {
    return createCopy (weak_.lock ());
  }

  virtual std::ostream& print (std::ostream &os) const
  {
    os << "LocalPath between " << q1_ [0] << " and " << q2_ [0];
    return os;
  }
protected:
  LocalPath (const interval_t& interval) : Path (interval, 1,1),
					   q1_ (1), q2_ (1)
  {
    q1_ [0] = interval.first;
    q2_ [0] = interval.second;
  }

  LocalPath (const LocalPath& other) :
    Path (other), q1_ (other.q1_), q2_ (other.q2_)
  {
  }

  void init (LocalPathWkPtr_t wkPtr)
  {
    Path::init (wkPtr.lock ());
    weak_ = wkPtr;
  }

private:
  Configuration_t q1_, q2_;
  LocalPathWkPtr_t weak_;
};

BOOST_AUTO_TEST_CASE (path_extraction_1)
{
  PathVectorPtr_t pv (PathVector::create (1, 1));
  // l0 = 0.52843097738664657
  interval_t inter0 (std::make_pair (0,0.52843097738664657));
  // L0 + l1 = 2.633158413063464
  interval_t inter1 (std::make_pair (0,2.1047274356768177));
  // l0 + l1 + l2 = 3.331454231080471
  interval_t inter2 (std::make_pair (0.6751734883957865,1.3734693064127934));
  // l0 + l1 + l2 + l3 = 4.079212904814735
  interval_t inter3 (std::make_pair (0.83075838111776035, 1.578517054852024));
  LocalPathPtr_t p0 (LocalPath::create (inter0));
  LocalPathPtr_t p1 (LocalPath::create (inter1));
  LocalPathPtr_t p2 (LocalPath::create (inter2));
  LocalPathPtr_t p3 (LocalPath::create (inter3));
  pv->appendPath (p0);
  pv->appendPath (p1);
  pv->appendPath (p2);
  pv->appendPath (p3);
  value_type tmin (3.7487655421722721), tmax (4.0792129048147352);
  PathPtr_t extracted (pv->extract (std::make_pair (tmin, tmax)));
  BOOST_CHECK (extracted->timeRange ().first == 0);
  BOOST_CHECK (fabs (extracted->timeRange ().second - (tmax - tmin)) <
	       std::numeric_limits <float>::epsilon ());
  BOOST_CHECK (fabs (extracted->length () - (tmax - tmin)) <
	       std::numeric_limits <float>::epsilon ());
}
BOOST_AUTO_TEST_SUITE_END()

