//
// Copyright (c) 2014 CNRS
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

#define BOOST_TEST_MODULE intervals
#include "continuous-collision-checking/intervals.hh"
#include <boost/test/included/unit_test.hpp>

using hpp::core::interval_t;
using hpp::core::continuousCollisionChecking::Intervals;

bool checkIntervals (const Intervals& intervals)
{
  if (intervals.list ().empty ()) return true;
  std::list <interval_t>::const_iterator it = intervals.list ().begin ();
  std::list <interval_t>::const_iterator it1 = it;

  while (it1 != intervals.list ().end ()) {
    BOOST_CHECK (it1->first <= it1->second);
    ++it1;
    if (it1 != intervals.list ().end ()) {
      BOOST_CHECK (it->second < it1->first);
      it = it1;
    }
  }
  return true;
}

BOOST_AUTO_TEST_SUITE( test_hpp_core )
BOOST_AUTO_TEST_CASE (intervals_1)
{
  Intervals intervals;

  interval_t interval (4, 5);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  interval = interval_t (2, 3);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  interval = interval_t (6, 7);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  interval = interval_t (0, 1);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  BOOST_CHECK (intervals.list ().size () == 4);
  std::list <interval_t>::const_iterator it = intervals.list ().begin ();
  BOOST_CHECK_EQUAL (it->first, 0);
  BOOST_CHECK_EQUAL (it->second, 1);
  ++it;
  BOOST_CHECK_EQUAL (it->first, 2);
  BOOST_CHECK_EQUAL (it->second, 3);
  ++it;
  BOOST_CHECK_EQUAL (it->first, 4);
  BOOST_CHECK_EQUAL (it->second, 5);
  ++it;
  BOOST_CHECK_EQUAL (it->first, 6);
  BOOST_CHECK_EQUAL (it->second, 7);
}

BOOST_AUTO_TEST_CASE (intervals_2)
{
  Intervals intervals;

  interval_t interval (0, 1);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  interval = interval_t (2, 3);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  interval = interval_t (4, 5);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  interval = interval_t (6, 7);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  interval = interval_t (1.5, 3.5);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  BOOST_CHECK (intervals.list ().size () == 4);
  std::list <interval_t>::const_iterator it = intervals.list ().begin ();
  BOOST_CHECK_EQUAL (it->first, 0);
  BOOST_CHECK_EQUAL (it->second, 1);
  ++it;
  BOOST_CHECK_EQUAL (it->first, 1.5);
  BOOST_CHECK_EQUAL (it->second, 3.5);
  ++it;
  BOOST_CHECK_EQUAL (it->first, 4);
  BOOST_CHECK_EQUAL (it->second, 5);
  ++it;
  BOOST_CHECK_EQUAL (it->first, 6);
  BOOST_CHECK_EQUAL (it->second, 7);
}

BOOST_AUTO_TEST_CASE (intervals_3)
{
  Intervals intervals;

  interval_t interval (4, 5);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  interval = interval_t (2, 3);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  interval = interval_t (6, 7);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  interval = interval_t (0, 1);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  interval = interval_t (0.2, 0.3);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  BOOST_CHECK (intervals.list ().size () == 4);
  std::list <interval_t>::const_iterator it = intervals.list ().begin ();
  BOOST_CHECK_EQUAL (it->first, 0);
  BOOST_CHECK_EQUAL (it->second, 1);
  ++it;
  BOOST_CHECK_EQUAL (it->first, 2);
  BOOST_CHECK_EQUAL (it->second, 3);
  ++it;
  BOOST_CHECK_EQUAL (it->first, 4);
  BOOST_CHECK_EQUAL (it->second, 5);
  ++it;
  BOOST_CHECK_EQUAL (it->first, 6);
  BOOST_CHECK_EQUAL (it->second, 7);
}

BOOST_AUTO_TEST_CASE (intervals_4)
{
  Intervals intervals;

  interval_t interval (4, 5);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  interval = interval_t (2, 3);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  interval = interval_t (6, 7);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  interval = interval_t (0, 1);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  interval = interval_t (2.8, 3.2);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  BOOST_CHECK (intervals.list ().size () == 4);
  std::list <interval_t>::const_iterator it = intervals.list ().begin ();
  BOOST_CHECK_EQUAL (it->first, 0);
  BOOST_CHECK_EQUAL (it->second, 1);
  ++it;
  BOOST_CHECK_EQUAL (it->first, 2);
  BOOST_CHECK_EQUAL (it->second, 3.2);
  ++it;
  BOOST_CHECK_EQUAL (it->first, 4);
  BOOST_CHECK_EQUAL (it->second, 5);
  ++it;
  BOOST_CHECK_EQUAL (it->first, 6);
  BOOST_CHECK_EQUAL (it->second, 7);
}

BOOST_AUTO_TEST_CASE (intervals_5)
{
  Intervals intervals;

  interval_t interval (4, 5);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  interval = interval_t (2, 3);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  interval = interval_t (6, 7);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  interval = interval_t (0, 1);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  interval = interval_t (1.5, 6.5);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  BOOST_CHECK (intervals.list ().size () == 2);
  std::list <interval_t>::const_iterator it = intervals.list ().begin ();
  BOOST_CHECK_EQUAL (it->first, 0);
  BOOST_CHECK_EQUAL (it->second, 1);
  ++it;
  BOOST_CHECK_EQUAL (it->first, 1.5);
  BOOST_CHECK_EQUAL (it->second, 7);
}

BOOST_AUTO_TEST_CASE (interval_6)
{
  Intervals intervals;

  interval_t interval (0, 1);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  interval = interval_t (2, 3);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  interval = interval_t (1, 2);
  intervals.unionInterval (interval);
  checkIntervals (intervals);

  std::cout << intervals << std::endl;
}

BOOST_AUTO_TEST_CASE (interval_7)
{
  using hpp::core::value_type;
  Intervals intervals;
  interval_t interval (0, 0);

  for (unsigned int i=0; i<1000; ++i) {
    value_type t = (1.*rand ())/RAND_MAX;
    value_type l = .01*rand ()/RAND_MAX;

    interval.first = t-l;
    interval.second = t+l;
    intervals.unionInterval (interval);
    checkIntervals (intervals);
    BOOST_CHECK (intervals.contains (interval));
  }
}

BOOST_AUTO_TEST_SUITE_END()
