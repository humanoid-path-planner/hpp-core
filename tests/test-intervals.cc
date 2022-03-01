//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#define BOOST_TEST_MODULE intervals
#include "continuous-validation/intervals.hh"
#include <boost/test/included/unit_test.hpp>

using hpp::core::interval_t;
using hpp::core::continuousValidation::Intervals;

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

  BOOST_TEST_MESSAGE(intervals);
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
