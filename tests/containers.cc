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

#define BOOST_TEST_MODULE containers
#include <boost/test/included/unit_test.hpp>

#include <hpp/core/container.hh>

typedef std::string Type1;
typedef boost::shared_ptr<std::string> Type2;

using hpp::core::Container;

BOOST_AUTO_TEST_SUITE( test_hpp_core_container )

BOOST_AUTO_TEST_CASE (containers) {
  Container<Type1> c1;
  Container<Type2> c2;
  c1.add ("key1", Type1 ("type1_1"));
  c2.add ("key1", Type2 (new std::string ("type2_1")));

  c1.print (std::cout);
  c2.print (std::cout);

  c2.add ("key1", Type2 (new std::string ("type2_2")));
  c2.print (std::cout);
}

BOOST_AUTO_TEST_SUITE_END()



