//
// Copyright (c) 2005, 2006, 2007, 2008, 2009, 2010, 2011 CNRS
// Authors: Eiichi Yoshida
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

#ifndef HPP_CORE_COLLISION_PAIR_HH
#define HPP_CORE_COLLISION_PAIR_HH

#include <iostream>
#include <map>
#include <vector>
#include <utility>    // pair
#include <functional> // greater

#include <hpp/util/deprecated.hh>

namespace hpp {
  namespace core {

    /// \brief Generate collision checking between bodies of a robot
    class CollisionPair
    {
      
      typedef std::pair<unsigned int, std::vector<unsigned int> > pair_t;
      typedef std::map< unsigned int, std::vector<unsigned int>,
			std::less<unsigned int> > map_t; 
      
      /// \brief self collision table
      map_t collisionPairs_; 
      std::vector<unsigned int> emptyVector_;
    public:
      /// \brief Constructor
      CollisionPair();
      /// \brief Destructor
      ~CollisionPair();
      /// \brief Add a link joint j1 as collision-checking pair of j2
      bool addColPair(unsigned int j1, unsigned int j2);
      /// \brief Add collision pairs between joint j1 and joints [j2from:j2to]
      bool addColPairRange(unsigned int j1, unsigned int j2from,
			   unsigned int j2to);
      /// \brief check if there is already key j1
      bool existKey(unsigned int j1);
      /// \brief check if there is already that pair in the table
      bool existPair(unsigned int j1, unsigned int j2);
      /// \brief check if there is value j2 for key j1
      bool existPairNarrow(unsigned int j1, unsigned int j2);
      /// \brief get vector of collision pair for body of joint 1
      std::vector<unsigned int>& getColPairList(unsigned int j, bool& flag);
      /// \brief Write list of pairs in standard output
      void printPair();
      
    }; // class CollisionPair
  } // namespace core
} // namespace hpp
typedef hpp::core::CollisionPair ChppColPair HPP_DEPRECATED;
#endif // HPP_CORE_COLLISION_PAIR_HH
