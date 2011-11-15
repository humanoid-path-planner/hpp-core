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


#include <algorithm>
#include "hpp/core/collision-pair.hh"

namespace hpp {
  namespace core {

    ChppColPair::ChppColPair()
    {
      collisionPairs_.clear();
      emptyVector_.clear();
    }

    ChppColPair::~ChppColPair()
    {
    }

    bool ChppColPair::addColPair(unsigned int j1, unsigned int j2)
    {
      if(!existPair(j1, j2)){

	if(!existKey(j1)){
	  std::vector <unsigned int> v;

	  // std::cout<<"adding new key "<<j1<<"... "<<std::endl;
	  v.clear();
	  v.push_back(j2);
	  collisionPairs_.insert(pair_t(j1, v));
	}
    
	else{
	  bool flag;
	  std::vector <unsigned int> &v = getColPairList(j1, flag);
      
	  v.push_back(j2);
	  sort(v.begin(), v.end());
	}
	return true;
      }
      return false;
    }

    bool ChppColPair::addColPairRange(unsigned int j1, unsigned int j2from, unsigned int j2to)
    {
      if(j2from > j2to){
	std::cout << " addColpairRange: j2from " << j2from 
		  << " must be smaller than "<< j2to <<std::endl;
	return false;
      }

      for(unsigned int j = j2from; j<= j2to; j++) {
	addColPair(j1, j);
      }
      return true;
    }

    bool ChppColPair::existKey(unsigned int j1)
    {
      map_t::iterator tbl;

      tbl = collisionPairs_.find(j1);

      return tbl != collisionPairs_.end();
    }

    bool ChppColPair::existPair(unsigned int j1, unsigned int j2)
    {

      if(existPairNarrow(j1, j2)){
	// std::cout<<j2<<" is already in the list of "<<j1<<std::endl;
	return true;
      }
      /*
	else
	std::cout<<"key "<<j1<<" not found"<<std::endl;
      */

      if(existPairNarrow(j2, j1)){
	// std::cout<<j1<<" is already in the list of "<<j2<<std::endl;
	return true;
      }
      /*
	else
	std::cout<<"key "<<j1<<" not found"<<std::endl;
      */

      return false;
    }

    bool ChppColPair::existPairNarrow(unsigned int j1, unsigned int j2)
    {
      map_t::iterator tbl;

      tbl = collisionPairs_.find(j1);
      if(tbl != collisionPairs_.end()){
	std::vector<unsigned int> v = collisionPairs_[j1];

	if(count(v.begin(), v.end(), j2) > 0){
	  return true;
	}
      }

      return false;
    }

    std::vector<unsigned int>& ChppColPair::
    getColPairList(unsigned int j, bool& flag)
    {
      if(existKey(j)){
	flag = true;
	return collisionPairs_[j];
      }

      else{
	flag =  false;
	return emptyVector_;
      }
    }


    void ChppColPair::printPair()
    {
      std::cout<<" table size "<<collisionPairs_.size()<<std::endl;
      map_t::iterator it = collisionPairs_.begin();
      bool flag;

      unsigned int current_key;
      for(; it != collisionPairs_.end(); it++) {
	current_key = it->first;
	std::cout<<" for j "<<current_key<<":";

	std::vector<unsigned int> v;
	v = getColPairList(current_key, flag);
	for(unsigned int j=0; j<v.size(); j++)
	  std::cout<<" "<<v[j];
	std::cout<<std::endl;
      }
    }

  } // namespace core
} // namespace hpp
