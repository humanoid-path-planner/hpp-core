#include "hppColPair.h"

#include <algorithm>

// using namespace std;

ChppColPair::ChppColPair()
{
  tmp.clear();
  mapCol.clear();
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
      mapCol.insert(cPair(j1, v));
    }
    
    else{
      bool flag;
      std::vector <unsigned int> &v = getColPairList(j1, flag);
      
      v.push_back(j2);
      sort(v.begin(), v.end());
    }
  }
}

bool ChppColPair::addColPairRange(unsigned int j1, unsigned int j2from, unsigned int j2to)
{
  if(j2from > j2to){
    std::cout<<" addColpairRange: j2from "<<j2from<<" must be smaller than "<<j2to<<std::endl;
    return false;
  }

  for(int j = j2from; j<= j2to; j++){
    addColPair(j1, j);
  }
}

bool ChppColPair::existKey(unsigned int j1)
{
  cMap::iterator tbl;

  tbl = mapCol.find(j1);

  return tbl != mapCol.end();
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
  cMap::iterator tbl;

  tbl = mapCol.find(j1);
  if(tbl != mapCol.end()){
    std::vector<unsigned int> v = mapCol[j1];

    if(count(v.begin(), v.end(), j2) > 0){
      return true;
    }
  }

  return false;
}

// bool ChppColPair::getColPairList(unsigned int j, std::vector<unsigned int>& v)
std::vector<unsigned int>& ChppColPair::getColPairList(unsigned int j, bool& flag)
{
  if(existKey(j)){
    flag = true;
    return mapCol[j];
  }

  else{
    flag =  false;
    return tmp;
  }
}


void ChppColPair::printPair()
{
  std::cout<<" table size "<<mapCol.size()<<std::endl;
  cMap::iterator it = mapCol.begin();
  bool flag;

  unsigned int current_key;
  for(; it != mapCol.end(); it++) {
    current_key = it->first;
    std::cout<<" for j "<<current_key<<":";

    std::vector<unsigned int> v;
    v = getColPairList(current_key, flag);
    for(int j=0; j<v.size(); j++)
      std::cout<<" "<<v[j];
    std::cout<<std::endl;
  }
}


