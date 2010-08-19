#ifndef HPP_CORE_COLLISION_PAIR_HH
#define HPP_CORE_COLLISION_PAIR_HH

#warning "This header is deprecated."
#warning "Include <hpp/core/collision-pair.hh> instead."

#include <iostream>
#include <map>
#include <vector>
#include <utility>    // pair
#include <functional> // greater

/**
   \brief Collision pair generating class
*/
class ChppColPair { // friend?
  typedef std::pair<unsigned int, std::vector<unsigned int> > cPair;
  typedef std::map< unsigned int, std::vector<unsigned int>, std::less<unsigned int> > cMap; 

  std::vector<unsigned int> tmp;
  /**
     \brief self collision table
  */
  cMap mapCol; 

 public:
  /**
     \brief constructor
  */
  ChppColPair();

  /**
     \brief destructor
  */
  ~ChppColPair();

  /**
     \brief add a link jounsigned int j1 as collision-checking pair of j2
  */
  bool addColPair(unsigned int j1, unsigned int j2);

  /**
     \brief add a link for jounsigned int j1 as collision-checking pair from j2from of j2to
  */
  bool addColPairRange(unsigned int j1, unsigned int j2from, unsigned int j2to);

  /**
     \brief check if there is already key j1
  */
  bool existKey(unsigned int j1);

  /**
     \brief check if there is already that pair in the table
  */
  bool existPair(unsigned int j1, unsigned int j2);

  /**
     \brief check if there is value j2 for key j1
  */
  bool existPairNarrow(unsigned int j1, unsigned int j2);

  /**
     \brief get vector of collision pair for body of joint 1
  */
  std::vector<unsigned int>& getColPairList(unsigned int j, bool& flag);

  void printPair();

}; // this semicolon is very important

#endif // colPair.h
