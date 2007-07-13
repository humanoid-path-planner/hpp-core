#ifndef HPP_COLPAIR_H
#define HPP_COLPAIR_H

#include <iostream>
#include <map>
#include <vector>
#include <utility>    // pair
#include <functional> // greater

// using namespace std;

/// \brief Collision pair generating class
class ChppColPair { // friend?
  typedef std::pair<unsigned int, std::vector<unsigned int> > cPair;
  typedef std::map< unsigned int, std::vector<unsigned int>, std::less<unsigned int> > cMap; 

  unsigned int dof;
  std::vector<unsigned int> tmp;
  cMap mapCol; 
  // \brief self collision table

 public:
  ChppColPair(unsigned int tblDof);
  // \brief constructor with initialization with number of DOF

  ~ChppColPair();
  // \brief destructor

  bool addColPair(unsigned int j1, unsigned int j2);
  // \brief add a link jounsigned int j1 as collision-checking pair of j2

  bool addColPairRange(unsigned int j1, unsigned int j2from, unsigned int j2to);
  // \brief add a link for jounsigned int j1 as collision-checking pair from j2from of j2to

  bool existKey(unsigned int j1);
  // \brief check if there is already key j1

  bool existPair(unsigned int j1, unsigned int j2);
  // \brief check if there is already that pair in the table

  bool existPairNarrow(unsigned int j1, unsigned int j2);
  // \brief check if there is value j2 for key j1

  std::vector<unsigned int>& getColPairList(unsigned int j, bool& flag);
  // \brief get vector of collision pair for body of joint 1

  void printPair();

}; // this semicolon is very important

#endif // colPair.h
