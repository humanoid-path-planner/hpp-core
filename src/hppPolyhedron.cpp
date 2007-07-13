/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)

*/

#include "hppPolyhedron.h"
#include "kcd2/kcdPolyhedron.h"

ChppPolyhedronShPtr ChppPolyhedron::create(const std::string &inName)
{
  ChppPolyhedron *hppPolyhedron = new ChppPolyhedron();
  ChppPolyhedronShPtr hppPolyhedronShPtr(hppPolyhedron);

  hppPolyhedron->init(hppPolyhedronShPtr, inName);
  return hppPolyhedronShPtr;
}

void ChppPolyhedron::init(const ChppPolyhedronWkPtr& inPolyhedronWkPtr, const std::string &i_name)
{
  CkppKCDPolyhedron::init(inPolyhedronWkPtr, i_name);
}


void ChppPolyhedron::addPoint (const kcdReal i_x, const kcdReal i_y, const kcdReal i_z, unsigned int &o_rank)
{
  CkcdPolyhedron::addPoint(i_x, i_y, i_z, o_rank);
}

/* already implemented
std::string ChppPolyhedron::name()
{
  return polyhedronName;
}
*/
