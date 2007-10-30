/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS) Eiichi Yoshida (JRL/LAAS-CNRS/AIST)

*/

#ifndef HPPPOLYHEDRON_H_
#define HPPPOLYHEDRON_H_



/*************************************
INCLUDE
**************************************/

#include "KineoKCDModel/kppKCDPolyhedron.h"
//#include "KineoWorks2/kwsInterface.h"
// #include "kitInterface.h"
// #include "kcd2/kcdInterface.h"
// #include "kwsKcd2/kwsKCDBody.h"
// #include "hppNotification.h"


KIT_PREDEF_CLASS(ChppPolyhedron);
/**
 \brief This class represents polyhedra. It derives from KCD CkcdPolyhedron class. 
 In order to store more information, we have derived CkcdPolyhedron class. 
 A name is given at the creation of the polyhedron (ChppPolyhedron::create(std::string inName)). 
 The constructor is protected and method create returns a shared pointer to the device.
\sa Smart pointers documentation: http://www.boost.org/libs/smart_ptr/smart_ptr.htm 
*/
class ChppPolyhedron : public CkppKCDPolyhedron {
public: 
  //already implemented   std::string name();
  static ChppPolyhedronShPtr create(const std::string &inName);

  void addPoint (const kcdReal i_x, const kcdReal i_y, const kcdReal i_z, unsigned int &o_rank);

protected:
  ChppPolyhedron()  {};
    void init(const ChppPolyhedronWkPtr& inPolyhedronShPtr, const std::string &i_name);

    // private:
    // already implemented std::string polyhedronName;

};


#endif /*HPPPOLYHEDRON_H_*/
