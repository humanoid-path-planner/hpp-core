/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS) Eiichi Yoshida (JRL/LAAS-CNRS/AIST)

*/

#ifndef HPPBOX_H_
#define HPPBOX_H_



/*************************************
INCLUDE
**************************************/

#include "KineoKCDModel/kppKCDBox.h"
//#include "KineoWorks2/kwsInterface.h"
// #include "kitInterface.h"
// #include "kcd2/kcdInterface.h"
// #include "kwsKcd2/kwsKCDBody.h"
// #include "hppNotification.h"


KIT_PREDEF_CLASS(ChppBox);
/**
 \brief This class represents polyhedra. It derives from KCD CkcdBox class. 
 In order to store more information, we have derived CkcdBox class. 
 A name is given at the creation of the polyhedron (ChppBox::create(std::string inName)). 
 The constructor is protected and method create returns a shared pointer to the device.
\sa Smart pointers documentation: http://www.boost.org/libs/smart_ptr/smart_ptr.htm 
*/
class ChppBox : public CkppKCDBox {
public: 
  // already implemented   std::string name();
  static ChppBoxShPtr create(const std::string &inName,
			     const double i_xSize, const double i_ySize, const double i_zSize);
protected:
  ChppBox()  {};
    void init(const ChppBoxWkPtr& inBoxWkPtr, const std::string &i_name,
	      const double i_xSize, const double i_ySize, const double i_zSize);
    //private:
    // already implemented  std::string boxName;
};


#endif /*HPPBOX_H_*/
