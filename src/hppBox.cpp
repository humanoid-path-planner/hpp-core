/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)

*/

#include "hppBox.h"

ChppBoxShPtr ChppBox::create(const std::string &inName,
			     const double i_xSize, const double i_ySize, const double i_zSize)
{
  ChppBox *hppBox = new ChppBox();
  ChppBoxShPtr hppBoxShPtr(hppBox);

  hppBox->init(hppBoxShPtr, inName, i_xSize, i_ySize, i_zSize);
  return hppBoxShPtr;
}

void ChppBox::init(const ChppBoxWkPtr& inBoxWkPtr, const std::string &i_name,
		   const double i_xSize, const double i_ySize, const double i_zSize)
{
  CkppKCDBox::init(inBoxWkPtr, i_name, i_xSize, i_ySize, i_zSize);
}

// already implemented
/*
std::string ChppBox::name()
{
  return boxName;
}
*/
