/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)
           and Eiichi Yoshida (ISRI-AIST)
*/

#ifndef HPP_DEVICE_H
#define HPP_DEVICE_H


/*************************************
INCLUDE
**************************************/

#include "KineoWorks2/kwsInterface.h"
#include "KineoUtility/kitInterface.h"
#include "KineoModel/kppDeviceComponent.h"

#include "kcd2/kcdInterface.h"
#include "kwsKcd2/kwsKCDBody.h"

KIT_PREDEF_CLASS(ChppDevice);
/**
 \brief This class represents devices (robots). It derives from KineoWorks CkwsDevice class. 
 In order to give a name to each robot, we have derived CkwsDevice class. The name is given at the creation of the device (ChppDevice::create()). The constructor is private and method create returns a shared pointer to the device.
\sa Smart pointers documentation: http://www.boost.org/libs/smart_ptr/smart_ptr.htm 
*/
class ChppDevice : public CkppDeviceComponent {
public: 


  virtual ~ChppDevice();

  static ChppDeviceShPtr create(const std::string inName);
  static ChppDeviceShPtr	createCopy(const ChppDeviceShPtr& i_deviceComponent);

  virtual CkwsDeviceShPtr	clone() const;
  virtual CkppComponentShPtr	cloneComponent() const;
  virtual bool			isComponentClonable() const;

  /**
     \brief provide the total mass of the robot.
   */
  void updateTotalMass();
  double totalMass() const;
  
  /**
     \brief Compute the position of the center of mass of the robot in current configuration.
     \todo Eiichi: write the function.
  */
  CkitPoint3 computeCenterOfMass();

  /**
     \brief Get a shared pointer on the joint corresponding to the robot gaze if any.
  */
  const CkwsJointShPtr& gazeJoint() const;

  /**
     \brief Set the joint corresponding to the robot gaze.
  */
  void gazeJoint(CkwsJointShPtr& inGazeJoint);

  /** 
      \brief Get initial gaze direction (in zero-configuration)
  */
  const CkitVect3& initGazeDir() const;

  /** 
      \brief Set initial gaze direction (in zero-configuration)
  */
  void initGazeDir(const CkitVect3& inInitGazeDir);


  /**
     \brief Compute the bounding box of the robot in current configuration.
  */
  ktStatus axisAlignedBoundingBox (double& xMin, double& yMin, double& zMin,
				  double& xMax, double& yMax, double& zMax) const;

  
  /**
     \brief put the device give in parameter in the IgnoredOuterObject of the hppDevice (this)
     \param deviceIgnored : the device to be ignored
  */
  ktStatus ignoreDeviceForCollision (ChppDeviceShPtr deviceIgnored ) ;

  /**
   * \brief Add obstacle to the list.
   * \param inObject a new object.
   * \note Compute collision entities.
   */
  ktStatus addObstacle(const CkcdObjectShPtr& inObject);


protected:
  /**
     \brief Joint corresponding to the robot gaze (head for HRP-2)
  */
  CkwsJointShPtr attGazeJoint;

  /**
     \brief Gaze direction in zero-configuration.
  */
  CkitVect3 attInitGazeDir;

protected:
  /**
     \brief Initialization.
  */

  ktStatus init( const ChppDeviceWkPtr& i_weakPtr, const std::string& i_name);

  /**
     \brief Initialization with shared pointer.
  */

  ktStatus init( const ChppDeviceWkPtr& i_weakPtr, const ChppDeviceShPtr& i_device);

  /**
     \brief Constructor.
  */

  ChppDevice();

  /**
     \brief Copy constructor.
  */

  ChppDevice(const ChppDevice& i_device);

private:

  /**
     \brief Name of the device.
  */
  // std::string deviceName;
  ChppDeviceWkPtr m_weakPtr;

  /**
     \brief Total Mass.
  */
  double _totalMass;

  void cksBodyBoundingBox(const CkwsKCDBodyShPtr& body, double& xMin, double& yMin, 
                               double& zMin, double& xMax, double& yMax, double& zMax) const;

  void ckcdObjectBoundingBox(const CkcdObjectShPtr& object, double& xMin, double& yMin, 
                               double& zMin, double& xMax, double& yMax, double& zMax) const;

};


#endif
