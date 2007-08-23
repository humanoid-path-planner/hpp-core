/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)
           and Eiichi Yoshida (ISRI-AIST)

*/

#include <iostream>
#include "hppDevice.h"
#include "hppBody.h"


// ==========================================================================

ChppDevice::ChppDevice() :  _totalMass(0.0)
{
  /**
     \brief Constructor of an empty robot with a given name.
  */
  attGazeJoint.reset();
}

// ==========================================================================

ChppDevice::ChppDevice(const ChppDevice& i_device) :
CkppDeviceComponent( i_device )
{
	// no op
}


ChppDevice::~ChppDevice()
{
	// no op
}

// ==========================================================================


ChppDeviceShPtr ChppDevice::create(std::string inName)
{
  ChppDevice *hppDevice = new ChppDevice();
  ChppDeviceShPtr hppDeviceShPtr(hppDevice);

  if (hppDevice->init(hppDeviceShPtr, inName) != KD_OK) {
    hppDeviceShPtr.reset();
  }

  return hppDeviceShPtr;
}

// ==========================================================================

ChppDeviceShPtr ChppDevice::createCopy(const ChppDeviceShPtr& i_device)
{
	ChppDevice* ptr = new ChppDevice(*i_device);
	ChppDeviceShPtr	shPtr(ptr);

	if(KD_OK != ptr->init(shPtr, i_device))
	{
		shPtr.reset();
	}

	return shPtr;
}

// ==========================================================================

CkwsDeviceShPtr ChppDevice::clone() const
{
	return ChppDevice::createCopy(m_weakPtr.lock());
}

// ==========================================================================

CkppComponentShPtr ChppDevice::cloneComponent() const
{
	return ChppDevice::createCopy(m_weakPtr.lock());
}

// ==========================================================================

bool ChppDevice::isComponentClonable() const
{
	return true;
}

// ==========================================================================

ktStatus ChppDevice::init(const ChppDeviceWkPtr& inDevWkPtr, const std::string &i_name)
{
  
  
  ktStatus    success = CkppDeviceComponent::init(inDevWkPtr, i_name);
  
  if(KD_OK == success)
  {  
     m_weakPtr = inDevWkPtr;
  }

  return success;
}

// ==========================================================================

ktStatus ChppDevice::init(const ChppDeviceWkPtr& i_weakPtr,
				     const ChppDeviceShPtr& i_device)
{
  ktStatus  success = CkppDeviceComponent::init(i_weakPtr, i_device);

  if(KD_OK == success)
    {
      m_weakPtr = i_weakPtr;
    }
  
  return success;
}

// ==========================================================================

/* 
std::string ChppDevice::name()
{
  return deviceName;
}
*/

// ==========================================================================

void ChppDevice::updateTotalMass()
{
  TBodyVector bodyVec;
  getBodyVector (bodyVec);
  _totalMass = 0.0;
  for(TBodyConstIterator iter = bodyVec.begin(); iter != bodyVec.end(); iter++){
    // dynamic cast
    ChppBodyShPtr hppBody = KIT_DYNAMIC_PTR_CAST(ChppBody, *iter);
    _totalMass += hppBody->mass();
  }
}

// ==========================================================================

double ChppDevice::totalMass() const
{
  return _totalMass;
}

// ==========================================================================

CkitPoint3 ChppDevice::computeCenterOfMass()
{
  CkitPoint3 centerOfMass(0, 0, 0);

  TBodyVector bodyVec;
  getBodyVector (bodyVec);
  for(TBodyConstIterator iter = bodyVec.begin(); iter != bodyVec.end(); iter++){
    // dynamic cast
    ChppBodyShPtr hppBody = KIT_DYNAMIC_PTR_CAST(ChppBody, *iter);
    CkitPoint3 currentComPos;
    if(hppBody->currentComPos(currentComPos) == KD_ERROR){
      std::cerr<<"ChppDevice::computeCenterOfMass(): error in hppBody::currentComPos() of"
	       <<hppBody->name()<<std::endl;
    }
      
    centerOfMass = centerOfMass + currentComPos * hppBody->mass();
  }

  return centerOfMass / _totalMass;
}


// ==========================================================================
  
const CkwsJointShPtr& ChppDevice::gazeJoint() const
{
  return attGazeJoint;
}

// ==========================================================================
 
void ChppDevice::gazeJoint(CkwsJointShPtr& inGazeJoint)
{
  attGazeJoint = inGazeJoint;
}

// ==========================================================================

const CkitVect3& ChppDevice::initGazeDir() const
{
  return attInitGazeDir;
}

// ==========================================================================

void ChppDevice::initGazeDir(const CkitVect3& inInitGazeDir)
{
  attInitGazeDir = inInitGazeDir;
}


// ==========================================================================

ktStatus ChppDevice::axisAlignedBoundingBox (double& xMin, double& yMin, double& zMin,
                                double& xMax, double& yMax, double& zMax) const
{

  TBodyVector bodyVector;
  this->getBodyVector(bodyVector);
  unsigned int j=bodyVector.size();
 
  xMin=9999999;
  yMin=9999999;
  zMin=9999999;
  xMax=-9999999;
  yMax=-9999999;
  zMax=-9999999;

  for(unsigned int i=0; i<j; i++)
  {
    /*Dynamic cast*/
    CkwsKCDBodyShPtr a;
    a=KIT_DYNAMIC_PTR_CAST(CkwsKCDBody, bodyVector[i]);
    if(!a)
    {
      std::cerr << "Error in axisAlignedBoundingBox, the CkwsBody not of subtype CkwsKCDBody" <<std::endl;
      return KD_ERROR;
    }
    cksBodyBoundingBox(a,xMin,yMin,zMin,xMax,yMax,zMax);
  }
  return KD_OK;
}

// ==========================================================================

ktStatus ChppDevice::ignoreDeviceForCollision (ChppDeviceShPtr deviceIgnored ) {
   ktStatus status = KD_OK ;

  std::vector< CkcdObjectShPtr > ignoredOuterList;

  //
  // build the ignored list from the deviceIgnored
  //
  CkwsDevice::TBodyVector deviceIgnoredBodyVector;
  deviceIgnored->getBodyVector(deviceIgnoredBodyVector) ;
   

  for (unsigned int bodyIter = 0 ;  bodyIter < deviceIgnoredBodyVector.size(); bodyIter++)  {
      
    CkwsKCDBodyShPtr kcdBody;

    if (kcdBody = KIT_DYNAMIC_PTR_CAST(CkwsKCDBody,deviceIgnoredBodyVector[bodyIter])) {

      std::vector< CkcdObjectShPtr > kcdBodyInnerObjects = kcdBody->innerObjects() ;

      for (unsigned int count =0 ; count < kcdBodyInnerObjects.size() ; count ++) {
	CkcdObjectShPtr kcdObject = kcdBodyInnerObjects[count] ;
	ignoredOuterList.push_back(kcdObject) ;
        
      }
    }
    else {
      std::cerr << "ChppDevice::ignoreDeviceForCollision : body is not KCD body. box not inserted." << std::endl;
      return KD_ERROR ;
    } 
  }


  //
  // set the ignored List in the device
  //
  CkwsDevice::TBodyVector thisBodyVector;
  this->getBodyVector(thisBodyVector) ;

  for (unsigned int bodyIter = 0 ;  bodyIter < thisBodyVector.size(); bodyIter++)  {

    CkwsKCDBodyShPtr thisKcdBody;

    if (thisKcdBody = KIT_DYNAMIC_PTR_CAST(CkwsKCDBody,thisBodyVector[bodyIter])) {
  
      std::vector< CkcdObjectShPtr > thisIgnoredListObjects = thisKcdBody->ignoredOuterObjects() ;

      // update the vector with  old object + new object
      for( unsigned int count = 0 ; count < ignoredOuterList.size() ; count++) {
	thisIgnoredListObjects.push_back(ignoredOuterList[count]) ;
      }


      thisKcdBody->ignoredOuterObjects(thisIgnoredListObjects) ;

    }
    else {
      std::cerr << "ChppDevice::ignoreDeviceForCollision : body is not KCD body. box not inserted." << std::endl;
      return KD_ERROR ;
    } 
  }
  
  return status ;
}

// ==========================================================================

void ChppDevice::cksBodyBoundingBox(const CkwsKCDBodyShPtr& body, double& xMin, double& yMin, 
                               double& zMin, double& xMax, double& yMax, double& zMax) const
{
  std::vector<CkcdObjectShPtr> listObject = body->innerObjects();
  unsigned int j= listObject.size();
  for(unsigned int i=0; i<j; i++)
  {
    ckcdObjectBoundingBox(listObject[i],xMin,yMin,zMin,xMax,yMax,zMax);
  }
}

// ==========================================================================

void ChppDevice::ckcdObjectBoundingBox(const CkcdObjectShPtr& object, double& xMin, double& yMin, 
                               double& zMin, double& xMax, double& yMax, double& zMax) const
{
  double x,y,z;

  object->boundingBox()->getHalfLengths(x, y, z) ;

  /*Matrices absolute et relative*/
  CkitMat4 matrixAbsolutePosition;
  CkitMat4 matrixRelativePosition;
  object->getAbsolutePosition(matrixAbsolutePosition);
  object->boundingBox()->getRelativePosition(matrixRelativePosition);

  /*Creer les points et change position points*/
  CkitMat4 matrixChangePosition = matrixAbsolutePosition*matrixRelativePosition;

  CkitPoint3 position[8];

  position[0]=matrixChangePosition*CkitPoint3( x, y, z);
  position[1]=matrixChangePosition*CkitPoint3( x, y,-z);
  position[2]=matrixChangePosition*CkitPoint3( x,-y, z);
  position[3]=matrixChangePosition*CkitPoint3(-x, y, z);
  position[4]=matrixChangePosition*CkitPoint3( x,-y,-z);
  position[5]=matrixChangePosition*CkitPoint3(-x,-y, z);
  position[6]=matrixChangePosition*CkitPoint3(-x, y,-z);
  position[7]=matrixChangePosition*CkitPoint3(-x,-y,-z);

  for(int i=0; i<8; i++)
  {
	if((position[i])[0]<xMin)
	{
	  xMin=(position[i])[0];
	}
	if((position[i])[1]<yMin)
	{
	  yMin=(position[i])[1];
	}
	if((position[i])[2]<zMin)
	{
	  zMin=(position[i])[2];
	}
	if((position[i])[0]>xMax)
	{
	  xMax=(position[i])[0];
	}
	if((position[i])[1]>yMax)
	{
	  yMax=(position[i])[1];
	}
	if((position[i])[2]>zMax)
	{
  	  zMax=(position[i])[2];
	}
  }

}

ktStatus ChppDevice::addObstacle(const CkcdObjectShPtr& inObject)
{
  // Get robot vector of bodies.
  CkwsDevice::TBodyVector bodyVector;
  getBodyVector(bodyVector);
    
  // Loop over bodies of robot.
  for (CkwsDevice::TBodyIterator bodyIter = bodyVector.begin(); bodyIter < bodyVector.end(); bodyIter++) {
    // Try to cast body into CkwsKCDBody
    CkwsKCDBodyShPtr kcdBody;
    ChppBodyShPtr hppBody;
    if (kcdBody = boost::dynamic_pointer_cast<CkwsKCDBody>(*bodyIter)) {
      std::vector< CkcdObjectShPtr > collisionList = kcdBody->outerObjects();
      collisionList.push_back(inObject);

      if(hppBody = boost::dynamic_pointer_cast<ChppBody>(kcdBody)){
	hppBody->setOuterObjects(collisionList);
      }
      else
	kcdBody->outerObjects(collisionList);

    }
    else {
      std::cout << "ChppDevice::addObstacle: body is not KCD body. Obstacle is not inserted." << std::endl;
    }
  }
  return KD_OK;
}
