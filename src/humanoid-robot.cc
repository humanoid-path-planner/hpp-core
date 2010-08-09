/*
 *  Copyright (c) 2010 LAAS-CNRS
 *
 *  Author: Florent Lamiraux
 */

#include <hppModel/hppHumanoidRobot.h>
#include "humanoid-robot.hh"

using hpp::core::io::HumanoidRobot;
ChppHumanoidRobotShPtr HumanoidRobot::createHppHumanoidRobot() const
{
  ChppHumanoidRobotShPtr robot = ChppHumanoidRobot::create(name());
  try {
    buildKinematicChain(rootJointComponent());
  }
  catch (const std::exception& exc) {
    std::cerr <<
      "Exception in hpp::core::io::HumanoidRobot::createHppHumanoidRobot():"
	      << exc.what() << std::endl;
    robot.reset();
  }
  return robot;
}

void
HumanoidRobot::buildKinematicChain(const CkppJointComponentConstShPtr& root)
  const
{
  std::string message;

}

