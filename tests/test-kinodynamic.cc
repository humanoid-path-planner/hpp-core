// Copyright (c) 2016, LAAS-CNRS
// Authors: Pierre Fernbach (pierre.fernbach@laas.fr)
//
// This file is part of the hpp-core.
//
// hpp-core is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// test-hpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with hpp-core.  If not, see <http://www.gnu.org/licenses/>.


#define BOOST_TEST_MODULE kinodynamic



#include <sstream>
#include <ostream>
#include <fstream>
#include <vector>

#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/simple-device.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/liegroup.hh>

# include <hpp/core/steering-method/steering-kinodynamic.hh>
# include <hpp/core/problem.hh>
# include <hpp/core/weighed-distance.hh>
# include <hpp/core/kinodynamic-path.hh>
# include <hpp/core/kinodynamic-distance.hh>
#include <boost/assign.hpp>

#include <hpp/util/debug.hh>
#include <hpp/core/fwd.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/configuration-shooter/uniform.hh>
#include <hpp/core/connected-component.hh>
#include <hpp/core/node.hh>
#include <hpp/core/nearest-neighbor.hh>
#include <hpp/core/joint-bound-validation.hh>
#include <hpp/core/discretized-path-validation.hh>



#include <boost/test/included/unit_test.hpp>
#include <../tests/util.hh>

using namespace hpp::core;
using namespace hpp::pinocchio;

BOOST_AUTO_TEST_SUITE( test_hpp_core )

BOOST_AUTO_TEST_CASE (kinodynamic) {
  
  DevicePtr_t robot = unittest::makeDevice(unittest::HumanoidSimple);
  robot->controlComputation((Computation_t) (JOINT_POSITION | JACOBIAN));
  robot->rootJoint()->lowerBound (0, -10);
  robot->rootJoint()->lowerBound (1, -10);
  robot->rootJoint()->lowerBound (2, -10);
  robot->rootJoint()->upperBound (0,  10);
  robot->rootJoint()->upperBound (1,  10);
  robot->rootJoint()->upperBound (2,  10);
  
  robot->setDimensionExtraConfigSpace(6);
  // define velocity and acceleration bounds
  const double vMax = 2;
  const double aMax = 0.5;
  robot->extraConfigSpace ().lower(0) = -vMax;
  robot->extraConfigSpace ().lower(1) = -vMax;
  robot->extraConfigSpace ().lower(2) = 0;
  robot->extraConfigSpace ().upper(0) = vMax;
  robot->extraConfigSpace ().upper(1) = vMax;
  robot->extraConfigSpace ().upper(2) = 0;
  robot->extraConfigSpace ().lower(3) = -aMax;
  robot->extraConfigSpace ().lower(4) = -aMax;
  robot->extraConfigSpace ().lower(5) = 0;
  robot->extraConfigSpace ().upper(3) = aMax;
  robot->extraConfigSpace ().upper(4) = aMax;
  robot->extraConfigSpace ().upper(5) = 0;
  BOOST_CHECK_MESSAGE( robot->extraConfigSpace().dimension () == 6 , "error during creation of the robot");
  
  
  

  // Create steering method
  Problem p = Problem (robot);
  p.setParameter(std::string("Kinodynamic/velocityBound"),Parameter(vMax));
  p.setParameter(std::string("Kinodynamic/accelerationBound"),Parameter(aMax));

  steeringMethod::KinodynamicPtr_t sm = steeringMethod::Kinodynamic::create (p);
  KinodynamicDistancePtr_t dist = KinodynamicDistance::createFromProblem(p);

  // try to connect several states : (notation : sx = (px, vx, ax)
  Configuration_t q0 (robot->currentConfiguration());
  Configuration_t q1 (robot->currentConfiguration());
  size_t indexECS = robot->configSize() - 6;

  JointBoundValidationPtr_t jointValidation = JointBoundValidation::create(robot);
  DiscretizedPathValidationPtr_t pathVal = DiscretizedPathValidation::create(robot,0.001);
  pathVal->add(jointValidation);
  PathValidationReportPtr_t validationReport;
  PathPtr_t validPath;

  // 1) simple case with null velocity, require a 2 segment trajectory
  // px0 = (0,0,0) , px1 = (1,0,0)
  q0[0] = 0;
  q1[0] = 1;
  PathPtr_t path = (*sm)(q0,q1);
  BOOST_REQUIRE (path);
  KinodynamicPathPtr_t pathKino = HPP_DYNAMIC_PTR_CAST(KinodynamicPath,path);
  BOOST_REQUIRE (pathKino);

  // length should be 2.82843
  BOOST_CHECK_CLOSE(path->length(),2.82843,1e-3);
  BOOST_CHECK_EQUAL(path->length(),(*dist)(q0,q1));
  // check if 2 segment only
  BOOST_CHECK_EQUAL(pathKino->getTv()[0],0.);
  BOOST_CHECK_EQUAL(pathKino->getT0()[0],0.);
  BOOST_CHECK_EQUAL(pathKino->getT1()[0] + pathKino->getTv()[0] + pathKino->getT2()[0],path->length());

  // check if no unecessary motion :
  for(size_t i = 1 ; i < 3 ; i++){
    BOOST_CHECK_EQUAL(pathKino->getT0()[i],0.);
    BOOST_CHECK_EQUAL(pathKino->getT1()[i],0.);
    BOOST_CHECK_EQUAL(pathKino->getT2()[i],0.);
    BOOST_CHECK_EQUAL(pathKino->getA1()[i],0.);
    BOOST_CHECK_EQUAL(pathKino->getTv()[i],path->length());
   }
   // check if the trajectory is really bang-bang :
  BOOST_CHECK_EQUAL(pathKino->getA1()[0],aMax);
  BOOST_CHECK_EQUAL(pathKino->getT1()[0],pathKino->getT2()[0]);
   // check if bounds are respected :
  BOOST_CHECK(pathVal->validate(path,false,validPath,validationReport));


  // 2) null velocity but hit velocity bounds, require 3 segments trajectory
  q0[0] = -2;
  q1[0] = 7;
  path = (*sm)(q0,q1);
  BOOST_REQUIRE (path);
  pathKino = HPP_DYNAMIC_PTR_CAST(KinodynamicPath,path);
  BOOST_REQUIRE (pathKino);

  // check length
  BOOST_CHECK_CLOSE(path->length(),8.5,1e-3);
  BOOST_CHECK_EQUAL(path->length(),(*dist)(q0,q1));
  // check if 3 segment
  BOOST_CHECK_EQUAL(pathKino->getT0()[0],0.);
  BOOST_CHECK(pathKino->getT1()[0] > 0);
  BOOST_CHECK(pathKino->getTv()[0] > 0);
  BOOST_CHECK(pathKino->getT2()[0] > 0);
  BOOST_CHECK_EQUAL(pathKino->getT1()[0] + pathKino->getTv()[0] + pathKino->getT2()[0],path->length());

  // check if no unecessary motion :
  for(size_t i = 1 ; i < 3 ; i++){
    BOOST_CHECK_EQUAL(pathKino->getT0()[i],0.);
    BOOST_CHECK_EQUAL(pathKino->getT1()[i],0.);
    BOOST_CHECK_EQUAL(pathKino->getT2()[i],0.);
    BOOST_CHECK_EQUAL(pathKino->getA1()[i],0.);
    BOOST_CHECK_EQUAL(pathKino->getTv()[i],path->length());
   }
   // check if the trajectory is really bang-bang :
  BOOST_CHECK_EQUAL(pathKino->getA1()[0],aMax);
  BOOST_CHECK_EQUAL(pathKino->getT1()[0],pathKino->getT2()[0]);
   // check if bounds are respected :
  BOOST_CHECK(pathVal->validate(path,false,validPath,validationReport));

  // 3) only velocity change :
  // px0 = (0,2,0) ; px1 = (0,-1,0)
  q0[0] = 0;
  q0[indexECS] = 2;
  q1[0] = 0;
  q1[indexECS] = -1;

  path = (*sm)(q0,q1);
  BOOST_REQUIRE (path);
  pathKino = HPP_DYNAMIC_PTR_CAST(KinodynamicPath,path);
  BOOST_REQUIRE (pathKino);

  // check length
  BOOST_CHECK_CLOSE(path->length(),8.32456,1e-3);
  BOOST_CHECK_EQUAL(path->length(),(*dist)(q0,q1));
  // check if 2 segment
  BOOST_CHECK_EQUAL(pathKino->getT0()[0],0.);
  BOOST_CHECK(pathKino->getT1()[0] > 0);
  BOOST_CHECK_EQUAL(pathKino->getTv()[0] , 0);
  BOOST_CHECK(pathKino->getT2()[0] > 0);
  BOOST_CHECK_EQUAL(pathKino->getT1()[0] + pathKino->getTv()[0] + pathKino->getT2()[0],path->length());

  // check if no unecessary motion :
  for(size_t i = 1 ; i < 3 ; i++){
    BOOST_CHECK_EQUAL(pathKino->getT0()[i],0.);
    BOOST_CHECK_EQUAL(pathKino->getT1()[i],0.);
    BOOST_CHECK_EQUAL(pathKino->getT2()[i],0.);
    BOOST_CHECK_EQUAL(pathKino->getA1()[i],0.);
    BOOST_CHECK_EQUAL(pathKino->getTv()[i],path->length());
   }
   // check if the trajectory is really bang-bang :
  BOOST_CHECK_EQUAL(pathKino->getA1()[0],-aMax);
   // check if bounds are respected :
  BOOST_CHECK(pathVal->validate(path,false,validPath,validationReport));


  // 4) non null velocity  (and sigma == 0)
  // px0 = (0,1,0) ; px1 = (1,1,0)
  q0[0] = 0;
  q0[indexECS] = 1;
  q1[0] = 1;
  q1[indexECS] = 1;

  path = (*sm)(q0,q1);
  BOOST_REQUIRE (path);
  pathKino = HPP_DYNAMIC_PTR_CAST(KinodynamicPath,path);
  BOOST_REQUIRE (pathKino);

  // check length
  BOOST_CHECK_CLOSE(path->length(),0.89898,1e-3);
  BOOST_CHECK_EQUAL(path->length(),(*dist)(q0,q1));
  // check if 2 segment
  BOOST_CHECK_EQUAL(pathKino->getT0()[0],0.);
  BOOST_CHECK(pathKino->getT1()[0] > 0);
  BOOST_CHECK_EQUAL(pathKino->getTv()[0] , 0);
  BOOST_CHECK(pathKino->getT2()[0] > 0);
  BOOST_CHECK_EQUAL(pathKino->getT1()[0] + pathKino->getTv()[0] + pathKino->getT2()[0],path->length());

  // check if no unecessary motion :
  for(size_t i = 1 ; i < 3 ; i++){
    BOOST_CHECK_EQUAL(pathKino->getT0()[i],0.);
    BOOST_CHECK_EQUAL(pathKino->getT1()[i],0.);
    BOOST_CHECK_EQUAL(pathKino->getT2()[i],0.);
    BOOST_CHECK_EQUAL(pathKino->getA1()[i],0.);
    BOOST_CHECK_EQUAL(pathKino->getTv()[i],path->length());
   }
   // check if the trajectory is really bang-bang :
  BOOST_CHECK_CLOSE(pathKino->getA1()[0],aMax,1e-3);
  BOOST_CHECK_EQUAL(pathKino->getT1()[0],pathKino->getT2()[0]);
   // check if bounds are respected :
  BOOST_CHECK(pathVal->validate(path,false,validPath,validationReport));


  // 5) case with infeasible interval
  // infeasible interval on x axis is [1.172,6.828] , add a motion on y axis with a min time inside this interval :
  // py0 = (-2,0,0) py1 = (1,0,0)
  q0[1] = -2;
  q0[indexECS+1] = 0;
  q1[1] = 1;
  q1[indexECS+1] = 0;


  path = (*sm)(q0,q1);
  BOOST_REQUIRE (path);
  pathKino = HPP_DYNAMIC_PTR_CAST(KinodynamicPath,path);
  BOOST_REQUIRE (pathKino);

  // check length
  BOOST_CHECK_CLOSE(path->length(),6.82843,1e-3);
  // don't check if distance == length here because the distance method don't consider infeasile interval
  // It's a arbitrary choice made to speed up the computation of the distance

  // check if 2 segment
  for(size_t i = 0 ; i < 2; i++){
    BOOST_CHECK_EQUAL(pathKino->getT0()[0],0.);
    BOOST_CHECK(pathKino->getT1()[i] > 0);
    BOOST_CHECK_EQUAL(pathKino->getTv()[i], 0);
    BOOST_CHECK(pathKino->getT2()[i] > 0);
    BOOST_CHECK_EQUAL(pathKino->getT1()[i] + pathKino->getTv()[i] + pathKino->getT2()[i],path->length());
    }

  // check if no unecessary motion :
  BOOST_CHECK_EQUAL(pathKino->getT0()[2],0.);
  BOOST_CHECK_EQUAL(pathKino->getT1()[2],0.);
  BOOST_CHECK_EQUAL(pathKino->getT2()[2],0.);
  BOOST_CHECK_EQUAL(pathKino->getA1()[2],0.);
  BOOST_CHECK_EQUAL(pathKino->getTv()[2],path->length());
   // check if the trajectory is really bang-bang :
  BOOST_CHECK_EQUAL(pathKino->getA1()[0],-aMax);
  BOOST_CHECK_EQUAL(pathKino->getT1()[0],pathKino->getT2()[0]);
  BOOST_CHECK_EQUAL(pathKino->getT1()[1],pathKino->getT2()[1]);
  BOOST_CHECK(pathKino->getA1()[1] < aMax);
  BOOST_CHECK(pathKino->getA1()[1] > 0);

   // check if bounds are respected :
  BOOST_CHECK(pathVal->validate(path,false,validPath,validationReport));



  // ##  check extract method
  q0[0] = -2;
  q0[indexECS] = 0;
  q1[0] = 7;
  q1[indexECS] = 0;


  path = (*sm)(q0,q1);
  BOOST_REQUIRE (path);
  pathKino = HPP_DYNAMIC_PTR_CAST(KinodynamicPath,path);
  BOOST_REQUIRE (pathKino);


  PathPtr_t extractedPath;
  KinodynamicPathPtr_t extractedPathKino;
  bool success;
  // take only beginning of the path (cut during second segment on x and third segment on y):
  double begin = 0. ;
  double end = 4.3;
  extractedPath = path->extract(std::make_pair(begin,end));
  BOOST_REQUIRE (extractedPath);
  extractedPathKino = HPP_DYNAMIC_PTR_CAST(KinodynamicPath,extractedPath);
  BOOST_REQUIRE (extractedPathKino);

  BOOST_CHECK_EQUAL(extractedPath->length(),end-begin);
  BOOST_CHECK_EQUAL(extractedPath->initial(),path->initial());
  BOOST_CHECK_EQUAL(extractedPath->end().head(indexECS + 3 ),(*path)(end,success).head(indexECS + 3 )); // ignore last 3 extraDof because acceleration is set to 0 in initial/end configuration
  for(size_t i = 0 ; i < 3 ; i++){
    BOOST_CHECK_EQUAL(extractedPathKino->getA1()[i], pathKino->getA1()[i]);
    BOOST_CHECK_EQUAL(extractedPathKino->getT0()[i], pathKino->getT0()[i]);
    BOOST_CHECK_EQUAL(extractedPathKino->getT1()[i], pathKino->getT1()[i]);
  }
  BOOST_CHECK_EQUAL(extractedPathKino->getTv()[2],extractedPath->length());
  BOOST_CHECK_EQUAL(extractedPathKino->getT2()[2],0);
  BOOST_CHECK_EQUAL(extractedPathKino->getTv()[0], end - extractedPathKino->getT1()[0]);
  BOOST_CHECK_EQUAL(extractedPathKino->getT2()[0],0);
  BOOST_CHECK_EQUAL(extractedPathKino->getTv()[1],0);
  BOOST_CHECK_EQUAL(extractedPathKino->getT2()[1],end - extractedPathKino->getT1()[1]);

  // take only beginning of the path (cut during first segment on both axis):
  begin = 0. ;
  end = 3.;
  extractedPath = path->extract(std::make_pair(begin,end));
  BOOST_REQUIRE (extractedPath);
  extractedPathKino = HPP_DYNAMIC_PTR_CAST(KinodynamicPath,extractedPath);
  BOOST_REQUIRE (extractedPathKino);

  BOOST_CHECK_EQUAL(extractedPath->length(),end-begin);
  BOOST_CHECK_EQUAL(extractedPath->initial(),path->initial());
  BOOST_CHECK_EQUAL(extractedPath->end().head(indexECS + 3 ),(*path)(end,success).head(indexECS + 3 )); // ignore last 3 extraDof because acceleration is set to 0 in initial/end configuration
  for(size_t i = 0 ; i < 3 ; i++){
    BOOST_CHECK_EQUAL(extractedPathKino->getA1()[i], pathKino->getA1()[i]);
    BOOST_CHECK_EQUAL(extractedPathKino->getT0()[i], pathKino->getT0()[i]);
    BOOST_CHECK_EQUAL(extractedPathKino->getT2()[i], 0.);
  }
  BOOST_CHECK_EQUAL(extractedPathKino->getT1()[2],0);
  BOOST_CHECK_EQUAL(extractedPathKino->getTv()[2],extractedPath->length());
  BOOST_CHECK_EQUAL(extractedPathKino->getT2()[2],0);
  BOOST_CHECK_EQUAL(extractedPathKino->getT1()[0],extractedPath->length());
  BOOST_CHECK_EQUAL(extractedPathKino->getTv()[0], 0);
  BOOST_CHECK_EQUAL(extractedPathKino->getT1()[1],extractedPath->length());
  BOOST_CHECK_EQUAL(extractedPathKino->getTv()[1],0);

  // take only end of the path (cut during first segment)
  begin = 2.;
  end = path->length();
  extractedPath = path->extract(std::make_pair(begin,end));
  BOOST_REQUIRE (extractedPath);
  extractedPathKino = HPP_DYNAMIC_PTR_CAST(KinodynamicPath,extractedPath);
  BOOST_REQUIRE (extractedPathKino);

  BOOST_CHECK_EQUAL(extractedPath->length(),end-begin);
  BOOST_CHECK_EQUAL(extractedPath->end(),path->end());
  BOOST_CHECK_EQUAL(extractedPath->initial().head(indexECS + 3 ),(*path)(begin,success).head(indexECS + 3 )); // ignore last 3 extraDof because acceleration is set to 0 in initial/end configuration
  for(size_t i = 0 ; i < 3 ; i++){
    BOOST_CHECK_EQUAL(extractedPathKino->getA1()[i], pathKino->getA1()[i]);
    BOOST_CHECK_EQUAL(extractedPathKino->getT0()[i], pathKino->getT0()[i]);
    BOOST_CHECK_EQUAL(extractedPathKino->getT2()[i], pathKino->getT2()[i]);
  }
  BOOST_CHECK_EQUAL(extractedPathKino->getT1()[2],0);
  BOOST_CHECK_EQUAL(extractedPathKino->getTv()[2],extractedPath->length());
  BOOST_CHECK_EQUAL(extractedPathKino->getT2()[2],0);
  BOOST_CHECK_EQUAL(extractedPathKino->getT1()[0],pathKino->getT1()[0] - begin );
  BOOST_CHECK_EQUAL(extractedPathKino->getTv()[0], pathKino->getTv()[0]);
  BOOST_CHECK_EQUAL(extractedPathKino->getT1()[1],pathKino->getT1()[1] - begin );
  BOOST_CHECK_EQUAL(extractedPathKino->getTv()[1],0);

  // extract path in the middle :
  begin = 1.5;
  end = 4.4;
  extractedPath = path->extract(std::make_pair(begin,end));
  BOOST_REQUIRE (extractedPath);
  extractedPathKino = HPP_DYNAMIC_PTR_CAST(KinodynamicPath,extractedPath);
  BOOST_REQUIRE (extractedPathKino);

  BOOST_CHECK_EQUAL(extractedPath->length(),end-begin);
  BOOST_CHECK_EQUAL(extractedPath->end().head(indexECS + 3 ),(*path)(end,success).head(indexECS + 3 ));
  BOOST_CHECK_EQUAL(extractedPath->initial().head(indexECS + 3 ),(*path)(begin,success).head(indexECS + 3 )); // ignore last 3 extraDof because acceleration is set to 0 in initial/end configuration
  for(size_t i = 0 ; i < 3 ; i++){
    BOOST_CHECK_EQUAL(extractedPathKino->getA1()[i], pathKino->getA1()[i]);
    BOOST_CHECK_EQUAL(extractedPathKino->getT0()[i], pathKino->getT0()[i]);
  }
  BOOST_CHECK_EQUAL(extractedPathKino->getT1()[2],0);
  BOOST_CHECK_EQUAL(extractedPathKino->getTv()[2],extractedPath->length());
  BOOST_CHECK_EQUAL(extractedPathKino->getT2()[2],0);
  // x axis is cut during first and second segment :
  BOOST_CHECK_EQUAL(extractedPathKino->getT1()[0],pathKino->getT1()[0] - begin );
  BOOST_CHECK_EQUAL(extractedPathKino->getTv()[0], extractedPath->length() - extractedPathKino->getT1()[0]);
  BOOST_CHECK_EQUAL(extractedPathKino->getT2()[0], 0);
  // y axis is cut during first and third segment
  BOOST_CHECK_EQUAL(extractedPathKino->getT1()[1],pathKino->getT1()[1] - begin );
  BOOST_CHECK_EQUAL(extractedPathKino->getTv()[1],0);
  BOOST_CHECK_EQUAL(extractedPathKino->getT2()[1],  extractedPath->length() - extractedPathKino->getT1()[1]);

}
BOOST_AUTO_TEST_SUITE_END()
