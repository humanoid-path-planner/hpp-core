//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#include <Eigen/SVD>
#include <hpp/core/kinodynamic-distance.hh>
#include <hpp/core/problem.hh>
#include <hpp/pinocchio/body.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/util/debug.hh>
#include <limits>

namespace hpp {
namespace core {

KinodynamicDistancePtr_t KinodynamicDistance::create(const DevicePtr_t& robot) {
  KinodynamicDistance* ptr = new KinodynamicDistance(robot);
  KinodynamicDistancePtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

KinodynamicDistancePtr_t KinodynamicDistance::createFromProblem(
    const ProblemConstPtr_t& problem) {
  KinodynamicDistance* ptr = new KinodynamicDistance(problem);
  KinodynamicDistancePtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

KinodynamicDistancePtr_t KinodynamicDistance::createCopy(
    const KinodynamicDistancePtr_t& distance) {
  KinodynamicDistance* ptr = new KinodynamicDistance(*distance);
  KinodynamicDistancePtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

DistancePtr_t KinodynamicDistance::clone() const {
  return createCopy(weak_.lock());
}

KinodynamicDistance::KinodynamicDistance(const DevicePtr_t& robot)
    : robot_(robot) {
  if (robot_->extraConfigSpace().dimension() < 6) {
    std::cout << "Error : you need at least 6 extra DOF" << std::endl;
    hppDout(error, "Error : you need at least 6 extra DOF");
  }
  hppDout(warning,
          "Kinodynamic distance create from robot, cannot access user-defined "
          "velocity and acceleration bounds. Use default values");
  aMax_ = 10.;
  vMax_ = 1.;
}

KinodynamicDistance::KinodynamicDistance(const ProblemConstPtr_t& problem)
    : robot_(problem->robot()) {
  if (robot_->extraConfigSpace().dimension() < 6) {
    std::cout << "Error : you need at least 6 extra DOF" << std::endl;
    hppDout(error, "Error : you need at least 6 extra DOF");
  }
  aMax_ = problem->getParameter("Kinodynamic/accelerationBound").floatValue();
  vMax_ = problem->getParameter("Kinodynamic/velocityBound").floatValue();
}

KinodynamicDistance::KinodynamicDistance(const KinodynamicDistance& distance)
    : robot_(distance.robot_) {}

void KinodynamicDistance::init(KinodynamicDistanceWkPtr_t self) {
  weak_ = self;
}

inline double sgnenum(double val) { return ((0. < val) - (val < 0.)); }

inline int sgn(double d) { return d >= 0.0 ? 1 : -1; }

inline double sgnf(double d) { return d >= 0.0 ? 1.0 : -1.0; }

double KinodynamicDistance::computeMinTime(double p1, double p2, double v1,
                                           double v2) const {
  // hppDout(info,"p1 = "<<p1<<"  p2 = "<<p2<<"   ; v1 = "<<v1<<"    v2 =
  // "<<v2);
  //  compute the sign of each acceleration
  double t1, t2, tv;
  int sigma;
  double deltaPacc = 0.5 * (v1 + v2) * (fabs(v2 - v1) / aMax_);
  sigma = sgn(p2 - p1 - deltaPacc);
  // hppDout(info,"sigma = "<<sigma);
  if (sigma == 0) {
    sigma = sgn(p2 - p1);
    // hppDout(info,"sigma Bis= "<<sigma);
  }
  double a1 = (sigma)*aMax_;
  double a2 = -a1;
  double vLim = (sigma)*vMax_;
  // hppDout(info,"Vlim = "<<vLim<<"   ;  aMax = "<<aMax_);
  if (fabs(p2 - p1) < (std::numeric_limits<double>::epsilon() * 100.) &&
      fabs(v2 - v1) < (std::numeric_limits<double>::epsilon() * 100.)) {
    // hppDout(notice,"No movement in this joints, abort.");
    return 0.;
  }
  // test if two segment trajectory is valid :
  bool twoSegment = false;

  // solve quadratic equation (cf eq 13 article)
  const double a = a1;
  const double b = 2. * v1;
  const double c = (0.5 * (v1 + v2) * (v2 - v1) / a2) - (p2 - p1);

  const double q = -0.5 * (b + sgnf(b) * sqrt(b * b - 4 * a * c));
  // hppDout(info, "sign of "<<b<<" is : "<<sgnf(b));
  const double x1 = q / a;
  const double x2 = c / q;
  const double x = std::max(x1, x2);
  // hppDout(info,"Solve quadratic equation : x1 = "<<x1<<"  ; x2 = "<<x2);
  // hppDout(info," x = "<<x);
  // hppDout(info,"t1 before vel limit = "<<x);

  // hppDout(info,"inf bound on t1 (from t2 > 0) "<<-((v2-v1)/a2));
  double minT1 = std::max(
      0., -((v2 - v1) / a2));  // lower bound for valid t1 value (cf eq 14)

  if (x >= minT1) {
    twoSegment = true;
    t1 = x;
    // hppDout(info,"t1 >= minT1");
  }
  if (twoSegment) {  // check if max velocity is respected
    if (std::abs(v1 + (t1)*a1) > vMax_) {
      twoSegment = false;
      // hppDout(info,"Doesn't respect max velocity, need 3 segments");
    }
  }
  if (twoSegment) {  // compute t2 for two segment trajectory
    tv = 0.;
    t2 = ((v2 - v1) / a2) + (t1);  // eq 14
  } else {  // compute 3 segment trajectory, with constant velocity phase :
    t1 = (vLim - v1) / a1;  // eq 15
    tv = ((v1 * v1 + v2 * v2 - 2 * vLim * vLim) / (2 * vLim * a1)) +
         (p2 - p1) / vLim;  // eq 16
    t2 = (v2 - vLim) / a2;  // eq 17
  }
  /*
  if(twoSegment){
    hppDout(notice,"Trajectory with 2 segments");
  }else{
    hppDout(notice,"Trajectory with 3 segments");
  }
  */
  // hppDout(notice,"a1 = "<<a1<<"  ;  a2 ="<<a2);
  // hppDout(notice,"t = "<<(t1)<<"   ;   "<<(tv)<<"   ;   "<<(t2));
  // double T = (t1)+(tv)+(t2);
  // hppDout(notice,"T = "<<T);
  return (t1) + (tv) + (t2);
}

value_type KinodynamicDistance::impl_distance(ConfigurationIn_t q1,
                                              ConfigurationIn_t q2) const {
  double Tmax = 0;
  double T = 0;

  size_type configSize =
      robot_->configSize() - robot_->extraConfigSpace().dimension();
  // looking for Tmax
  // hppDout(notice,"KinodynamicDistance :  Looking for Tmax :");
  // hppDout(info,"Distance between : "<<pinocchio::displayConfig(q1));
  // hppDout(info,"and              : "<<pinocchio::displayConfig(q2));

  for (int indexConfig = 0; indexConfig < 3;
       indexConfig++) {  // FIX ME : only work with freeflyer
                         // hppDout(notice,"For joint
    // :"<<robot_->getJointAtConfigRank(indexConfig)->name());
    // if(robot_->getJointAtConfigRank(indexConfig)->name() != "base_joint_SO3"
    // true){
    T = computeMinTime(q1[indexConfig], q2[indexConfig],
                       q1[indexConfig + configSize],
                       q2[indexConfig + configSize]);
    if (T > Tmax) Tmax = T;
    /*}else{
        hppDout(notice,"!! Kinodynamic distance for quaternion not implemented
    yet.");
    }*/
  }
  // hppDout(info," is : "<<Tmax);
  return Tmax;
}
}  //   namespace core
}  // namespace hpp
