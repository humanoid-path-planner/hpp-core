//
// Copyright (c) 2015 CNRS
// Author: Mylene Campana, Joseph Mirabel
//
//
// This file is part of hpp-core
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_COLLISION_CONSTRAINTS_HH
# define HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_COLLISION_CONSTRAINTS_HH

#include <hpp/util/exception-factory.hh>

# include <hpp/fcl/collision.h>
# include <hpp/fcl/distance.h>

# include <hpp/pinocchio/util.hh>
# include <hpp/pinocchio/configuration.hh>
# include <pinocchio/multibody/liegroup/liegroup.hpp>

# include <hpp/constraints/generic-transformation.hh>

namespace hpp {
  namespace core {
    namespace pathOptimization {

      template <typename SplinePtr_t>
      class CollisionFunction : public DifferentiableFunction
      {
      public:
        typedef boost::shared_ptr <CollisionFunction> Ptr_t;
       virtual ~CollisionFunction () {}
       static Ptr_t create
       (const DevicePtr_t& robot, const SplinePtr_t& freeSpline,
        const SplinePtr_t& collSpline,
        const CollisionPathValidationReportPtr_t& report)
       {
         CollisionFunction* ptr = new CollisionFunction
           (robot, freeSpline, collSpline, report);
         return Ptr_t (ptr);
       }

       void updateConstraint (const Configuration_t& q)
       {
         fcl::CollisionResult result = checkCollision(q, true);

         if (result.numContacts() == 1) { // Update qColl_
           qColl_ = q;
           contactPoint_ = result.getContact (0).pos;
           hppDout (info, "contact point = " << contactPoint_.transpose());
         } else { // Update qFree_
           qFree_ = q;
         }

         computeJacobian();
       }

	Configuration_t qFree_, qColl_;

      protected:
       CollisionFunction (const DevicePtr_t& robot,
                          const SplinePtr_t& freeSpline,
                          const SplinePtr_t& collSpline,
                          const CollisionPathValidationReportPtr_t& report) :
         DifferentiableFunction (robot->configSize (), robot->numberDof (),
                                 LiegroupSpace::R1 (), ""),
         qFree_ (), qColl_ (), robot_ (robot), object1_ (), object2_ (),
         J_ (), difference_ (robot->numberDof ())
        {
          const value_type& tColl = report->parameter;
          bool success;
          qColl_ = (*collSpline) (tColl, success);
          assert(success);

          HPP_STATIC_CAST_REF_CHECK (CollisionValidationReport,
                                     *(report->configurationReport));
          CollisionValidationReportPtr_t collisionReport
            (HPP_STATIC_PTR_CAST (CollisionValidationReport,
                                  report->configurationReport));
          object1_ = collisionReport->object1;
          object2_ = collisionReport->object2;

          hppDout (info, "obj1 = " << object1_->name()
                   << " and obj2 = " << object2_->name());
          hppDout (info, "qColl = " << pinocchio::displayConfig (qColl_));

          // Backtrack collision in previous path (x0) to create constraint
          value_type tFree = tColl * freeSpline->length () /
            collSpline->length();

          qFree_ = (*freeSpline) (tFree, success);
          assert(success);
          hppDout (info, "qFree = " << pinocchio::displayConfig (qFree_));
          // Compute contact point in configuration qColl
          const fcl::CollisionResult& result (collisionReport->result);
          if (result.numContacts () < 1) {
            abort ();
          }
          assert (result.numContacts () >= 1);
          contactPoint_ = result.getContact (0).pos;
          hppDout (info, "contact point = " << contactPoint_.transpose());
          computeJacobian();
        }

       fcl::CollisionResult checkCollision (const Configuration_t& q, bool enableContact)
       {
	 robot_->currentConfiguration (q);
	 robot_->computeForwardKinematics ();
         robot_->updateGeometryPlacements ();
	 fcl::CollisionResult result;
         fcl::CollisionRequest collisionRequest (1, enableContact, false, 1, false, true, fcl::GST_INDEP);
         fcl::collide (object1_->fcl (), object2_->fcl (), collisionRequest, result);
         return result;
       }

       void computeJacobian()
       {
         static const matrix3_t I3 (matrix3_t::Identity());

         DifferentiableFunctionPtr_t f;
         vector3_t u;

         robot_->currentConfiguration (qColl_);
         robot_->computeForwardKinematics ();

         if (!object2_->joint() || object2_->joint()->index() == 0) {
           object1_.swap(object2_);
         }

	 JointConstPtr_t joint1 = object1_->joint ();
	 JointConstPtr_t joint2 = object2_->joint ();
         assert(joint2 && joint2->index() > 0);
         Transform3f M2 (joint2->currentTransformation ());
         vector3_t x2_J2 (M2.actInv(contactPoint_));

         if (joint1 && joint1->index() > 0) { // object1 = body part
           Transform3f M1 (joint1->currentTransformation ());
           // Position of contact point in each object local frame
           vector3_t x1_J1 = M1.actInv (contactPoint_);
           // Compute contact points in configuration qFree
           robot_->currentConfiguration (qFree_);
           robot_->computeForwardKinematics ();
           M2 = joint2->currentTransformation ();
           M1 = joint1->currentTransformation ();
           // Position of x1 in local frame of joint2
           vector3_t x1_J2 (M2.actInv (M1.act (x1_J1)));
           hppDout (info, "x2 in J2 = " << x2_J2.transpose());
           hppDout (info, "x1 in J2 = " << x1_J2.transpose());

           u = (x1_J2 - x2_J2).normalized();
           f = constraints::RelativePosition::create
             ("", robot_, joint1, joint2, Transform3f(I3, x1_J1), Transform3f(I3 ,x2_J2));
         } else{ // object1 = fixed obstacle and has no joint
           vector3_t x1_J1 (contactPoint_);
           // Compute contact points in configuration qFree
           robot_->currentConfiguration (qFree_);
           robot_->computeForwardKinematics ();
           Transform3f M2 (joint2->currentTransformation ());
           // position of x2 in global frame
           vector3_t x2_J1 (M2.act (x2_J2));
           hppDout (info, "x2 in J1 = " << x2_J1.transpose());

           u = (x2_J1 - x2_J2).normalized();
           f = constraints::Position::create
             ("", robot_, joint2, Transform3f(I3 ,x2_J2), Transform3f(I3 ,x1_J1));
         }
         matrix_t Jpos (f->outputSize (), f->inputDerivativeSize ());
         f->jacobian (Jpos, qFree_);
         J_ = u.transpose () * Jpos;
         assert (J_.rows () == 1);
       }

       virtual void impl_compute (LiegroupElement& result, vectorIn_t argument)
         const
       {
         pinocchio::difference<pinocchio::DefaultLieGroupMap> (robot_, argument, qFree_, difference_);
         result.vector () = J_ * difference_;
         result.check ();
       }
       virtual void impl_jacobian (matrixOut_t jacobian, vectorIn_t) const
       {
         jacobian = J_;
       }
      private:
	DevicePtr_t robot_;
        CollisionObjectConstPtr_t object1_, object2_;
	matrix_t J_;
        vector3_t contactPoint_;
	mutable vector_t difference_;
      }; // class CollisionFunction
    } // namespace pathOptimization
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_PATH_OPTIMIZATION_SPLINE_GRADIENT_BASED_COLLISION_CONSTRAINTS_HH
