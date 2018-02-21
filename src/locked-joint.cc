// Copyright (c) 2015, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-core.
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
// hpp-core. If not, see <http://www.gnu.org/licenses/>.

#include "hpp/core/locked-joint.hh"

#include <sstream>

#include <boost/assign/list_of.hpp>

#include <hpp/util/debug.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>

#include <hpp/constraints/affine-function.hh>

namespace hpp {
  namespace core {
    using boost::assign::list_of;
    using constraints::ConstantFunction;
    namespace {
        template <typename T>
	std::string numToStr (const T& v) {
 	  std::stringstream ss; ss<<v; return ss.str ();
	}

        DifferentiableFunctionPtr_t makeFunction (
            const LiegroupElement& lge, const std::string& name)
        {
          return DifferentiableFunctionPtr_t
            (new ConstantFunction (lge, 0, 0, "LockedJoint " + name));
        }
    }

    /// Copy object and return shared pointer to copy
    EquationPtr_t LockedJoint::copy () const
    {
      return createCopy (weak_.lock ());
    }

    LockedJointPtr_t LockedJoint::create (const JointPtr_t& joint,
					  const LiegroupElement& value)
    {
      LockedJoint* ptr = new LockedJoint (joint, value);
      LockedJointPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    LockedJointPtr_t LockedJoint::create (const JointPtr_t& joint,
                                          const size_type index,
					  vectorIn_t value)
    {
      LockedJoint* ptr = new LockedJoint (joint, index, value);
      LockedJointPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    LockedJointPtr_t LockedJoint::create (const DevicePtr_t& dev,
        const size_type index, vectorIn_t value)
    {
      LockedJoint* ptr = new LockedJoint (dev, index, value);
      LockedJointPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    LockedJointPtr_t LockedJoint::createCopy (LockedJointConstPtr_t other)
    {
      LockedJoint* ptr = new LockedJoint (*other);
      LockedJointPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    size_type LockedJoint::rankInConfiguration () const
    {
      return outputConf_[0].first;
    }

    size_type LockedJoint::rankInVelocity () const
    {
      return outputVelocity_[0].first;
    }

    size_type LockedJoint::configSize () const
    {
      return configSpace_->nq ();
    }

    size_type LockedJoint::numberDof () const
    {
      return configSpace_->nv ();
    }

    const LiegroupSpacePtr_t& LockedJoint::configSpace () const
    {
      return configSpace_;
    }

    bool LockedJoint::isSatisfied (ConfigurationIn_t config)
    {
      vector_t error;
      return isSatisfied (config, error);
    }

    bool LockedJoint::isSatisfied (ConfigurationIn_t config, vector_t& error)
    {
      LiegroupElement q (config.segment (rankInConfiguration(), configSize ()),
                         configSpace_);
      error = q - (configSpace_->exp (rightHandSide ()));
      return error.isApprox (vector_t::Zero (joint_->numberDof ()));
    }

    void LockedJoint::rightHandSideFromConfig (ConfigurationIn_t config)
    {
      if (!constantRightHandSide ()) {
        LiegroupElement q (config.segment (rankInConfiguration(), configSize ()),
                           configSpace_);
        rightHandSide (q - configSpace_->neutral ());
      }
    }

    LockedJoint::LockedJoint (const JointPtr_t& joint,
                              const LiegroupElement& value) :
      ExplicitNumericalConstraint (
          joint->robot(),
          makeFunction (value, joint->name()),
          segments_t(), // input conf
          segments_t(), // input vel
          list_of(segment_t (joint->rankInConfiguration(), joint->configSize())), // output conf
          list_of(segment_t (joint->rankInVelocity     (), joint->numberDof ())), // output vel
          ComparisonTypes_t(joint->numberDof(), constraints::Equality)),
      jointName_ (joint->name ()),
      configSpace_ (joint->configurationSpace ())
    {
      assert (rhsSize () == joint->numberDof ());
      assert (*(value.space ()) == *configSpace_);
      // rightHandSide (value - configSpace_->neutral ());
    }

    LockedJoint::LockedJoint (const JointPtr_t& joint, const size_type index,
        vectorIn_t value) :
      ExplicitNumericalConstraint (
          joint->robot(),
          makeFunction (
            LiegroupElement (value, LiegroupSpace::Rn (joint->configSize () - index)),
            "partial_" + joint->name()),
          segments_t(), // input conf
          segments_t(), // input vel
          list_of(segment_t (joint->rankInConfiguration(), joint->configSize()-index)), // output conf
          list_of(segment_t (joint->rankInVelocity     (), joint->numberDof ()-index)), // output vel
          ComparisonTypes_t(joint->numberDof()-index, constraints::Equality)),
      jointName_ ("partial_" + joint->name ()),
      joint_ (joint),
      configSpace_ (LiegroupSpace::Rn (joint->configSize () - index))
    {
      assert (joint->numberDof () == joint->configSize ());
      // rightHandSide (value);
      assert (rhsSize () == value.size());
    }

    LockedJoint::LockedJoint (const DevicePtr_t& dev, const size_type index,
        vectorIn_t value) :
      ExplicitNumericalConstraint (
          dev,
          makeFunction (
            LiegroupElement (value, LiegroupSpace::Rn (value.size ())),
            dev->name() + "_extraDof" + numToStr (index)),
          segments_t(), // input conf
          segments_t(), // input vel
          list_of(segment_t (
              dev->configSize () - dev->extraConfigSpace().dimension() + index,
              value.size())), // output conf
          list_of(segment_t (
              dev->numberDof ()  - dev->extraConfigSpace().dimension() + index,
              value.size())), // output vel
          ComparisonTypes_t(value.size(), constraints::Equality)),
      jointName_ (dev->name() + "_extraDof" + numToStr (index)),
      joint_ (JointPtr_t ()), configSpace_ (LiegroupSpace::Rn (value.size ()))
    {
      assert (value.size() > 0);
      assert (rankInConfiguration() + value.size() <= dev->configSize());
      // rightHandSide (value);
      assert (rhsSize () == value.size());
    }

    void LockedJoint::init (const LockedJointPtr_t& self)
    {
      ExplicitNumericalConstraint::init(self);
      weak_ = self;
    }

    std::ostream& LockedJoint::print (std::ostream& os) const
    {
      LiegroupElement v; vector_t empty;
      function().value(v, empty);
      v += rightHandSide();
      os << "Locked joint " << jointName_
	 << ", value = " << pinocchio::displayConfig (v.vector())
        << ": rank in configuration = " << rankInConfiguration()
        << ": rank in velocity = " << rankInVelocity()
        << std::endl;
      return os;
    }

    LockedJoint::LockedJoint (const LockedJoint& other) :
      ExplicitNumericalConstraint (other), jointName_ (other.jointName_),
      joint_ (other.joint_), configSpace_ (other.configSpace_), weak_ ()
    {
    }

    bool LockedJoint::isEqual (const Equation& other, bool swapAndTest) const
    {
      try {
	const LockedJoint& lj =
	  dynamic_cast <const LockedJoint&> (other);
	if (!Equation::isEqual (other, false)) return false;
	if (jointName_ != lj.jointName_) return false;
	if (rankInConfiguration() != lj.rankInConfiguration()) return false;
	if (rankInVelocity() != lj.rankInVelocity()) return false;
	if (*configSpace_ != *(lj.configSpace_)) return false;
	if (swapAndTest) return lj.isEqual (*this, false);
	return true;
      } catch (const std::bad_cast& err) {
	return false;
      }
    }
  } // namespace core
} // namespace hpp
