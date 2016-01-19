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

#include <hpp/util/debug.hh>
#include <hpp/model/configuration.hh>
#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>

namespace hpp {
  namespace core {
    namespace {
        template <typename T>
	std::string numToStr (const T& v) {
 	  std::stringstream ss; ss<<v; return ss.str ();
	}
    }

    /// Copy object and return shared pointer to copy
    EquationPtr_t LockedJoint::copy () const
    {
      return createCopy (weak_.lock ());
    }

    LockedJointPtr_t LockedJoint::create (const JointPtr_t& joint,
					  vectorIn_t value)
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

    std::size_t LockedJoint::rankInConfiguration () const
    {
      return rankInConfiguration_;
    }

    std::size_t LockedJoint::rankInVelocity () const
    {
      return rankInVelocity_;
    }

    std::size_t LockedJoint::size () const
    {
      return rhsSize ();
    }

    std::size_t LockedJoint::numberDof () const
    {
      return numberDof_;
    }

    vectorIn_t LockedJoint::value () const
    {
      return rightHandSide ();
    }

    void LockedJoint::value (vectorIn_t value)
    {
      rightHandSide (value);
    }

    bool LockedJoint::isSatisfied (ConfigurationIn_t config)
    {
      bool result (config.segment (rankInConfiguration_,
				   size ()).isApprox (rightHandSide ()));
      if (result) {
	hppDout (info, "(" <<
		 config.segment (rankInConfiguration_, size ()).transpose ()
		 << ")==(" << rightHandSide ().transpose () << ") succeeded");
	return true;
      } else {
	hppDout (info, "(" <<
		 config.segment (rankInConfiguration_, size ()).transpose ()
		 << ")==(" << rightHandSide ().transpose () << ") failed");
	return false;
      }
    }

    bool LockedJoint::isSatisfied (ConfigurationIn_t config, vector_t& error)
    {
      error = config.segment (rankInConfiguration_, size ()) - rightHandSide ();
      return config.segment (rankInConfiguration_, size ()).isApprox (rightHandSide ());
    }

    void LockedJoint::rightHandSideFromConfig (ConfigurationIn_t config)
    {
      if (!comparisonType ()->constantRightHandSide ())
        nonConstRightHandSide () = config.segment (rankInConfiguration_, size ());
    }

    LockedJoint::LockedJoint (const JointPtr_t& joint, vectorIn_t value) :
      Equation (Equality::create (), vector_t::Zero (joint->configSize ())),
      jointName_ (joint->name ()),
      rankInConfiguration_ (joint->rankInConfiguration ()),
      rankInVelocity_ (joint->rankInVelocity ()),
      numberDof_ (joint->numberDof ())
    {
      rightHandSide (value);
      assert (rhsSize () == joint->configSize ());
    }

    LockedJoint::LockedJoint (const JointPtr_t& joint, const size_type index,
        vectorIn_t value) :
      Equation (Equality::create (), vector_t::Zero (value.size ())),
      jointName_ ("partial_" + joint->name ()),
      rankInConfiguration_ (joint->rankInConfiguration () + index),
      rankInVelocity_ (joint->rankInVelocity () + index),
      numberDof_ (joint->numberDof ())
    {
      assert (joint->numberDof () == joint->configSize ());
      rightHandSide (value);
    }

    LockedJoint::LockedJoint (const DevicePtr_t& dev, const size_type index,
        vectorIn_t value) :
      Equation (Equality::create (), vector_t::Zero (value.size())),
      jointName_ (dev->name() + "_extraDof" + numToStr (index)),
      rankInConfiguration_
      (dev->configSize () - dev->extraConfigSpace().dimension() + index),
      rankInVelocity_
      (dev->numberDof ()  - dev->extraConfigSpace().dimension() + index),
      numberDof_ (value.size())
    {
      assert (value.size() > 0);
      assert (rankInConfiguration_ + value.size() <= dev->configSize());
      rightHandSide (value);
      assert (rhsSize () == value.size());
    }

    void LockedJoint::init (const LockedJointPtr_t& self)
    {
      weak_ = self;
    }

    std::ostream& LockedJoint::print (std::ostream& os) const
    {
      os << "Locked joint " << jointName_
	 << ", value = " << model::displayConfig (rightHandSide ())
        << ": rank in configuration = " << rankInConfiguration_
        << ": rank in velocity = " << rankInVelocity_
        << std::endl;
      return os;
    }

    LockedJointPtr_t LockedJoint::create (const DevicePtr_t& robot)
    {
      LockedJoint* ptr = new LockedJoint (robot);
      LockedJointPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    LockedJoint::LockedJoint (const DevicePtr_t& robot) :
      Equation (ComparisonType::createDefault (), vector_t (1)),
      jointName_ ("FakeLockedJoint"),
      rankInConfiguration_ (robot->configSize ()),
      rankInVelocity_ (robot->numberDof ()), numberDof_ (0)
    {
    }

    LockedJoint::LockedJoint (const LockedJoint& other) :
      Equation (other), jointName_ (other.jointName_),
      rankInConfiguration_ (other.rankInConfiguration_),
      rankInVelocity_ (other.rankInVelocity_), numberDof_ (other.numberDof_),
      weak_ ()
    {
    }
  } // namespace core
} // namespace hpp
