//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
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

#include <hpp/core/constraint-set.hh>

#include <boost/serialization/vector.hpp>
#include <boost/serialization/weak_ptr.hpp>

#include <hpp/util/debug.hh>
#include <hpp/util/serialization.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/core/config-projector.hh>

namespace hpp {
  namespace core {
    bool ConstraintSet::impl_compute (ConfigurationOut_t configuration)
    {
      for (Constraints_t::iterator itConstraint = constraints_.begin ();
	   itConstraint != constraints_.end (); ++itConstraint) {
	if (!(*itConstraint)->impl_compute (configuration)) return false;
      }
      return true;
    }

    ConstraintPtr_t ConstraintSet::copy () const
    {
      return createCopy (weak_.lock ());
    }

    ConstraintSet::ConstraintSet (const DevicePtr_t& robot,
				  const std::string& name) :
      Constraint (name), constraints_ (), configProjI_ (-1)
    {
      constraints_.push_back (ConfigProjector::create (robot, "Trivial", 1e-12, 0));
    }

    ConstraintSet::ConstraintSet (const ConstraintSet& other) :
      Constraint (other), constraints_ (), configProjI_ (other.configProjI_)
    {
      for (Constraints_t::const_iterator it = other.constraints_.begin ();
	   it != other.constraints_.end (); ++it)
	constraints_.push_back ((*it)->copy ());
    }

    void ConstraintSet::addConstraint (const ConstraintPtr_t& c)
    {
      ConfigProjectorPtr_t cp (HPP_DYNAMIC_PTR_CAST(ConfigProjector, c));
      ConstraintSetPtr_t cs (HPP_DYNAMIC_PTR_CAST(ConstraintSet, c));
      if (cp) {
        if (configProjI_ >= 0)
          throw std::runtime_error("Constraint set " + name() + " cannot store "
              "more than one config-projector.");
        constraints_.erase(constraints_.begin());
        configProjI_ = static_cast<int>(constraints_.size());
      } else if (cs) {
        for (Constraints_t::iterator _c = cs->begin (); _c != cs->end (); ++_c)
          addConstraint(*_c);
        return;
      }
      constraints_.push_back(c);
    }

    ConfigProjectorPtr_t ConstraintSet::configProjector () const
    {
      return (configProjI_ >= 0
          ? HPP_STATIC_PTR_CAST (ConfigProjector, constraints_[configProjI_])
          : ConfigProjectorPtr_t());
    }

    bool ConstraintSet::isSatisfied (ConfigurationIn_t configuration)
    {
      for (Constraints_t::iterator itConstraint = constraints_.begin ();
	   itConstraint != constraints_.end (); ++itConstraint) {
	if (!(*itConstraint)->isSatisfied (configuration)) {
	  return false;
	}
      }
      return true;
    }

    bool ConstraintSet::isSatisfied (ConfigurationIn_t configuration,
				     vector_t& error)
    {
      bool result = true;
      error.resize (0);
      vector_t localError;
      for (Constraints_t::iterator itConstraint = constraints_.begin ();
	   itConstraint != constraints_.end (); ++itConstraint) {
	if (!(*itConstraint)->isSatisfied (configuration, localError)) {
	  result = false;
	}
	error.conservativeResize (error.size () + localError.size ());
	error.tail (localError.size ()) = localError;
      }
      return result;
    }

    size_type ConstraintSet::numberNonLockedDof () const
    {
      return _configProj()->numberFreeVariables ();
    }

    void ConstraintSet::compressVector (vectorIn_t normal,
					vectorOut_t small) const
    {
      _configProj()->compressVector (normal, small);
    }

    void ConstraintSet::uncompressVector (vectorIn_t small,
					  vectorOut_t normal) const
    {
      _configProj()->uncompressVector (small, normal);
    }

    void ConstraintSet::compressMatrix (matrixIn_t normal, matrixOut_t small,
					bool rows) const
    {
      _configProj()->compressMatrix (normal, small, rows);
    }

    void ConstraintSet::uncompressMatrix (matrixIn_t small,
					  matrixOut_t normal,
					  bool rows) const
    {
      _configProj()->uncompressMatrix (small, normal, rows);
    }

    std::ostream& ConstraintSet::print (std::ostream& os) const
    {
      os << "Constraint set " << name () << ", contains" << incindent;
      for (Constraints_t::const_iterator itConstraint = constraints_.begin ();
          itConstraint != constraints_.end (); itConstraint++) {
        os << iendl << **itConstraint;
      }
      return os << decindent;
    }

    ConfigProjectorPtr_t ConstraintSet::_configProj () const
    {
      return HPP_STATIC_PTR_CAST (ConfigProjector,
          constraints_[configProjI_ >= 0 ? configProjI_ : 0]);
    }

    template<class Archive>
    void ConstraintSet::serialize(Archive & ar, const unsigned int version)
    {
      using namespace boost::serialization;
      (void) version;
      ar & make_nvp("base", base_object<Constraint>(*this));
      ar & BOOST_SERIALIZATION_NVP(constraints_);
      ar & BOOST_SERIALIZATION_NVP(configProjI_);
      ar & BOOST_SERIALIZATION_NVP(weak_);
    }

    HPP_SERIALIZATION_IMPLEMENT(ConstraintSet);
  } // namespace core
} // namespace hpp

BOOST_CLASS_EXPORT(hpp::core::ConstraintSet)
