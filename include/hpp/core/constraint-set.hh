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

#ifndef HPP_CORE_CONSTRAINT_SET_HH
# define HPP_CORE_CONSTRAINT_SET_HH

# include <deque>
# include <hpp/core/constraint.hh>

namespace hpp {
  namespace core {
    /// \addtogroup constraints
    /// \{

    /// Set of constraints applicable to a robot configuration
    ///
    /// \warning If the set is to contain a ConfigProjector and several
    /// LockedJoint instances, the configProjector should be inserted first
    /// since following numerical projections might affect locked degrees of
    /// freedom.
    class HPP_CORE_DLLAPI ConstraintSet : public Constraint
    {
    public:
      /// Return shared pointer to new object
      static ConstraintSetPtr_t create (const DevicePtr_t& robot,
					const std::string& name)
      {
	ConstraintSet* ptr = new ConstraintSet (robot, name);
	ConstraintSetPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      /// Return shared pointer to new object
      static ConstraintSetPtr_t createCopy (const ConstraintSetPtr_t& cs)
      {
	ConstraintSet* ptr = new ConstraintSet (*cs);
	ConstraintSetPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      /// return shared pointer to copy
      virtual ConstraintPtr_t copy () const;

      /// Add a constraint to the set
      void addConstraint (const ConstraintPtr_t& constraint);

      /// Return pointer to config projector if any
      ConfigProjectorPtr_t configProjector () const;

      /// Iterator over the constraints
      Constraints_t::iterator begin () {
        return constraints_.begin ();
      }
      /// Iterator over the constraints
      Constraints_t::iterator end () {
        return constraints_.end ();
      }

      /// Check whether a configuration statisfies the constraint.
      virtual bool isSatisfied (ConfigurationIn_t config);

      /// Check whether a configuration satisfies the constraint.
      ///
      /// \param config the configuration to check
      /// \retval error concatenation of errors of each constraint.
      virtual bool isSatisfied (ConfigurationIn_t config, vector_t& error);

      /// \name Compression of locked degrees of freedom
      ///
      /// Degrees of freedom related to locked joint are not taken into
      /// account in numerical constraint resolution. The following methods
      /// Compress or uncompress vectors or matrices by removing lines and
      /// columns corresponding to locked degrees of freedom.
      /// \{

      /// Get number of non-locked degrees of freedom
      size_type numberNonLockedDof () const;

      /// Compress Velocity vector by removing locked degrees of freedom
      ///
      /// \param normal input velocity vector
      /// \retval small compressed velocity vectors
      void compressVector (vectorIn_t normal, vectorOut_t small) const;

      /// Expand compressed velocity vector
      ///
      /// \param small compressed velocity vector without locked degrees of
      ///              freedom,
      /// \retval normal uncompressed velocity vector.
      /// \note locked degree of freedom are not set. They should be initialized
      ///       to zero.
      void uncompressVector (vectorIn_t small, vectorOut_t normal) const;

      /// Compress matrix
      ///
      /// \param normal input matrix
      /// \retval small compressed matrix
      /// \param rows whether to compress rows and colums or only columns
      void compressMatrix (matrixIn_t normal, matrixOut_t small,
			   bool rows = true) const;

      /// Uncompress matrix
      ///
      /// \param small input matrix
      /// \retval normal uncompressed matrix
      /// \param rows whether to uncompress rows and colums or only columns
      void uncompressMatrix (matrixIn_t small, matrixOut_t normal,
			     bool rows = true) const;
      /// \}

    protected:
      ConstraintSet (const DevicePtr_t& robot, const std::string& name);
      /// Copy constructor
      ConstraintSet (const ConstraintSet& other);
      /// Store weak pointer to itself.
      void init (const ConstraintSetPtr_t& self)
      {
	Constraint::init (self);
	weak_ = self;
      }
      virtual bool impl_compute (ConfigurationOut_t configuration);

      virtual std::ostream& print (std::ostream& os) const;

      /// Constructor for serialization only.
      ConstraintSet() {}

    private:
      ConfigProjectorPtr_t _configProj () const;

      Constraints_t constraints_;
      int configProjI_;
      ConstraintSetWkPtr_t weak_;

      friend class constraints::LockedJoint;
      friend class Constraint;
      friend class ConfigProjector;

      HPP_SERIALIZABLE();
    }; // class ConstraintSet
    /// \}
  } // namespace core
} // namespace hpp

BOOST_CLASS_EXPORT_KEY(hpp::core::ConstraintSet)

#endif // HPP_CORE_CONSTRAINT_SET_HH
