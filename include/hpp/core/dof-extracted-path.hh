//
// Copyright (c) 2016 CNRS
// Authors: Joseph Mirabel
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

#ifndef HPP_CORE_DOF_EXTRACTED_PATH_HH
# define HPP_CORE_DOF_EXTRACTED_PATH_HH

# include <hpp/core/path.hh>

namespace hpp {
  namespace core {
    /// Result of the selection of some elements of an original path
    /// \note Decorator design pattern
    class DofExtractedPath : public Path
    {
    public:
      typedef Path parent_t;

      virtual ~DofExtractedPath () throw () {}

      /// Return a shared pointer to a copy of this
      virtual PathPtr_t copy () const
      {
	return createCopy (weak_.lock ());
      }

      /// Return a shared pointer to a copy of this and set constraints
      ///
      /// \param constraints constraints to apply to the copy
      /// \precond *this should not have constraints.
      virtual PathPtr_t copy (const ConstraintSetPtr_t& constraints) const
      {
	return createCopy (weak_.lock (), constraints);
      }

      static DofExtractedPathPtr_t
      create (const PathPtr_t& original, const SizeIntervals_t& intervals)
      {
	DofExtractedPath* ptr = new DofExtractedPath (original, intervals);
	DofExtractedPathPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      static DofExtractedPathPtr_t
      createCopy (const DofExtractedPathPtr_t& path)
      {
	DofExtractedPath* ptr = new DofExtractedPath (*path);
	DofExtractedPathPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      static DofExtractedPathPtr_t
      createCopy (const DofExtractedPathPtr_t& path,
		  const ConstraintSetPtr_t& constraints)
      {
	DofExtractedPath* ptr = new DofExtractedPath (*path, constraints);
	DofExtractedPathPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      virtual bool impl_compute (ConfigurationOut_t result,
				 value_type param) const
      {
        bool success = (*original_) (q_, param);
        if (success) dofExtract (q_, result);
        return success;
      }

      /// Get the initial configuration
      inline Configuration_t initial () const
      {
        Configuration_t q(outputSize());
        dofExtract(original_->initial(), q);
        return q;
      }

      /// Get the final configuration
      inline Configuration_t end () const
      {
        Configuration_t q(outputSize());
        dofExtract(original_->end(), q);
        return q;
      }

      void dofExtract (ConfigurationIn_t qin, ConfigurationOut_t qout) const
      {
        size_type r = 0;
        for (SizeIntervals_t::const_iterator _rank = intervals_.begin();
            _rank != intervals_.end(); ++_rank) {
          qout.segment(r, _rank->second) = qin.segment(_rank->first, _rank->second);
          r += _rank->second;
        }
        assert (r == outputSize());
      }

    protected:
      /// Print path in a stream
      virtual std::ostream& print (std::ostream &os) const
      {
	os << "Dof Extracted Path:" << std::endl;
	os << "intervals: ";
        for (SizeIntervals_t::const_iterator _rank = intervals_.begin();
            _rank != intervals_.end(); ++_rank)
          os << "[ "  << _rank->first << ", " << _rank->second << "], " << std::endl;
	os << "original path:" << std::endl;
	os << *original_ << std::endl;
	return os;
      }

      /// Constructor
      ///
      /// \param original Path to extract,
      /// \param subInterval definition interval of the extracted path
      /// \note If subInterval.first is bigger than subInterval.second, then,
      /// the path is reversed.
      DofExtractedPath (const PathPtr_t& original, const SizeIntervals_t& intervals) :
	Path (original->timeRange(), intervalsToSize(intervals), outputSize ()),
	original_ (original), intervals_ (intervals),
        q_ (Configuration_t::Zero(original->outputSize()))
      {}

      DofExtractedPath (const DofExtractedPath& path) : Path (path),
						  original_ (path.original_),
                                                  intervals_ (path.intervals_),
                                                  q_ (path.q_),
						  weak_ ()
      {
      }

      DofExtractedPath (const DofExtractedPath& path,
		     const ConstraintSetPtr_t& constraints) :
	Path (path, constraints), original_ (path.original_),
	intervals_ (path.intervals_), weak_ ()
      {
      }

      void init (DofExtractedPathPtr_t self)
      {
	parent_t::init (self);
	weak_ = self;
      }

    private:
      PathPtr_t original_;
      SizeIntervals_t intervals_;
      mutable Configuration_t q_;
      DofExtractedPathWkPtr_t weak_;

      static size_type intervalsToSize(const SizeIntervals_t& ints)
      {
        size_type l = 0;
        for (SizeIntervals_t::const_iterator _rank = ints.begin();
            _rank != ints.end(); ++_rank)
          l += _rank->second;
        return l;
      }
    };
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_DOF_EXTRACTED_PATH_HH
