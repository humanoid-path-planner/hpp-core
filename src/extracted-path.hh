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

#ifndef HPP_CORE_EXTRACTED_PATH_HH
# define HPP_CORE_EXTRACTED_PATH_HH

# include <hpp/core/path.hh>

namespace hpp {
  namespace core {
    /// Result of restriction of a path to a sub interval or of reversion
    /// \note Decorator design pattern
    class ExtractedPath : public Path
    {
    public:
      typedef Path parent_t;

      virtual ~ExtractedPath () {}

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

      static ExtractedPathPtr_t
      create (const PathPtr_t& original, const interval_t& subInterval)
      {
	ExtractedPath* ptr = new ExtractedPath (original, subInterval);
	ExtractedPathPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      static ExtractedPathPtr_t
      createCopy (const ExtractedPathPtr_t& path)
      {
	ExtractedPath* ptr = new ExtractedPath (*path);
	ExtractedPathPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      static ExtractedPathPtr_t
      createCopy (const ExtractedPathPtr_t& path,
		  const ConstraintSetPtr_t& constraints)
      {
	ExtractedPath* ptr = new ExtractedPath (*path, constraints);
	ExtractedPathPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      virtual bool impl_compute (ConfigurationOut_t result,
				 value_type param) const
      {
        param = sInOriginalPath (param);
        return original_->impl_compute (result, param);
      }

      virtual void impl_derivative (vectorOut_t result, const value_type& s,
				    size_type order) const
      {
	if (reversed_) {
	  value_type param = (paramRange ().second - s);
	  original_->impl_derivative (result, param, order);
	  if (order % 2 == 1) result *= -1.;
	} else {
	  original_->impl_derivative (result, s, order);
	}
      }

      void impl_velocityBound (vectorOut_t result, const value_type& t0, const value_type& t1) const override
      {
	value_type tmin = sInOriginalPath (t0),
                   tmax = sInOriginalPath (t1);
	if (tmin > tmax) std::swap (tmin, tmax);
        original_->velocityBound(result, tmin, tmax);
      }

      virtual PathPtr_t impl_extract (const interval_t& subInterval) const
      {
	ExtractedPathPtr_t path = createCopy (weak_.lock ());
	value_type tmin = sInOriginalPath (subInterval.first),
                   tmax = sInOriginalPath (subInterval.second);
	path->reversed_ = tmin > tmax;
	if (path->reversed_) std::swap (tmin, tmax);
	// path->reversed_ = ((this->reversed_) && (!reversed)) ||
	  // ((!this->reversed_) && (reversed));
	path->timeParameterization (
            TimeParameterizationPtr_t(), std::make_pair (tmin, tmax));
	assert (path->timeRange().first >= timeRange ().first -
		std::numeric_limits <float>::epsilon ());
	assert (path->timeRange().second <= timeRange ().second +
		std::numeric_limits <float>::epsilon ());
	return path;
      }

      /// Get the initial configuration
      inline Configuration_t initial () const
      {
	bool success;
        return original_->eval(reversed_?timeRange().second:
			       timeRange().first, success);
      }

      /// Get the final configuration
      inline Configuration_t end () const
      {
	bool success;
        return original_->eval(reversed_?timeRange().first:
			       timeRange().second, success);
      }

    protected:
      /// Print path in a stream
      virtual std::ostream& print (std::ostream &os) const
      {
	os << "Extracted Path:" << std::endl;
        Path::print (os);
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
      ExtractedPath (const PathPtr_t& original, const interval_t& subInterval) :
	Path (std::pair <value_type, value_type> (0,0), original->outputSize (),
	      original->outputDerivativeSize (), original->constraints ()),
	original_ (original)
      {
	reversed_ = subInterval.first <= subInterval.second ? false : true;
        interval_t tr;
        if (reversed_) {
          tr.first = subInterval.second;
	  tr.second = subInterval.first;
	} else {
	  tr = subInterval;
	}
        timeRange (tr);
	assert (timeRange().first >= original->timeRange ().first);
	assert (timeRange().second <= original->timeRange ().second);
      }

      ExtractedPath (const ExtractedPath& path) : Path (path),
						  original_ (path.original_),
						  reversed_ (path.reversed_),
						  weak_ ()
      {
      }

      ExtractedPath (const ExtractedPath& path,
		     const ConstraintSetPtr_t& constraints) : 
	Path (path, constraints), original_ (path.original_),
	reversed_ (path.reversed_), weak_ ()
      {
      }

      void init (ExtractedPathPtr_t self)
      {
	parent_t::init (self);
	weak_ = self;
      }

      /// For serialization only.
      ExtractedPath() {}

    private:
      inline value_type sInOriginalPath (const value_type& s) const
      {
        assert (paramRange().first <= s && s <= paramRange().second);
        if (!reversed_) return s;
        return paramRange().second - s;
      }

      PathPtr_t original_;
      bool reversed_;
      ExtractedPathWkPtr_t weak_;

      HPP_SERIALIZABLE();
    };
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_EXTRACTED_PATH_HH
