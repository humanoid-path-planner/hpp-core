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

      virtual ~ExtractedPath () throw () {}

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

      virtual PathPtr_t impl_extract (const interval_t& subInterval) const
        throw (projection_error)
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
        return (*original_)(reversed_?timeRange().second:timeRange().first,
              success);
      }

      /// Get the final configuration
      inline Configuration_t end () const
      {
	bool success;
        return (*original_)(reversed_?timeRange().first:timeRange().second,
              success);
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
    };
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_EXTRACTED_PATH_HH
