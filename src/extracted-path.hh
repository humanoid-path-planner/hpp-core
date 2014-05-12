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

      virtual void impl_compute (ConfigurationOut_t result,
				 value_type param) const
      {
	if (reversed_) {
	  param = timeRange ().first + timeRange ().second - param ;
	  (*original_) (result, param);
	} else {
	  (*original_) (result, param);
	}
      }

      virtual PathPtr_t extract (const interval_t& subInterval) const
      {
	ExtractedPathPtr_t path = createCopy (weak_.lock ());
	bool reversed = subInterval.first > subInterval.second ? true : false;
	value_type tmin, tmax;
	if (reversed) {
	  tmin = subInterval.second; tmax = subInterval.first;
	} else {
	  tmin = subInterval.first; tmax = subInterval.second;
	}
	path->reversed_ = ((this->reversed_) && (!reversed)) ||
	  ((!this->reversed_) && (reversed));
	path->timeRange_ = std::make_pair (tmin, tmax);
	assert (path->timeRange_.first >= timeRange ().first);
	assert (path->timeRange_.second <= timeRange ().second);
	return path;
      }

    protected:
      /// Print path in a stream
      virtual std::ostream& print (std::ostream &os) const
      {
	os << "Extracted Path:" << std::endl;
	os << "interval: [ " << timeRange ().first << ", "
	   << timeRange ().second << " ]" << std::endl;
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
	      original->constraints ()),
	original_ (original)
      {
	reversed_ = subInterval.first <= subInterval.second ? false : true;
	if (reversed_) {
	  timeRange_.first = subInterval.second;
	  timeRange_.second = subInterval.first;
	} else {
	  timeRange_ = subInterval;
	}
	assert (timeRange_.first >= original->timeRange ().first);
	assert (timeRange_.second <= original->timeRange ().second);
      }

      ExtractedPath (const ExtractedPath& path) : Path (path),
						  original_ (path.original_),
						  reversed_ (path.reversed_),
						  weak_ ()
      {
      }

      void init (ExtractedPathPtr_t self)
      {
	parent_t::init (self);
	weak_ = self;
      }

    private:
      PathPtr_t original_;
      bool reversed_;
      ExtractedPathWkPtr_t weak_;
    };
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_EXTRACTED_PATH_HH
