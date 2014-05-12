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

#ifndef HPP_CORE_PATH_VECTOR_HH
# define HPP_CORE_PATH_VECTOR_HH

# include <hpp/model/device.hh>
# include <hpp/core/fwd.hh>
# include <hpp/core/path.hh>

namespace hpp {
  namespace core {
    /// Concatenation of several paths
    class HPP_CORE_DLLAPI PathVector : public Path
    {
    public:
      typedef Path parent_t;
      /// \name Construction, destruction, copy
      /// \{

      /// Create instance and return shared pointer
      static PathVectorPtr_t create (std::size_t outputSize)
      {
	PathVector* ptr = new PathVector (outputSize);
	PathVectorPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      /// Create instance and return shared pointer
      static PathVectorPtr_t createCopy (const PathVector& original)
      {
	PathVector* ptr = new PathVector (original);
	PathVectorPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      /// Return a shared pointer to a copy of this
      virtual PathPtr_t copy () const
      {
	return createCopy (*this);
      }
      
      /// Destructor
      virtual ~PathVector () throw ()
      {
      }
      /// \}

      /// Get the number of sub paths
      std::size_t numberPaths () const
      {
	return paths_.size ();
      }

      /// Get a path in the vector
      const PathPtr_t& pathAtRank (std::size_t rank) const {
	return paths_ [rank];
      }

      /// Get rank of direct path in vector at param
      ///
      /// \param param parameter in interval of definition,
      /// \retval localParam parameter on sub-path
      /// \return rank of direct path in vector
      std::size_t rankAtParam (const value_type& param, value_type& localParam) const;

      /// Append a path at the end of the vector
      void appendPath (const PathPtr_t& path);

      /// Concatenate two vectors of path
      void concatenate (const PathVector& path);

      /// Extraction of a sub-path
      /// \param subInterval interval of definition of the extract path
      virtual PathPtr_t extract (const interval_t& subInterval) const;

    protected:
      /// Print path in a stream
      virtual std::ostream& print (std::ostream &os) const
      {
	os << "PathVector:" << std::endl;
	os << "interval: [ " << timeRange ().first << ", "
	   << timeRange ().second << " ]" << std::endl;
	for (Paths_t::const_iterator itPath = paths_.begin ();
	     itPath != paths_.end (); ++itPath) {
	  os << (**itPath) << std::endl;
	}
	return os;
      }
      /// Constructor
      PathVector (std::size_t outputSize) : parent_t (std::make_pair (0, 0),
						      outputSize),
	paths_ ()
	  {
	  }
      ///Copy constructor
      PathVector (const PathVector& path) : parent_t (path),
	paths_ ()
	  {
	    timeRange_ = path.timeRange_;
	    for (Paths_t::const_iterator it = path.paths_.begin ();
		 it != path.paths_.end (); it++) {
	      paths_.push_back ((*it)->copy ());
	    }
	  }

      void init (PathVectorPtr_t self)
      {
	parent_t::init (self);
	weak_ = self;
      }
      virtual void impl_compute (ConfigurationOut_t result, value_type t) const;

    private:
      Paths_t paths_;
      PathVectorWkPtr_t weak_;
    }; // class PathVector
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_PATH_VECTOR_HH
