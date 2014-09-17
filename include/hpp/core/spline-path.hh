//
// Copyright (c) 2014 CNRS
// Authors: Florian Valenza
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

#ifndef HPP_CORE_SPLINE_PATH_HH
# define HPP_CORE_SPLINE_PATH_HH

# include <hpp/model/device.hh>
# include <hpp/core/fwd.hh>
# include <hpp/core/path.hh>
# include <hpp/core/path-vector.hh>
# include <boost/make_shared.hpp>

namespace hpp {
  namespace core {
    /// Concatenation of several paths
    class HPP_CORE_DLLAPI SplinePath : public Path
    {
    public:
      typedef Path parent_t;
      /// \name Construction, destruction, copy
      /// \{

      /// Create instance  and return shared pointer
      static SplinePathPtr_t create (interval_t tr, std::size_t outputSize, const vector_t& params, const std::string name = "cubic B-Spline")
      {
       SplinePath* ptr = new SplinePath (tr,outputSize,params,name);
       SplinePathPtr_t shPtr (ptr);
       ptr->init (shPtr);
       return shPtr;
     }
      /// Create instance from a PathVector and return shared pointer
      static SplinePathPtr_t create (const PathVectorPtr_t& pv)
      {
        // get the time range
        double t0 = pv->timeRange().first;
        double tf = pv->timeRange().second;
        interval_t tr = roboptim::CubicBSpline::makeInterval(t0,tf);
        
        // get the dimension
        size_t dim = pv->outputSize();

        // get the number of control points
        size_t nbCp = pv->paths().size()+1;

        // create param vector
        roboptim::CubicBSpline::vector_t params ( nbCp * dim);
        size_t i =0;
        Configuration_t current = (*pv)(pv->paths()[i]->timeRange().first);

        do {
          for (size_t j = 0; j < dim; ++j)
          {
            params[dim*i + j] = current[j];
          }
          i++;
          current= (*pv->paths()[i])(pv->paths()[i]->timeRange().second);
          
        } while (i < nbCp);

        // create the new splinePath with sharedPointer
        SplinePath* ptr = new SplinePath (tr,dim,params,"cubic B-Spline");
        SplinePathPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;

     }
     
     /// Create instance from a CubicBSpline and return a shared pointer
     static SplinePathPtr_t create(const roboptim::CubicBSpline& spline)
     {
       SplinePath* ptr = new SplinePath (spline.timeRange(),spline.outputSize(),spline.parameters(),"cubic-b-spline");
       SplinePathPtr_t shPtr (ptr);
       ptr->init (shPtr);
       return shPtr;
	 }

      /// Create instance and return shared pointer
     static SplinePathPtr_t createCopy (const SplinePath& original)
     {
      SplinePath* ptr = new SplinePath (original);
      SplinePathPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

      /// Return a shared pointer to a copy of this
     virtual PathPtr_t copy () const
     {
       return createCopy (*this);
     }

      /// Destructor
     virtual ~SplinePath () throw ()
     {
     }
      /// \}
      
      /// Get the associated spline
      SplinePtr_t spline(){
		  return spline_;
	  }

	// TODO
	virtual std::ostream& print (std::ostream &os) const
		{
		os << "SplinePath:" << std::endl;
		os << "interval: [ " << timeRange ().first << ", "
		   << timeRange ().second << " ]" << std::endl;
		return os;
		}
		
      /// Constructor from parameters needed for a spline
     SplinePath (interval_t tr, std::size_t outputSize, const vector_t& params, const std::string name = "cubic B-Spline") :
      parent_t (tr, outputSize)
     {
      spline_ = boost::make_shared<roboptim::CubicBSpline> (tr,outputSize,params,name);

     }

      ///Copy constructor
     SplinePath (const SplinePath& path) :
      parent_t (path)
     {
     }

     void init (SplinePathPtr_t self)
     {
       parent_t::init (self);
       weak_ = self;
     }
     virtual void impl_compute (ConfigurationOut_t result, value_type t) const;

   private:
    SplinePtr_t spline_;
      SplinePathWkPtr_t weak_; //TODO WARINING Should be SplinePathWkPtr_t weak_  ..... ou le mettre.
    }; // class SplinePath
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_SPLINE_PATH_HH
