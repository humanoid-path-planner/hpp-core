// 
// Copyright (c) 2014 CNRS
// Authors: Florian Valenza
//
// This file is part of the hpp-core.
//
// roboptim is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// roboptim is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with roboptim.  If not, see <http://www.gnu.org/licenses/>.

#ifndef HPP_CORE_SPLINE_COST_HH
# define HPP_CORE_SPLINE_COST_HH
# include <roboptim/trajectory/sys.hh>

# include <boost/optional/optional_fwd.hpp>

# include <roboptim/trajectory/fwd.hh>
# include <roboptim/trajectory/cubic-b-spline.hh>
# include <roboptim/trajectory/trajectory-cost.hh>

# include <hpp/model/device.hh>
# include <hpp/core/fwd.hh>

// Variable a changer si on veut changer de fonction cout (debug)

#define basicCost
//#define quadraticError
//#define obstacleAvoidance  

// ---------------------------------------------  BASIC COST ------------------------------------------------------------- //
// ----------------------------------------------------------------------------------------------------------------------- //
  #ifdef basicCost
namespace hpp {
	namespace core{

  /// \brief Approximate the cost of a Spline.
  ///
  /// The cost is computed using:
  ///
  class SplineCost : public roboptim::TrajectoryCost<CubicBSpline_t>
  {
  public:
    /// Construct the function from a Spline and a definition interval.
    ///
    /// The interval allows to only compute the length of the Spline
    /// on a specific interval. The step associated to the interval controls
    /// the approximation precision.
    /// \param spline spline used for length computation
    /// \param interval interval on which the length is computed.
    /// \param nDiscretizationPoints number of discretization points
    SplineCost (const CubicBSpline_t& spline,
		  const size_type nDiscretizationPoints = 100,
		  boost::optional<interval_t> interval = boost::none_t ());

    ~SplineCost () throw();

  protected:
    void impl_compute (result_t&, const argument_t&) const;
    void impl_gradient (gradient_t&, const argument_t&, size_type) const;

  private:
    /// \brief Interval on which the length is computed.
    interval_t interval_;
    /// \brief Number of discretization points.
    size_type nDiscretizationPoints_;
  };

  } //   namespace core
} // namespace hpp




// --------------------------------------------- QUADRATIC ERROR ---------------------------------------------------------- //
// ----------------------------------------------------------------------------------------------------------------------- //
  #elif defined quadraticError

namespace hpp {
	namespace core{

  /// \brief Approximate the cost of a Spline.
  ///
  /// The cost is computed using:
  ///
  class SplineCost : public roboptim::TrajectoryCost<CubicBSpline_t>
  {
  public:
    /// Construct the function from a Spline and a definition interval.
    ///
    /// The interval allows to only compute the length of the Spline
    /// on a specific interval. The step associated to the interval controls
    /// the approximation precision.
    /// \param spline spline used for length computation
    /// \param interval interval on which the length is computed.
    /// \param nDiscretizationPoints number of discretization points
    SplineCost (const CubicBSpline_t& spline,
		  const PathVectorPtr_t& straightPath,
		  const size_type nDiscretizationPoints = 100,
		  boost::optional<interval_t> interval = boost::none_t ());
  
    ~SplineCost () throw();

  protected:
    void impl_compute (result_t&, const argument_t&) const;
    void impl_gradient (gradient_t&, const argument_t&, size_type) const;

  private:
    /// \brief Interval on which the length is computed.
    interval_t interval_;
    /// \brief Number of discretization points.
    size_type nDiscretizationPoints_;
    /// \brief Original broken line path
    PathVectorPtr_t blPath_;
    
  };

  } //   namespace core
} // namespace hpp





// ---------------------------------------------  OSTACLE AVOIDANCE ----------------------------------------------------------- //
// ----------------------------------------------------------------------------------------------------------------------- //
  #elif defined obstacleAvoidance
  
  #define ALPHA 2
  #define DI 0.3
  #define EPS 0.0001
namespace hpp {
	namespace core{

  /// \brief Approximate the cost of a Spline.
  ///
  /// The cost is computed using:
  ///
  class SplineCost : public roboptim::TrajectoryCost<CubicBSpline_t>
  {
  public:
    /// Construct the function from a Spline and a definition interval.
    ///
    /// The interval allows to only compute the length of the Spline
    /// on a specific interval. The step associated to the interval controls
    /// the approximation precision.
    /// \param spline spline used for length computation
    /// \param interval interval on which the length is computed.
    /// \param nDiscretizationPoints number of discretization points
    SplineCost (const CubicBSpline_t& spline,
		  const DevicePtr_t& device,
		  const PathVectorPtr_t& straightPath,
		  const size_type nDiscretizationPoints = 100,
		  boost::optional<interval_t> interval = boost::none_t ());
  
    ~SplineCost () throw();

  protected:
    void impl_compute (result_t&, const argument_t&) const;
    void impl_gradient (gradient_t&, const argument_t&, size_type) const;

  private:
    /// \brief Interval on which the length is computed.
    interval_t interval_;
    /// \brief Number of discretization points.
    size_type nDiscretizationPoints_;
    /// \brief Original broken line path
    const PathVectorPtr_t& blPath_;
    /// \brief Ptr on device associated to the spline
    DevicePtr_t device_;
  };

  } //   namespace core
} // namespace hpp
  #endif
#endif //! HPP_CORE_SPLINE_COST_HH
