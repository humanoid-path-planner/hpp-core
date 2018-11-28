//
// Copyright (c) 2014,2015,2016,2018 CNRS
// Authors: Florent Lamiraux, Joseph Mirabel, Diane Bury
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

#ifndef HPP_CORE_CONTINUOUS_VALIDATION_INTERVAL_VALIDATION_HH
#define HPP_CORE_CONTINUOUS_VALIDATION_INTERVAL_VALIDATION_HH

#include <limits>
#include <iterator>

#include <boost/icl/continuous_interval.hpp>
#include <boost/icl/interval_set.hpp>

#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/collision.h>
#include <hpp/pinocchio/body.hh>
#include <hpp/pinocchio/collision-object.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/core/deprecated.hh>
#include <hpp/core/fwd.hh>

namespace hpp {
  namespace core {
    namespace continuousValidation {
      /// Computation of collision-free sub-intervals of a path
      ///
      /// This class aims at validating a path for the absence of collision
      /// between two bodies of a robot.
      ///
      /// The interval of definition of the path is successively covered
      /// by intervals where boths bodies are proved to be collision-free.
      /// Each interval is computed by bounding from above the velocity of
      /// all points of body 1 in the reference frame of body 2.
      template <typename ValidationReportTypePtr_t>
      class IntervalValidation
      {
      public:
        virtual bool validateConfiguration(const value_type &t, interval_t &interval,
                                  ValidationReportTypePtr_t &report,
                                  pinocchio::DeviceData& data) = 0;

        /// Set path to validate
        /// \param path path to validate,
        /// \param reverse whether path is validated from end to beginning.
        void path(const PathPtr_t &path, bool reverse)
        {
          path_ = path;
          reverse_ = reverse;
          valid_ = false;
          validInterval_ = interval_set();
          setupPath();
        }

        /// Get path
        PathConstPtr_t path() const
        {
          return path_;
        }

        value_type tolerance() const
        {
          return tolerance_;
        }

        virtual std::string name () const = 0;
        virtual std::ostream& print (std::ostream& os) const = 0;

      protected:
        typedef boost::icl::continuous_interval<value_type> continuous_interval;
        typedef boost::icl::interval_set<value_type> interval_set;
        PathPtr_t path_;
        value_type tolerance_;
        bool reverse_;
        bool refine_;
        bool valid_;
        interval_set validInterval_;
        /// Constructor of interval validation element
        ///
        /// \param tolerance allowed penetration should be positive
        IntervalValidation (value_type tolerance) : tolerance_(tolerance),
          reverse_(false), refine_(true)
        {
          if (tolerance < 0) {
            throw std::runtime_error ("tolerance should be non-negative.");
          }
        }

        IntervalValidation (const IntervalValidation& other) :
          tolerance_(other.tolerance_), refine_(true)
        {
          assert (tolerance_ > 0);
        }

      private:
        virtual void setupPath() = 0;
      }; // class IntervalValidation

      template <typename ValidationReportTypePtr_t>
      inline std::ostream &operator<<(std::ostream &os, const IntervalValidation<ValidationReportTypePtr_t> &b)
      {
        return b.print(os);
      }
    } // namespace continuousValidation
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_CONTINUOUS_VALIDATION_INTERVAL_VALIDATION_HH
