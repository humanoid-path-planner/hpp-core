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
      /// Validation of a parameter interval of a path
      ///
      /// During path planning, some criteria need to be checked to be
      /// valid over the whole interval of definition of a path.
      /// The most common criterion is the absence of collision (implemented
      /// by derived class BodyPairCollision), but some other criteria might
      /// need to be checked. For instance the tension of the cables of a
      /// parallel cable driven robot need to remain in an interval.
      ///
      /// This class provides a common interface for continuous validation
      /// through method \link IntervalValidation::validateConfiguration
      /// validateConfiguration \endlink.
      ///
      /// A \link IntervalValidation::tolerance tolerance \endlink
      /// may be provided at construction. An interval will be
      /// considered as valid if the criterion is violated by less than the
      /// tolerance. This parameter interpretation is left to the
      /// specialization designers.
      class IntervalValidation
      {
      public:
        /// Validate an interval for a given criterion
        /// \param t center of the interval to validate
        /// \param interval over which the criterion should be checked,
        /// \retval interval part of the input interval that is valid,
        /// \retval report report in case of non validity of the configuration
        ///         at parameter t
        /// \param data data resulting from forward kinematics computed at
        ///        parameter t.
        virtual bool validateConfiguration(const value_type &t, interval_t &interval,
                                  ValidationReportPtr_t &report,
                                  const pinocchio::DeviceData& data) = 0;

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
        virtual IntervalValidationPtr_t copy () const = 0;

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
          if (tolerance_ < 0) {
            throw std::runtime_error ("tolerance should be non-negative.");
          }
        }

      private:
        virtual void setupPath() = 0;
      }; // class IntervalValidation

      inline std::ostream &operator<<(std::ostream &os,
                                      const IntervalValidation& b)
      {
        return b.print(os);
      }
    } // namespace continuousValidation
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_CONTINUOUS_VALIDATION_INTERVAL_VALIDATION_HH
