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

#ifndef HPP_CORE_VALIDATION_REPORT_HH
# define HPP_CORE_VALIDATION_REPORT_HH

# include <hpp/core/config.hh>
# include <hpp/core/fwd.hh>

namespace hpp {
  namespace core {
    /// \addtogroup validation
    /// \{

    /// Abstraction of validation report for paths and configurations
    ///
    /// This class is aimed at being derived to store information relative to
    /// various Validation derived classes
    /// \li CollisionValidation,
    /// \li collision related PathValidation classes.
    class HPP_CORE_DLLAPI ValidationReport
    {
    public:
      virtual ~ValidationReport ()
      {
      }
      /// Write report in a stream
      virtual std::ostream& print (std::ostream& os) const = 0;
    }; // class ValidationReport
    inline std::ostream& operator<< (std::ostream& os,
				     const ValidationReport& report)
    {
      return report.print (os);
    }
    /// \}
  } // namespace core
} // namespace hpp

#endif // HPP_CORE_VALIDATION_REPORT_HH
