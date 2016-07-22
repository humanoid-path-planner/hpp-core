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

#ifndef HPP_CORE_KINODYNAMIC_DISTANCE_HH
# define HPP_CORE_KINODYNAMIC_DISTANCE_HH

# include <hpp/core/distance.hh>

namespace hpp {
namespace core {
/// \addtogroup steering_method
/// \{

/// Weighed distance between configurations
///
/// Euclidean distance between configurations seen as vectors.
/// Each degree of freedom is weighed by a positive value.
class HPP_CORE_DLLAPI KinodynamicDistance : public Distance {
public:
    static KinodynamicDistancePtr_t create (const DevicePtr_t& robot);
    static KinodynamicDistancePtr_t createCopy
    (const KinodynamicDistancePtr_t& distance);
    virtual DistancePtr_t clone () const;


    /// Get robot
    const DevicePtr_t& robot () const
    {
        return robot_;
    }
protected:
    KinodynamicDistance (const DevicePtr_t& robot);
    KinodynamicDistance (const KinodynamicDistance& distance);
    void init (KinodynamicDistanceWkPtr_t self);
    /// Derived class should implement this function
    virtual value_type impl_distance (ConfigurationIn_t q1,
                                      ConfigurationIn_t q2) const;

    double computeMinTime(double p1, double p2, double v1, double v2)const ;


private:
    DevicePtr_t robot_;
    double aMax_;
    double vMax_;
    KinodynamicDistanceWkPtr_t weak_;
}; // class KinodynamicDistance
/// \}
} //   namespace core
} // namespace hpp
#endif // HPP_CORE_WEIGHED_DISTANCE_HH
