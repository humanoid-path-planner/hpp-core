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

#ifndef HPP_CORE_WEIGHED_DISTANCE_HH
# define HPP_CORE_WEIGHED_DISTANCE_HH

# include <hpp/core/distance.hh>

namespace hpp {
  namespace core {
    /// \addtogroup steering_method
    /// \{

    /// Weighed distance between configurations
    ///
    /// Euclidean distance between configurations seen as vectors.
    /// Each degree of freedom is weighed by a positive value.
    class HPP_CORE_DLLAPI WeighedDistance : public Distance {
    public:
      static WeighedDistancePtr_t createFromProblem
	(const Problem& problem);
      static WeighedDistancePtr_t create (const DevicePtr_t& robot);
      static WeighedDistancePtr_t
        createWithWeight (const DevicePtr_t& robot, const vector_t& weights);
      static WeighedDistancePtr_t createCopy
	(const WeighedDistancePtr_t& distance);
      virtual DistancePtr_t clone () const;
      /// Get weight of joint at given rank
      /// \param rank rank of the joint in robot joint vector
      value_type getWeight( size_type rank ) const;
      /// Set weight of joint at given rank
      /// \param rank rank of the joint in robot joint vector
      void setWeight(size_type rank, value_type weight);
      /// Get weights
      const vector_t& weights() const;
      /// Set weights
      void weights(const vector_t& ws);
      /// Get size of weight vector
      size_type size () const
      {
	return weights_.size ();
      }

      /// Get robot
      const DevicePtr_t& robot () const
      {
	return robot_;
      }
    protected:
      WeighedDistance (const Problem& problem);
      WeighedDistance (const DevicePtr_t& robot);
      WeighedDistance (const DevicePtr_t& robot, const vector_t& weights);
      WeighedDistance (const WeighedDistance& distance);
      void init (WeighedDistanceWkPtr_t self);
      /// Derived class should implement this function
      virtual value_type impl_distance (ConfigurationIn_t q1,
				    ConfigurationIn_t q2) const;
    private:
      void computeWeights ();
      DevicePtr_t robot_;
      vector_t weights_;
      WeighedDistanceWkPtr_t weak_;
    }; // class WeighedDistance
    /// \}
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_WEIGHED_DISTANCE_HH
