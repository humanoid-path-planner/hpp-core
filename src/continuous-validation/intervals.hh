//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#ifndef HPP_CORE_CONTINUOUS_VALIDATION_INTERVALS_HH
#define HPP_CORE_CONTINUOUS_VALIDATION_INTERVALS_HH

#include <hpp/core/fwd.hh>
#include <list>
#include <ostream>

namespace hpp {
namespace core {
namespace continuousValidation {
inline bool lastElement(const std::list<interval_t>& list,
                        const std::list<interval_t>::iterator& it) {
  std::list<interval_t>::iterator it1 = it;
  ++it1;
  if (it1 == list.end()) return true;
  return false;
}
/// Union of intervals
class Intervals {
 public:
  /// Reset to empty set
  void clear() { intervals_.clear(); }
  /// Union of this with an interval
  void unionInterval(const interval_t& interval) {
    std::list<std::list<interval_t>::iterator> toErase;
    unsigned int cas = 0;
    std::list<interval_t>::iterator it = intervals_.begin();
    for (; it != intervals_.end(); ++it) {
      if (interval.first < it->first) {
        cas = 1;
        break;
      } else if (interval.first <= it->second) {
        cas = 2;
        break;
      }
    }
    if (cas == 0) {
      intervals_.push_back(interval);
    } else if (cas == 1) {
      // intervals_ |------------|       |=== *it ===|   |---------|
      // interval                    |------------------------------
      for (; it != intervals_.end(); ++it) {
        if (interval.second < it->first) {
          // intervals_ ---|     |xxxxxxxx|      |=== *it ===|   |-------|
          // interval        |----------------|
          intervals_.insert(it, interval);
          break;
        } else if (interval.second < it->second) {
          // intervals_ ---|     |xxxxxx|      |=== *it ===|   |---------|
          // interval        |---------------------|
          it->first = interval.first;
          break;
        } else {
          // intervals_ ---|     |=== *it ===|     |--------|   |--------|
          // interval        |------------------|
          toErase.push_back(it);
        }
      }
      if (it == intervals_.end()) {
        intervals_.push_back(interval);
      }
    } else if (cas == 2) {
      // intervals_ |==== *it ====|       |-------|   |---------|
      // interval             |----------------------------------
      for (std::list<interval_t>::iterator it1 = it; it1 != intervals_.end();
           ++it1) {
        if (interval.second < it1->first) {
          // intervals_ |==== *it ====|  |-----|   |=== *it1 ===|
          // interval             |--------------|
          it->second = interval.second;
        } else if (interval.second < it1->second) {
          // intervals_ |==== *it ====|  |-----|   |=== *it1 ===|
          // interval             |-------------------|
          it->second = it1->second;
          // Remove it1 if different from it
          if (it1 != it) {
            toErase.push_back(it1);
          }
          break;
        } else if (lastElement(intervals_, it1)) {
          it->second = interval.second;
          if (it1 != it) {
            toErase.push_back(it1);
          }
          break;
        } else {
          // intervals_ |==== *it ====|  |xxxxx|   |=== *it1 ===|
          // interval             |------------------------------------|
          // Remove it1 if different from it
          if (it1 != it) {
            toErase.push_back(it1);
          }
        }
      }
    } else {
      abort();
    }
    for (std::list<std::list<interval_t>::iterator>::iterator itErase =
             toErase.begin();
         itErase != toErase.end(); ++itErase) {
      intervals_.erase(*itErase);
    }
  }

  bool contains(const interval_t& interval) const {
    for (std::list<interval_t>::const_iterator it = intervals_.begin();
         it != intervals_.end(); ++it) {
      if ((it->first <= interval.first) && (interval.second <= it->second)) {
        return true;
      }
    }
    return false;
  }

  bool contains(const value_type& value, bool reverse = false) const {
    if (reverse) {
      for (std::list<interval_t>::const_iterator it = intervals_.begin();
           it != intervals_.end(); ++it) {
        if ((it->first <= value) && (value <= it->second)) {
          return true;
        }
      }
    } else {
      for (std::list<interval_t>::const_reverse_iterator it =
               intervals_.rbegin();
           it != intervals_.rend(); ++it) {
        if ((it->first <= value) && (value <= it->second)) {
          return true;
        }
      }
    }
    return false;
  }

  const std::list<interval_t>& list() const { return intervals_; }

  std::ostream& print(std::ostream& os) const {
    os << "Intervals: " << std::endl;
    for (std::list<interval_t>::const_iterator it = intervals_.begin();
         it != intervals_.end(); ++it) {
      os << "[" << it->first << ", " << it->second << "]" << std::endl;
    }
    return os;
  }

  std::string toString() const {
    std::ostringstream oss;
    print(oss);
    return oss.str();
  }

 private:
  std::list<interval_t> intervals_;
};  // class Intervals

inline std::ostream& operator<<(std::ostream& os, const Intervals& o) {
  return o.print(os);
}
}  // namespace continuousValidation
}  // namespace core
}  // namespace hpp
#endif  // HPP_CORE_CONTINUOUS_VALIDATION_INTERVALS_HH
