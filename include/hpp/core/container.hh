// Copyright (c) 2015, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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

#ifndef HPP_CORE_CONTAINER_HH
#define HPP_CORE_CONTAINER_HH

#include <hpp/core/config.hh>
#include <hpp/util/pointer.hh>
#include <map>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace hpp {
namespace core {
/// @cond INTERNAL
namespace internal {
template <typename T>
struct is_pointer : std::is_pointer<T> {};
template <typename T>
struct is_pointer<shared_ptr<T> > : std::true_type {};
template <typename T>
struct remove_pointer : std::remove_pointer<T> {};
template <typename T>
struct remove_pointer<shared_ptr<T> > {
  typedef T type;
};
template <typename T>
struct remove_pointer<const shared_ptr<T> > {
  typedef T type;
};

template <bool deref_ptr>
struct deref {
  template <typename T>
  static inline T get(T t) {
    return t;
  }
};
template <>
struct deref<true> {
  template <typename T>
  static inline typename remove_pointer<T>::type get(T t) {
    return *t;
  }
};
}  // namespace internal
/// @endcond

template <typename Types, typename Key = std::string>
struct Container {
  typedef std::map<Key, Types> Map_t;
  typedef typename Map_t::value_type value_type;
  typedef typename Map_t::key_type key_type;
  typedef typename Map_t::mapped_type mapped_type;
  typedef typename Map_t::const_iterator const_iterator;
  typedef typename Map_t::iterator iterator;

  Map_t map;

  /// Erase the element named name
  void erase(const Key& name) { map.erase(name); }
  /// Clear content of container
  void clear() { map.clear(); }
  /// Add an element
  void add(const key_type& name, const mapped_type& element) {
    std::pair<iterator, bool> ret = map.insert(value_type(name, element));
    if (!ret.second) ret.first->second = element;
  }
  /// Return the element named name
  bool has(const key_type& name) const { return (map.find(name) != map.end()); }

  /// Return the element named name
  const mapped_type& get(const key_type& name) const {
    const_iterator _e = map.find(name);
    if (_e == map.end()) {
      std::stringstream ss;
      ss << "Invalid key: " << name;
      throw std::invalid_argument(ss.str());
    }
    return _e->second;
  }

  /// Return the element named name
  const mapped_type& get(const key_type& name,
                         const mapped_type& defaultValue) const {
    const_iterator _e = map.find(name);
    if (_e == map.end()) return defaultValue;
    return _e->second;
  }

  /// Return a list of all elements
  /// \tparam ReturnType must have a push_back method.
  template <typename ReturnType>
  ReturnType getAllAs() const {
    ReturnType l;
    for (const_iterator _e = map.begin(); _e != map.end(); ++_e)
      l.push_back(_e->second);
    return l;
  }

  template <typename ReturnType>
  ReturnType getKeys() const {
    ReturnType l;
    for (const_iterator _e = map.begin(); _e != map.end(); ++_e)
      l.push_back(_e->first);
    return l;
  }

  /// Print object in a stream
  std::ostream& print(std::ostream& os) const {
    typedef internal::is_pointer<mapped_type> should_deref;
    typedef internal::deref<should_deref::value> deref;
    for (const_iterator _e = map.begin(); _e != map.end(); ++_e)
      os << _e->first << ": "
         << deref::template get<const mapped_type>(_e->second) << std::endl;
    return os;
  }
};
}  // namespace core
}  // namespace hpp
#endif  // HPP_CORE_CONTAINER_HH
