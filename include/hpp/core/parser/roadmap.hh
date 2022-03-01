// Copyright (c) 2020 CNRS
// Authors: Joseph Mirabel
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

#ifndef HPP_CORE_PARSER_ROADMAP_HH
# define HPP_CORE_PARSER_ROADMAP_HH

# include <fstream>

# include <hpp/core/roadmap.hh>
# include <hpp/pinocchio/serialization.hh>

namespace hpp {
namespace core {
namespace parser {
namespace internal {

template<typename Parent, typename Child>
struct InsertChildClass : std::pair<std::string, Child*>
{
  using std::pair<std::string, Child>::pair;
};

template<class Archive, class A>
inline void insert(Archive& ar, const std::pair<std::string, A*>& a)
{
  ar.insert(a.first, a.second);
}

template<class Archive, class Parent, class Child>
inline void insert(Archive& ar, const InsertChildClass<Parent, Child>& a)
{
  ar.template insertChildClass<Parent, Child>(a.first, a.second);
}

template<class Archive> inline void inserts(Archive&) {}
template<class Archive, class A, class... B>
inline void inserts(Archive& ar, A& a, B&... b)
{
  insert(ar, a);
  inserts(ar, b...);
}
} // namespace internal

/// \addtogroup roadmap
/// \{
/// \tparam Archive an archive from hpp::serialization::(xml|binary|text)_(i|o)archive.
///         The only requirement is that it derives from hpp::serialization::archive_ptr_holder.
/// \tparam Args a list of std::pair<std::string, Object*> or InsertChildClass
/// \param roadmap the roadmap to save or fill.
/// \param filename the file to read or write.
/// \param args indicate the name and object pointer to insert in the archive pointer holder.
template<class A>
std::pair<std::string, A*> make_nvp (const std::string& n, A* a)
{ return std::pair<std::string, A*> (n, a); }
template<class Parent, class Child>
internal::InsertChildClass<Parent, Child> make_nvp_with_parent (const std::string& n, Child* a)
{ return internal::InsertChildClass<Parent, Child> (n, a); }

template<class Archive, class... Args>
void serializeRoadmap (RoadmapPtr_t& roadmap, const std::string& filename, Args... args)
{
  typename std::conditional<Archive::is_saving::value,
           std::ofstream, std::ifstream>::type
    fs (filename);
  Archive ar (fs);
  internal::inserts(ar, args...);
  ar.initialize();
  ar & hpp::serialization::make_nvp("roadmap", roadmap);
}
/// \}
} // namespace parser
} // namespace core
} // namespace hpp
#endif // HPP_CORE_PARSER_ROADMAP_HH
