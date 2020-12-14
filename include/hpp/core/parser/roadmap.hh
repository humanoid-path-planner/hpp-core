// Copyright (c) 2020 CNRS
// Authors: Joseph Mirabel
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
