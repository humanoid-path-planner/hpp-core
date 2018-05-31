// Copyright (c) 2015, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-core.
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
// hpp-core. If not, see <http://www.gnu.org/licenses/>.

#ifndef HPP_CORE_CONTAINER_HH
# define HPP_CORE_CONTAINER_HH

# include <map>
# include <list>
# include <stdexcept>

# include <boost/smart_ptr/shared_ptr.hpp>

# include <boost/mpl/inherit_linearly.hpp>
# include <boost/mpl/fold.hpp>
# include <boost/mpl/inherit.hpp>
# include <boost/mpl/vector.hpp>
# include <boost/mpl/for_each.hpp>
# include <boost/type_traits/is_pointer.hpp>
# include <boost/type_traits/remove_pointer.hpp>

# include <hpp/core/config.hh>

namespace hpp {
  namespace core {
    /// @cond INTERNAL
    namespace internal {
      template <typename T> struct is_pointer : boost::is_pointer<T> {};
      template <typename T> struct is_pointer<boost::shared_ptr<T> > : boost::true_type {};
      template <typename T> struct remove_pointer : boost::remove_pointer<T> {};
      template <typename T> struct remove_pointer<boost::shared_ptr<T> > { typedef T type; };
      template <typename T> struct remove_pointer<const boost::shared_ptr<T> > { typedef T type; };

      template <bool deref_ptr> struct deref {
        template <typename T> static inline T get (T t) { return t; }
      };
      template <> struct deref <true> {
        template <typename T> static inline typename remove_pointer<T>::type get (T t) { return *t; }
      };
    }
    /// @endcond

    template <typename Types, typename Key = std::string > struct Container
    {
      typedef std::map <Key, Types> Map_t;
      typedef typename Map_t::value_type value_type;
      typedef typename Map_t::key_type key_type;
      typedef typename Map_t::mapped_type mapped_type;
      typedef typename Map_t::const_iterator const_iterator;
      typedef typename Map_t::      iterator       iterator;

      Map_t map;

      /// Erase the element named name
      void erase (const Key& name) { map.erase (name); }
      /// Clear content of container
      void clear () { map.clear (); }
      /// Add an element
      void add (const key_type& name, const mapped_type& element)
      {
        std::pair<iterator, bool> ret = map.insert( value_type(name, element));
        if (!ret.second)
          ret.first->second = element;
      }
      /// Return the element named name
      bool has (const key_type& name) const { return (map.find (name) != map.end ()); }

      /// Return the element named name
      const mapped_type& get (const key_type& name) const
      {
        const_iterator _e = map.find (name);
        if (_e == map.end ()) {
          std::stringstream ss; ss << "Invalid key: " << name;
          throw std::invalid_argument (ss.str());
        }
        return _e->second;
      }

      /// Return the element named name
      const mapped_type& get (const key_type& name, const mapped_type& defaultValue) const
      {
        const_iterator _e = map.find (name);
        if (_e == map.end ()) return defaultValue;
        return _e->second;
      }

      /// Return a list of all elements
      /// \tparam ReturnType must have a push_back method.
      template <typename ReturnType>
        ReturnType getAllAs () const
        {
          ReturnType l;
          for (const_iterator _e = map.begin (); _e != map.end (); ++_e)
            l.push_back (_e->second);
          return l;
        }

      template <typename ReturnType>
        ReturnType getKeys () const
        {
          ReturnType l;
          for (const_iterator _e = map.begin (); _e != map.end (); ++_e)
            l.push_back (_e->first);
          return l;
        }

      /// Print object in a stream
      std::ostream& print (std::ostream& os) const
      {
        typedef internal::is_pointer<mapped_type> should_deref;
        typedef internal::deref<should_deref::value> deref;
        for (const_iterator _e = map.begin (); _e != map.end (); ++_e)
          os << _e->first << ": "
            << deref::template get<const mapped_type> (_e->second)
            << std::endl;
        return os;
      }
    };
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_CONTAINER_HH
