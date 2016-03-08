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

# include <boost/smart_ptr/shared_ptr.hpp>

# include <boost/mpl/inherit_linearly.hpp>
# include <boost/mpl/fold.hpp>
# include <boost/mpl/inherit.hpp>
# include <boost/mpl/vector.hpp>
# include <boost/mpl/for_each.hpp>

# include <hpp/core/config.hh>

namespace hpp {
  namespace core {
    /// @cond INTERNAL
    namespace internal {
      template <typename Key, typename Type>
        struct field
        {
          typedef std::map <Key, Type> Map_t;
          Map_t map;

          template <bool deref_ptr>
            void print (std::ostream& os) const
            {
              for (typename Map_t::const_iterator
                  it = map.begin (); it != map.end (); ++it)
              {
                if (deref_ptr) os << it->first << " : " << *it->second << std::endl;
                else           os << it->first << " : " <<  it->second << std::endl;
              }
            }
        };

      template <typename Types, typename Key>
        struct container_traits
        {
          typedef typename boost::mpl::inherit_linearly <
            Types, boost::mpl::inherit < boost::mpl::arg<1>,
                                         field< Key, boost::mpl::arg<2> >
                                       >
              >::type
              type;
        };

      template <bool deref_ptr>
        struct print {
          std::ostream& os;

          print (std::ostream& osio) : os (osio) {}

          template <class Field>
            void operator () (const Field& f)
          {
            f.Field::template print <deref_ptr> (os);
          }
        };
    }
    /// @endcond

    template < typename Element, typename Key = std::string>
    class HPP_CORE_DLLAPI Container
    {
      public:
        typedef std::map <Key, Element> ElementMap_t;
        typedef typename ElementMap_t::iterator iterator;
        typedef typename ElementMap_t::const_iterator const_iterator;
        typedef Key Key_t;
        typedef Element Element_t;

        /// Add an element
        void add (const Key& name, const Element& element)
        {
          map_[name] = element;
        }

        /// Return the element named name
        bool has (const Key& name) const
        {
          return (map_.find (name) != map_.end ());
        }

        /// Return the element named name
        const Element& get (const Key& name) const
        {
          typename ElementMap_t::const_iterator it = map_.find (name);
          if (it == map_.end ()) throw std::runtime_error ("invalid key");
          return it->second;
        }

        /// Return a list of all elements
        /// \tparam ReturnType must have a push_back method.
        template <typename ReturnType>
        ReturnType getAllAs () const
        {
          ReturnType l;
          for (typename ElementMap_t::const_iterator it = map_.begin ();
              it != map_.end (); ++it)
            l.push_back (it->second);
          return l;
        }

        template <typename ReturnType>
        ReturnType getKeys () const
        {
          ReturnType l;
          for (typename ElementMap_t::const_iterator it = map_.begin ();
              it != map_.end (); ++it)
            l.push_back (it->first);
          return l;
        }

        /// Return the underlying map.
        const ElementMap_t& getAll () const
        {
          return map_;
        }

        /// Print object in a stream
        std::ostream& print (std::ostream& os) const
        {
          for (typename ElementMap_t::const_iterator it = map_.begin ();
              it != map_.end (); ++it) {
            os << it->first << " : " << it->second << std::endl;
          }
          return os;
        }

        /// Print object in a stream
        std::ostream& printPointer (std::ostream& os) const
        {
          for (typename ElementMap_t::const_iterator it = map_.begin ();
              it != map_.end (); ++it) {
            os << it->first << " : " << *(it->second) << std::endl;
          }
          return os;
        }

      protected:
        /// Constructor
        Container () : map_ ()
        {}

      private:
        ElementMap_t map_;
    }; // class Container

    template <typename Types, typename Key = std::string >
      class Containers : private internal::container_traits <Types, Key>::type
    {
      public:
        template <typename Element> struct traits {
          typedef internal::field<Key, Element> Container;
          typedef typename Container::Map_t Map_t;
        };

      private:
        template <typename Element> struct _F {
          typedef internal::field<Key, Element> T;
          typedef typename T::Map_t Map_t;
        };

        /// Return the underlying map.
        template <typename Element>
        typename _F <Element>::Map_t& _map ()
        {
          return this->_F <Element>::T::map;
        }

      public:
        /// Return the underlying map.
        template <typename Element>
        const typename traits<Element>::Map_t& map () const
        {
          return this->_F <Element>::T::map;
        }

        /// Add an element
        template <typename Element>
        void add (const Key& name, const Element& element)
        {
          _map <Element>()[name] = element;
        }

        /// Return the element named name
        template <typename Element>
        bool has (const Key& name) const
        {
          return (map<Element>().find (name) != map<Element>().end ());
        }

        /// Return the element named name
        template <typename Element>
        const Element& get (const Key& name) const
        {
          typename _F<Element>::Map_t::const_iterator it
            = map<Element>().find (name);
          if (it == map<Element>().end ())
            throw std::runtime_error ("invalid key");
          return it->second;
        }

        /// Return a list of all elements
        /// \tparam ReturnType must have a push_back method.
        template <typename Element, typename ReturnType>
        ReturnType getAllAs () const
        {
          ReturnType l;
          for (typename _F<Element>::Map_t::const_iterator
              it = map<Element>().begin ();
              it != map<Element>().end ();
              ++it)
            l.push_back (it->second);
          return l;
        }

        template <typename Element, typename ReturnType>
        ReturnType getKeys () const
        {
          ReturnType l;
          for (typename _F<Element>::Map_t::const_iterator
              it = map<Element>().begin ();
              it != map<Element>().end ();
              ++it)
            l.push_back (it->first);
          return l;
        }

        /// Print object in a stream
        std::ostream& print (std::ostream& os) const
        {
          boost::mpl::for_each < Types > (print <false> (os));
          return os;
        }

        /// Print object in a stream
        std::ostream& printPointer (std::ostream& os) const
        {
          boost::mpl::for_each < Types > (print <true> (os));
          return os;
        }
    };
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_CONTAINER_HH
