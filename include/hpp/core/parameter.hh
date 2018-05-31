//
// Copyright 2018 CNRS
//
// Author: Florent Lamiraux, Joseph Mirabel
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

#ifndef HPP_CORE_PARAMETER_HH
# define HPP_CORE_PARAMETER_HH

#include <iostream>
#include <string>
#include <cassert>
#include <typeinfo>

#include <hpp/core/config.hh>
#include <hpp/core/fwd.hh>

namespace hpp {
  namespace core {
    /*
    class Parameter;
    class DYNAMIC_GRAPH_DLLAPI EitherType {
    public:
      EitherType(const Parameter& value);
      ~EitherType();
      operator bool () const;
      operator unsigned () const;
      operator int () const;
      operator float () const;
      operator double () const;
      operator std::string () const;
      operator Vector () const;
      operator Eigen::MatrixXd () const;
      operator Eigen::Matrix4d () const;
    private:
      const Parameter* value_;
    };
    */

    class HPP_CORE_DLLAPI Parameter
    {
      public:
        enum Type {
          NONE,
          BOOL,
          /// as \ref size_type
          INT,
          /// as \ref value_type
          FLOAT,
          STRING,
          /// as \ref vector_t
          VECTOR,
          /// as \ref matrix_t
          MATRIX,
          NB_TYPES
        };
        ~Parameter();
        void deleteValue ();
        explicit Parameter(const bool& value);
        explicit Parameter(const size_type& value);
        explicit Parameter(const value_type& value);
        explicit Parameter(const std::string& value);
        explicit Parameter(const vector_t& value);
        explicit Parameter(const matrix_t& value);
        /// Copy constructor
        Parameter(const Parameter& value);
        /// Construct an empty parameter (None)
        explicit Parameter();
        // operator assignement
        Parameter operator=(const Parameter& value);
        /// Return the type of the value
        Type type() const;

        /// Return the value as a castable value into the approriate type
        ///
        /// For instance,
        /// \code
        /// Parameter v1(5.0); // v1 is of type double
        /// Parameter v2(3);   // v2 is of type int
        /// double x1 = v1.value();
        /// double x2 = v2.value();
        /// \endcode
        /// The first assignment will succeed, while the second one will throw
        /// an exception.
        // const EitherType value () const;
        /// Return the name of the type
        static std::string typeName(Type type);

        /// Output in a stream
        HPP_CORE_DLLAPI friend std::ostream& operator<<(std::ostream& os, const Parameter& value);
      public:
        // friend class EitherType;
        bool              boolValue() const;
        size_type         intValue() const;
        value_type        floatValue() const;
        std::string       stringValue() const;
        vector_t          vectorValue() const;
        matrix_t          matrixValue() const;
        Type type_;

        const void* const value_;
    };

    class HPP_CORE_DLLAPI ParameterDescription
    {
      public:
        ParameterDescription (Parameter::Type type,
            std::string name,
            std::string doc = "",
            Parameter defaultValue = Parameter())
          : name_ (name)
          , doc_ (doc)
          , type_ (type)
          , defaultValue_ (defaultValue)
        {}

        ParameterDescription () : type_ (Parameter::NONE) {}

        const std::string&     name () const { return name_; }
        const Parameter::Type& type () const { return type_; }
        const std::string&     doc  () const { return doc_ ; }
        const Parameter&       defaultValue () const;

      private:
        std::string name_;
        std::string doc_;
        Parameter::Type type_;
        Parameter defaultValue_;
    };
  } // namespace core
} //namespace hpp

#endif // HPP_CORE_PARAMETER_HH
