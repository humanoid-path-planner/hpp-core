// Copyright (c) 2018, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr), Florent Lamiraux
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


#include <hpp/core/parameter.hh>

#include <hpp/util/exception-factory.hh>

namespace hpp {
  namespace core {

    static void* copyValue(const Parameter& value);

    /*
    EitherType::EitherType(const Parameter& value) : value_(new Parameter(value))
    {
    }

    EitherType::~EitherType()
    {
      delete value_;
      value_ = NULL;
    }

    EitherType::operator bool() const
    {
      return value_->boolValue();
    }
    EitherType::operator unsigned() const
    {
      return value_->unsignedValue();
    }
    EitherType::operator int() const
    {
      return value_->intValue();
    }
    EitherType::operator float() const
    {
      return value_->floatValue();
    }
    EitherType::operator double() const
    {
      return value_->doubleValue();
    }
    EitherType::operator std::string() const
    {
      return value_->stringValue();
    }
    EitherType::operator Vector() const
    {
      return value_->vectorValue();
    }
    EitherType::operator Eigen::MatrixXd() const
    {
      return value_->matrixXdValue();
    }

    EitherType::operator Eigen::Matrix4d() const
    {
      return value_->matrix4dValue();
    }
    */

    void Parameter::deleteValue ()
    {
      switch(type_) {
      case BOOL:
	delete(const bool*)value_;
	break;
      case INT:
	delete(const size_type*)value_;
	break;
      case FLOAT:
	delete(const value_type*)value_;
	break;
      case STRING:
	delete(const std::string*)value_;
	break;
      case VECTOR:
	delete(const vector_t*)value_;
	break;
      case MATRIX:
	delete(const matrix_t*)value_;
	break;
      default:;
      }
    }

    Parameter::~Parameter()
    {
      deleteValue ();
    }

    Parameter::Parameter(const bool& value)
      : type_(BOOL), value_(new bool(value))
    {}

    Parameter::Parameter(const size_type& value)
      : type_(INT), value_(new size_type(value))
    {}

    Parameter::Parameter(const value_type& value)
      : type_(FLOAT), value_(new value_type(value))
    {}
    Parameter::Parameter(const std::string& value)
      : type_(STRING), value_(new std::string(value))
    {}

    Parameter::Parameter(const vector_t& value)
      : type_(VECTOR), value_(new vector_t(value))
    {}

    Parameter::Parameter(const matrix_t& value)
      : type_(MATRIX), value_(new matrix_t(value))
    {}

    Parameter::Parameter(const Parameter& value)
      : type_(value.type_), value_(copyValue(value))
    {}

    void* copyValue(const Parameter& value)
    {
      void* copy;
      switch(value.type()) {
      case Parameter::NONE:
	copy = NULL;
        break;
      case Parameter::BOOL:
	copy = new bool(value.boolValue());
	break;
      case Parameter::INT:
	copy = new size_type(value.intValue());
	break;
      case Parameter::FLOAT:
	copy = new value_type(value.floatValue());
	break;
      case Parameter::STRING:
	copy = new std::string(value.stringValue());
	break;
      case Parameter::VECTOR:
	copy = new vector_t(value.vectorValue());
	break;
      case Parameter::MATRIX:
	copy = new matrix_t(value.matrixValue());
	break;
      default:
        throw std::invalid_argument ("value type is unknown.");
        break;
      }
      return copy;
    }

    Parameter::Parameter() : type_(NONE), value_(NULL)
    {}

    Parameter Parameter::operator=(const Parameter& value)
    {
      if (&value != this) {
	if(value_ != 0x0)
	  deleteValue ();
	type_ = value.type_;
	void** ptValue = const_cast<void**>(&value_);
	*ptValue = copyValue(value);
      }
      return *this;
    }

    // const EitherType Parameter::value() const
    // {
      // return EitherType(*this);
    // }

    Parameter::Type Parameter::type() const
    {
      return type_;
    }

    inline void check (Parameter::Type type, Parameter::Type expected)
    {
      if (type != expected)
        throw std::invalid_argument ("value is not a " + Parameter::typeName (expected));
    }

    bool Parameter::boolValue() const
    {
      check (type_, BOOL);
      return *((const bool*)value_);
    }

    size_type Parameter::intValue() const
    {
      check (type_, INT);
      return *((const size_type*)value_);
    }

    value_type Parameter::floatValue() const
    {
      check (type_, FLOAT);
      return *((const value_type*)value_);
    }

    std::string Parameter::stringValue() const
    {
      check (type_, STRING);
      return *((const std::string*)value_);
    }

    vector_t Parameter::vectorValue() const
    {
      check (type_, VECTOR);
      return *((const vector_t*)value_);
    }

    matrix_t Parameter::matrixValue() const
    {
      check (type_, MATRIX);
      return *((const matrix_t*)value_);
    }

    std::string Parameter::typeName(Type type)
    {
      switch(type) {
      case BOOL:
	return std::string("bool");
      case INT:
	return std::string("size_type");
      case FLOAT:
	return std::string("value_type");
      case STRING:
	return std::string("string");
      case VECTOR:
	return std::string("vector");
      case MATRIX:
	return std::string("matrix");
      default:
	return std::string("unknown");
      }
    }

    const Parameter& ParameterDescription::defaultValue () const
    {
      if (defaultValue_.type() == type_)
        return defaultValue_;
      HPP_THROW(std::logic_error,
          "Parameter " << name_ << " expected a default value of type "
          << Parameter::typeName(type_)
          << " but has invalid default value " << defaultValue_);
    }

    std::ostream& operator<<(std::ostream& os, const Parameter& value)
    {
      os << "Type=" << Parameter::typeName(value.type_)
	 << ", value=";
      switch(value.type_) {
      case Parameter::BOOL:
	os << value.boolValue();
	break;
      case Parameter::INT:
	os << value.intValue();
	break;
      case Parameter::FLOAT:
	os << value.floatValue();
	break;
      case Parameter::STRING:
	os << value.stringValue();
	break;
      case Parameter::VECTOR:
	os << value.vectorValue();
	break;
      case Parameter::MATRIX:
	os << value.matrixValue();
	break;
      default:
	return os;
      }
      return os;
    }

    // template< typename T >
      // struct DYNAMIC_GRAPH_DLLAPI ValueHelper
      // {
	// static const Parameter::Type TypeID;
      // };
    // template<> const Parameter::Type ValueHelper<bool>::TypeID = Parameter::BOOL;
    // template<> const Parameter::Type ValueHelper<unsigned>::TypeID = Parameter::UNSIGNED;
    // template<> const Parameter::Type ValueHelper<int>::TypeID = Parameter::INT;
    // template<> const Parameter::Type ValueHelper<float>::TypeID = Parameter::FLOAT;
    // template<> const Parameter::Type ValueHelper<double>::TypeID = Parameter::DOUBLE;
    // template<> const Parameter::Type ValueHelper<std::string>::TypeID = Parameter::STRING;
    // template<> const Parameter::Type ValueHelper<Vector>::TypeID = Parameter::VECTOR;
    // template<> const Parameter::Type ValueHelper<Eigen::MatrixXd>::TypeID = Parameter::MATRIX;
    // template<> const Parameter::Type ValueHelper<Eigen::Matrix4d>::TypeID = Parameter::MATRIX4D;

  } // namespace core
} // namespace hpp
