//
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

#include "extracted-path.hh"

#include <boost/serialization/weak_ptr.hpp>
#include <hpp/util/serialization.hh>

namespace hpp {
  namespace core {
    template<class Archive>
    void ExtractedPath::serialize(Archive & ar, const unsigned int version)
    {
      using namespace boost::serialization;
      (void) version;
      ar & make_nvp("base", base_object<Path>(*this));
      ar & BOOST_SERIALIZATION_NVP(original_);
      ar & BOOST_SERIALIZATION_NVP(reversed_);
      ar & BOOST_SERIALIZATION_NVP(weak_);
    }

    HPP_SERIALIZATION_IMPLEMENT(ExtractedPath);
  } //   namespace hpp-core
} // namespace hpp

BOOST_CLASS_EXPORT(hpp::core::ExtractedPath)
