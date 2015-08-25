// Copyright (c) 2015 CNRS
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

#ifndef HPP_CORE_PARSER_ROADMAP_FACTORY_HH
# define HPP_CORE_PARSER_ROADMAP_FACTORY_HH

# include <hpp/util/parser.hh>

# include <hpp/util/factories/sequence.hh>

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>

namespace hpp {
  namespace core {
    namespace parser {
      /// \addtogroup parser
      /// \{

      using hpp::util::parser::ObjectFactory;
      using hpp::util::parser::XMLElement;
      typedef hpp::util::parser::SequenceFactory<std::string> StringSequence;
      typedef hpp::util::parser::SequenceFactory<double> ConfigurationFactory;

      void writeRoadmap (std::ostream& o, const RoadmapPtr_t roadmap,
          const DevicePtr_t robot);

      RoadmapPtr_t readRoadmap (const std::string& filename, const DistancePtr_t distance,
          const DevicePtr_t robot);

      class HPP_CORE_DLLAPI RoadmapFactory : public ObjectFactory {
        public:
          typedef ::hpp::util::parser::ObjectFactory Parent_t;

          RoadmapFactory (const DistancePtr_t& distance,
              const DevicePtr_t& robot, ObjectFactory* parent,
              const XMLElement* element);

          RoadmapPtr_t roadmap () const {
            return roadmap_;
          }

          struct ArgumentParser {
            static DistancePtr_t d_;
            static DevicePtr_t r_;
            // ArgumentParser (const DistancePtr_t& d,
              // const DevicePtr_t& r) : d_ (d), r_ (r) {}
            static ObjectFactory* create (ObjectFactory* parent = NULL,
                const XMLElement* element = NULL) {
              return new RoadmapFactory (d_, r_, parent, element);
            }
          };

          virtual void finishTags ();

          RoadmapFactory (const DevicePtr_t& robot, RoadmapPtr_t roadmap,
              ObjectFactory* parent = NULL);

        private:
          DistancePtr_t distance_;
          DevicePtr_t robot_;

          void computePermutation (const std::vector <std::string>& jointNames);
          ConfigurationPtr_t permuteAndCreateConfiguration
            (const std::vector <double>& config);

          RoadmapPtr_t roadmap_;

          typedef std::vector <std::size_t> SizeVector_t;
          SizeVector_t permutation_;

          typedef std::vector <NodePtr_t> Nodes_t;
          typedef std::vector <EdgePtr_t> Edges_t;
          Nodes_t nodes_;
          Edges_t edges_;
      }; // class RoadmapVisitor
      /// \}
    } //   namespace parser
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_PARSER_ROADMAP_FACTORY_HH
