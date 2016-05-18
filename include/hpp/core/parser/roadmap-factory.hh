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
      typedef hpp::util::parser::SequenceFactory<unsigned int> IdSequence;

      /// Write roadmap to a text file.
      void writeRoadmap (std::ostream& o, const ProblemPtr_t& problem,
          const RoadmapPtr_t& roadmap);

      /// Create a new roadmap from a file.
      RoadmapPtr_t readRoadmap (const std::string& filename,
          const ProblemPtr_t& problem);

      /// Populate an existing roadmap from a file.
      RoadmapPtr_t readRoadmap (const std::string& filename,
          const RoadmapPtr_t& roadmap, const ProblemPtr_t& problem);

      class HPP_CORE_DLLAPI RoadmapFactory : public ObjectFactory {
        public:
          typedef ::hpp::util::parser::ObjectFactory Parent_t;

          /// Constructor using an already created Roadmap
          RoadmapFactory (
              const RoadmapPtr_t& roadmap, const ProblemPtr_t& problem,
              ObjectFactory* parent, const XMLElement* element);

          RoadmapPtr_t roadmap () const {
            return roadmap_;
          }

          static ObjectFactory* create (
              const RoadmapPtr_t& r, const ProblemPtr_t& p,
              ObjectFactory* parent, const XMLElement* el)
          {
            return new RoadmapFactory (r, p, parent, el);
          }

          virtual bool finishAttributes ();

          virtual void finishTags ();

          RoadmapFactory (const ProblemPtr_t& problem,
              const RoadmapPtr_t& roadmap, ObjectFactory* parent = NULL);

        private:
          ProblemPtr_t problem_;

          void computePermutation (const std::vector <std::string>& jointNames);
          ConfigurationPtr_t permuteAndCreateConfiguration
            (const std::vector <double>& config);

          size_type getNodeIdFromRoadmap (const NodePtr_t& node) const;

          RoadmapPtr_t roadmap_;

          typedef std::vector <std::size_t> SizeVector_t;
          SizeVector_t permutation_;

          size_type extraCSsize_;

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
