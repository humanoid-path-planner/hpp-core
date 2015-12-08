// Copyright (c) 2015, Joseph Mirabel
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

#include <hpp/util/debug.hh>
#include "hpp/core/parser/roadmap-factory.hh"

#include <string>

#include <hpp/model/configuration.hh>
#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>

#include "hpp/core/steering-method-straight.hh"
#include "hpp/core/roadmap.hh"
#include "hpp/core/node.hh"
#include "hpp/core/edge.hh"

namespace hpp {
  namespace core {
    namespace parser {
      DistancePtr_t RoadmapFactory::ArgumentParser::d_ = DistancePtr_t ();
      DevicePtr_t   RoadmapFactory::ArgumentParser::r_ = DevicePtr_t ();

      void writeRoadmap (std::ostream& o, const RoadmapPtr_t roadmap,
          const DevicePtr_t robot)
      {
        using namespace hpp::util::parser;
        XMLDocument doc;
        XMLPrinter printer;
        doc.InsertEndChild (XMLDeclaration ("0.0", "Unknown", "yes"));
        doc.InsertEndChild (XMLComment ("This file was automatically generated."));
        RoadmapFactory rf (robot, roadmap);
        // XMLNode* node = rf.write (&doc);
        rf.write (&doc);
        doc.Accept (&printer);

        o << printer.Str();
      }

      RoadmapPtr_t readRoadmap (const std::string& fn, const DistancePtr_t distance,
          const DevicePtr_t robot)
      {
        using namespace hpp::util::parser;
        Parser p;
        // RoadmapFactory::ArgumentParser ap (distance, robot);
        // p.addObjectFactory ("roadmap", (Parser::FactoryType)(&(ap.create)));
        RoadmapFactory::ArgumentParser::d_ = distance;
        RoadmapFactory::ArgumentParser::r_ = robot;
        p.addObjectFactory ("roadmap", RoadmapFactory::ArgumentParser::create);
        p.addObjectFactory ("joints", create <StringSequence>);
        p.addObjectFactory ("node", create <ConfigurationFactory>);
        p.addObjectFactory ("goal_nodes", create <IdSequence>);
        p.addObjectFactory ("path", create <ObjectFactory>);
        p.parseFile (fn);
        ObjectFactory* rf = 0;
        p.root ()->getChildOfType ("roadmap", rf);
        return rf->as<RoadmapFactory>()->roadmap ();
      }

      RoadmapFactory::RoadmapFactory (const DistancePtr_t& distance,
          const DevicePtr_t& robot, ObjectFactory* parent,
          const XMLElement* element)
        : ObjectFactory (parent, element), distance_ (distance), robot_ (robot),
        extraCSsize_ (0)
      {
      }

      bool RoadmapFactory::finishAttributes ()
      {
        if (hasAttribute ("extra_config_space")) {
          extraCSsize_ = boost::lexical_cast <size_type>
            (getAttribute ("extra_config_space"));
        }
        if (extraCSsize_ != robot_->extraConfigSpace().dimension()) {
          hppDout (error, "Robot extra config space do not match attribute "
              "\"extra_config_space\"");
          return false;
        }
        return ObjectFactory::finishAttributes ();
      }

      void RoadmapFactory::finishTags ()
      {
        ObjectFactory* o;
        /// First compute the permutation
        getChildOfType ("joints", o);
        StringSequence* jointNames = o->as <StringSequence>();
        computePermutation (jointNames->values());

        roadmap_ = Roadmap::create (distance_, robot_);

        /// Get all the configurations and build the list of nodes
        ObjectFactory::ObjectFactoryList nodeList = getChildrenOfType ("node");
        for (ObjectFactory::ObjectFactoryList::const_iterator
            it = nodeList.begin(); it != nodeList.end(); ++it) {
          ConfigurationFactory* cf = (*it)->as <ConfigurationFactory> ();
          nodes_.push_back (
              roadmap_->addNode (
                permuteAndCreateConfiguration (cf->values())
                )
              );
        }

        /// Get all the edges and build a path
        ObjectFactory::ObjectFactoryList pathList = getChildrenOfType ("path");
        SteeringMethodStraightPtr_t sm_ptr = SteeringMethodStraight::create (robot_);
        SteeringMethodStraight& sm = *sm_ptr;
        for (ObjectFactory::ObjectFactoryList::const_iterator
            it = pathList.begin(); it != pathList.end(); ++it) {
          ObjectFactory* p = *it;
          int fromId = boost::lexical_cast <int> (p->getAttribute ("from")),
                toId = boost::lexical_cast <int> (p->getAttribute ("to"));
          NodePtr_t from = nodes_[fromId],
                    to   = nodes_[toId];
          PathPtr_t path = sm (*(from->configuration()),
                               *(to  ->configuration()));
          if (o->hasAttribute ("constraint")) {
            std::string constraintName = o->getAttribute ("constraint");
            hppDout (warning, "Constraints in paths is not supported yet");
          }
          edges_.push_back (
            roadmap_->addEdge (from, to, path)
            );
        }

        /// Set init node
        if (hasAttribute ("init_node")) {
          int initId = boost::lexical_cast <int> (getAttribute ("init_node"));
          NodePtr_t init = nodes_[initId];
          roadmap_->initNode (init->configuration ());
        }
        /// Set goal nodes
        try {
          getChildOfType ("goal_nodes", o);
          IdSequence* idSeq = o->as <IdSequence> ();
          for (IdSequence::OutType::const_iterator it = idSeq->values ().begin ();
              it != idSeq->values().end(); ++it) {
            roadmap_->addGoalNode (nodes_[*it]->configuration ());
          }
        } catch (const std::invalid_argument& e) {
          hppDout (warning, e.what());
        }
      }

      void RoadmapFactory::computePermutation (
          const std::vector <std::string>& jn)
      {
        size_t rank = 0;
        permutation_ = SizeVector_t (robot_->configSize ());
        for (size_t i = 0; i < jn.size (); ++i) {
          JointPtr_t j = robot_->getJointByName (jn[i]);
          if (!j) throw std::invalid_argument ("Joint " + jn[i] + " not found");
          for (size_type r = 0; r < j->configSize (); ++r)
            permutation_ [rank + r] = j->rankInConfiguration() + (std::size_t)r;
          rank += j->configSize();
        }
        if (rank + extraCSsize_ != permutation_.size())
          throw std::logic_error (
              "The list of joints does not correspond to this robot.");
        for (size_type i = 0; i < extraCSsize_; ++i) {
          permutation_ [rank + i] = rank + i;
        }
      }

      ConfigurationPtr_t RoadmapFactory::permuteAndCreateConfiguration (
          const std::vector <double>& config)
      {
        ConfigurationPtr_t cfg (new Configuration_t (robot_->configSize()));
        Configuration_t& q =*cfg;
        for (size_type i = 0; i < q.size(); ++i)
          q[i] = config [permutation_[i]];
	normalize (robot_, q);
        return cfg;
      }

      RoadmapFactory::RoadmapFactory (const DevicePtr_t& robot, RoadmapPtr_t roadmap,
          ObjectFactory* parent) :
        ObjectFactory ("roadmap", parent), robot_ (robot), roadmap_ (roadmap),
        extraCSsize_ (robot->extraConfigSpace().dimension())
      {
        if (extraCSsize_ > 0)
          addAttribute ("extra_config_space",
              boost::lexical_cast <std::string> (extraCSsize_));

        // Write joint names
        const JointVector_t& joints = robot_->getJointVector ();
        size_type rank = -1;
        StringSequence::OutType ssValues (joints.size());
        for (JointVector_t::const_iterator it = joints.begin();
            it != joints.end(); ++it) {
          if ((*it)->configSize() <= 0) continue;
          assert (rank < (*it)->rankInConfiguration());
          rank = (*it)->rankInConfiguration();
          ssValues.push_back ((*it)->name());
        }
        StringSequence* ss = new StringSequence ("joints", this);
        ss->values (ssValues);

        // Write nodes and edges
        typedef std::list <NodePtr_t> NodeList;
        typedef std::list <EdgePtr_t> EdgeList;
        size_type fromId = 0;
        const NodeList& nodes = roadmap_->nodes ();
        for (NodeList::const_iterator it = nodes.begin ();
            it != nodes.end(); ++it) {
          ConfigurationFactory* cf = new ConfigurationFactory ("node", this);
          const Configuration_t& q = *((*it)->configuration());
          ConfigurationFactory::OutType values (q.size());
          for (size_type i = 0; i < q.size(); ++i) values[i] = q[i];
          cf->values (values);

          const EdgeList& edges = (*it)->outEdges ();
          for (EdgeList::const_iterator ed = edges.begin();
              ed != edges.end (); ++ed) {
            ObjectFactory* of = new ObjectFactory ("path", this);
            // Find id of final configuration
            size_type toId = getNodeIdFromRoadmap ((*ed)->to());
            if (toId < 0) {
              hppDout (error, "Id not found");
            }
            of->addAttribute ("from", boost::lexical_cast <std::string> (fromId));
            of->addAttribute ("to", boost::lexical_cast <std::string> (toId));
            of->addAttribute ("constraint", (*ed)->path()->constraints()->name());
          }
          fromId++;
        }

        size_type initId = getNodeIdFromRoadmap (roadmap_->initNode ());
        if (initId >= 0) {
          addAttribute ("init_node", boost::lexical_cast <std::string> (initId));
        }
        IdSequence* goalNodes = new IdSequence ("goal_nodes", this);
        IdSequence::OutType values;
        for (core::Nodes_t::const_iterator it = roadmap_->goalNodes ().begin ();
            it != roadmap_->goalNodes ().end(); ++it) {
          size_type id = getNodeIdFromRoadmap (*it);
          if (id >= 0) {
            values.push_back ((unsigned int)id);
          }
        }
        goalNodes->values (values);
      }

      size_type RoadmapFactory::getNodeIdFromRoadmap (const NodePtr_t& node) const
      {
        size_type id = -1;
        for (core::Nodes_t::const_iterator it = roadmap_->nodes().begin ();
            it != roadmap_->nodes().end(); ++it) {
          id++;
          if (*it == node) break;
        }
        return id;
      }
    } // namespace parser
  } // namespace core
} // namespace hpp
