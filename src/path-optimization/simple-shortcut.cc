//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux, Joseph Mirabel
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

#include <hpp/core/path-optimization/simple-shortcut.hh>

#include <hpp/util/assertion.hh>
#include <hpp/util/debug.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/problem-target.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/roadmap.hh>

namespace hpp {
  namespace core {
    namespace pathOptimization {
      SimpleShortcutPtr_t
      SimpleShortcut::create (const Problem& problem)
      {
        SimpleShortcut* ptr = new SimpleShortcut (problem);
        return SimpleShortcutPtr_t (ptr);
      }

      SimpleShortcut::SimpleShortcut (const Problem& problem) :
        PathOptimizer (problem)
      {
      }

      PathVectorPtr_t SimpleShortcut::optimize (const PathVectorPtr_t& path)
      {
        RoadmapPtr_t roadmap (Roadmap::create (problem ().distance (),
                                               problem ().robot ()));
        std::vector <NodePtr_t> nodes;
        ConfigurationPtr_t qPtr (new Configuration_t (path->initial ()));
        roadmap->initNode (qPtr);
        NodePtr_t node (roadmap->initNode ());
        nodes.push_back (node);
        for (std::size_t i=0; i<path->numberPaths (); ++i) {
          PathPtr_t p (path->pathAtRank (i));
          qPtr = ConfigurationPtr_t (new Configuration_t (p->end ()));
          node = roadmap->addNodeAndEdge (node, qPtr, p);
          nodes.push_back (node);
        }
        roadmap->addGoalNode (node->configuration ());
        PathValidationPtr_t pv (problem ().pathValidation ());
        for (std::size_t i=0; i < nodes.size () - 1; ++i) {
          for (std::size_t j=i+2; j < nodes.size (); ++j) {
            PathPtr_t path (steer (*(nodes [i]->configuration ()),
                                   *(nodes [j]->configuration ())));
            PathValidationReportPtr_t report;
            PathPtr_t unused;
            if ((path) &&
                (pv->validate (path, false, unused, report))) {
              roadmap->addEdge (nodes [i], nodes [j], path);
            }
          }
        }
        PathVectorPtr_t result (problem ().target ()->computePath (roadmap));
        assert (result);
        return result;
      }
    } // namespace pathOptimization
  } // namespace core
} // namespace hpp
