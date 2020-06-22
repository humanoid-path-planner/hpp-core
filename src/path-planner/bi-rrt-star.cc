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

#include <hpp/core/path-planner/bi-rrt-star.hh>

#include <hpp/core/configuration-shooter.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/path-validation-report.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>

namespace hpp {
  namespace core {
    namespace pathPlanner {
      BiRrtStarPtr_t BiRrtStar::create (const Problem& problem)
      {
        BiRrtStarPtr_t shPtr (new BiRrtStar (problem));
        shPtr->init (shPtr);
        return shPtr;
      }

      BiRrtStarPtr_t BiRrtStar::createWithRoadmap (const Problem& problem,
                                                 const RoadmapPtr_t& roadmap)
      {
        BiRrtStarPtr_t shPtr (new BiRrtStar (problem, roadmap));
        shPtr->init (shPtr);
        return shPtr;
      }

      BiRrtStar::BiRrtStar (const Problem& problem) :
        Parent_t (problem),
        gamma_ (1.),
        extendMaxLength_ (1.)
      {}

      BiRrtStar::BiRrtStar (const Problem& problem, const RoadmapPtr_t& roadmap) :
        Parent_t (problem, roadmap),
        gamma_ (1.),
        extendMaxLength_ (1.)
      {}

      void BiRrtStar::init (const BiRrtStarWkPtr_t& weak)
      {
        Parent_t::init (weak);
        weak_ = weak;
      }

      // ----------- Algorithm ---------------------------------------------- //

      void BiRrtStar::startSolve ()
      {
        Parent_t::startSolve ();

        if (roadmap()->goalNodes().size() != 1)
          throw std::invalid_argument("there should be only one goal node.");

        extendMaxLength_ = problem().getParameter("BiRRT*/maxStepLength").floatValue();
        gamma_ = problem().getParameter("BiRRT*/gamma").floatValue();

        a_ = roadmap()->initNode();
        b_ = roadmap()->goalNodes()[0];

        cost(a_, 0);
        cost(b_, 0);
      }

      void BiRrtStar::oneStep ()
      {
        Configuration_t q = sample();

        if (extend(a_->connectedComponent(), q)) {
          // in the unlikely event that extend connected the two graphs,
          // then one of the connected component is not valid.
          if (a_->connectedComponent() == b_->connectedComponent()) return;
          connect(b_, q);
        }

        std::swap(a_, b_);

        /*
        switch(roadmap()->connectedComponents().size()) {
          case 1:
            // Assume the problem is solved and we only want to refine the roadmap
            break;
          case 2:
            break;
          default:
            throw std::logic_error("BiRrtStar works only with 1 or 2 connected components.");
        }

        */
      }

      Configuration_t BiRrtStar::sample ()
      {
        ConfigurationShooterPtr_t shooter = problem().configurationShooter();
        Configuration_t q;
        shooter->shoot(q);
        return q;
      }

      value_type BiRrtStar::cost(NodePtr_t n)
      {
        Costs_t::const_iterator _c = costs_.find(n);
        if (_c == costs_.end())
          throw std::logic_error("this node has no cost.");
        return _c->second;
      }

      void BiRrtStar::cost(NodePtr_t n, value_type c)
      {
        costs_[n] = c;
      }

      bool validate(const Problem& problem, const PathPtr_t& path)
      {
        PathPtr_t validPart;
        PathValidationReportPtr_t report;
        return problem.pathValidation()
          ->validate (path, false, validPart, report);
      }

      PathPtr_t BiRrtStar::buildPath(const Configuration_t& q0, const Configuration_t& q1,
          value_type maxLength,
          bool validatePath)
      {
        PathPtr_t path = problem().steeringMethod()->steer(q0, q1);
        if (!path) return path;
        if (problem().pathProjector()) { // path projection
          PathPtr_t projected;
          problem().pathProjector()->apply (path, projected);
          if (!projected) return projected;
          path = projected;
        }

        if (maxLength > 0 && path->length() > maxLength) {
          const interval_t& I = path->timeRange();
          path = path->extract(I.first, I.first + maxLength);
        }

        if (!validatePath) return path;

        PathPtr_t validPart;
        PathValidationReportPtr_t report;
        problem().pathValidation()->validate (path, false, validPart, report);
        return validPart;
      }

      bool BiRrtStar::extend (ConnectedComponentPtr_t cc, Configuration_t& q)
      {
        value_type dist;
        NodePtr_t near = roadmap()->nearestNode(q, cc, dist);
        if (dist < 1e-16)
          return false;

        PathPtr_t path = buildPath(*near->configuration(), q, extendMaxLength_, true);
        if (!path || path->length() < 1e-10) return false;
        q = path->end();

        value_type n ((value_type)roadmap()->nodes().size());
        NodeVector_t nearNodes = roadmap()->nodesWithinBall(q, cc,
            gamma_ * std::pow(std::log(n)/n, 1./(value_type)problem().robot()->numberDof()));

        value_type cost_q (cost(near) + path->length());
        typedef std::pair<bool, PathPtr_t> ValidatedPath_t;
        std::vector<ValidatedPath_t> paths;
        paths.reserve(nearNodes.size());
        for (NodeVector_t::const_iterator _near = nearNodes.begin(); _near != nearNodes.end(); ++_near) {
          PathPtr_t near2new = buildPath(*(*_near)->configuration(), q, -1, false);
          paths.push_back(ValidatedPath_t(false, near2new));
          if (!near2new) {
            continue;
          }

          value_type _cost_q = cost(*_near) + near2new->length();
          if (_cost_q < cost_q) {
            paths.back().first = true;
            // Run path validation
            if (validate (problem(), path)) {
              // Path is valid and shorter.
              cost_q = _cost_q;
              near = *_near;
              path = near2new;
            } else
              paths.back().second.reset();
          }
        }

        NodePtr_t qnew = roadmap()->addNode(boost::make_shared<Configuration_t>(q));
        roadmap()->addEdges(near, qnew, path);
        cost(qnew, cost_q);

        for (std::size_t i = 0; i < nearNodes.size(); ++i) {
          if (nearNodes[i] == near || !paths[i].second) continue;

          value_type cost_q_near = cost_q + paths[i].second->length();
          if (cost_q_near < cost(nearNodes[i])) {
            bool pathValid = paths[i].first;
            if (!pathValid) // If path validation has not been run
              pathValid = validate(problem(), paths[i].second);
            if (pathValid) {
              // TODO remove edge between nearNodes[i] and qnew ?
              // does not seem necessary.
              roadmap()->addEdges(nearNodes[i], qnew, paths[i].second);
              cost(nearNodes[i], cost_q_near);
            }
          }
        }
        return true;
      }

      bool BiRrtStar::connect (NodePtr_t b, const Configuration_t& q)
      {
        Configuration_t qnew;
        // while extend did not reach q
        while (roadmap()->connectedComponents().size() == 2) {
          qnew = q;
          if (!extend(b->connectedComponent(), qnew)) // extend failed
            return false;
        }
        return true;
      }

      // ----------- Declare parameters ------------------------------------- //

      HPP_START_PARAMETER_DECLARATION(BiRrtStar)
      Problem::declareParameter(ParameterDescription (Parameter::FLOAT,
            "BiRRT*/maxStepLength",
            "The maximum step length when extending.",
            Parameter(1.)));
      Problem::declareParameter(ParameterDescription (Parameter::FLOAT,
            "BiRRT*/gamma",
            "",
            Parameter(1.)));
      HPP_END_PARAMETER_DECLARATION(BiRrtStar)
    } // namespace pathPlanner
  } // namespace core
} // namespace hpp

