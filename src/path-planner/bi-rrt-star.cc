//
// Copyright (c) 2020 CNRS
// Authors: Joseph Mirabel
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#include <hpp/core/config-validations.hh>
#include <hpp/core/configuration-shooter.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/path-planner/bi-rrt-star.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/path-validation-report.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/pinocchio/configuration.hh>
#include <queue>

namespace hpp {
namespace core {
namespace pathPlanner {
BiRrtStarPtr_t BiRrtStar::create(const ProblemConstPtr_t& problem) {
  BiRrtStarPtr_t shPtr(new BiRrtStar(problem));
  shPtr->init(shPtr);
  return shPtr;
}

BiRrtStarPtr_t BiRrtStar::createWithRoadmap(const ProblemConstPtr_t& problem,
                                            const RoadmapPtr_t& roadmap) {
  BiRrtStarPtr_t shPtr(new BiRrtStar(problem, roadmap));
  shPtr->init(shPtr);
  return shPtr;
}

BiRrtStar::BiRrtStar(const ProblemConstPtr_t& problem)
    : Parent_t(problem),
      gamma_(1.),
      extendMaxLength_(1.),
      minimalPathLength_(1e-5),
      toRoot_(2) {
  maxIterations(100);
  stopWhenProblemIsSolved(false);
}

BiRrtStar::BiRrtStar(const ProblemConstPtr_t& problem,
                     const RoadmapPtr_t& roadmap)
    : Parent_t(problem, roadmap),
      gamma_(1.),
      extendMaxLength_(1.),
      minimalPathLength_(1e-5),
      toRoot_(2) {
  maxIterations(100);
  stopWhenProblemIsSolved(false);
}

void BiRrtStar::init(const BiRrtStarWkPtr_t& weak) {
  Parent_t::init(weak);
  weak_ = weak;
}

// ----------- Algorithm ---------------------------------------------- //

typedef std::pair<bool, PathPtr_t> ValidatedPath_t;

typedef std::map<NodePtr_t, EdgePtr_t> ParentMap_t;
value_type computeCost(const ParentMap_t& map, NodePtr_t n) {
  typedef ParentMap_t::const_iterator It_t;
  value_type c = 0;
  for (It_t current = map.find(n); current->second;
       current = map.find(current->second->from())) {
    if (current == map.end())
      throw std::logic_error(
          "BiRRT*: This node has no parent. You cannot use BiRRT* from a "
          "precomputed roadmap.");
    c += current->second->path()->length();
  }
  return c;
}

/// \param map the parent map to update
/// \param n a roadmap node
/// \param e a roadmap edge that ends at \c n.
/// \note if \c e is NULL, then \c n is considered a root of the parent
/// map.
void setParent(ParentMap_t& map, NodePtr_t n, EdgePtr_t e, bool newNode) {
  if (e) {
    assert(e->to() == n);
    if (map.find(e->from()) == map.end())
      throw std::logic_error(
          "BiRRT*: Could not find node from of edge in parent map. You cannot "
          "use BiRRT* from a precomputed roadmap.");
  }
  if (newNode && map.count(n))
    throw std::logic_error("BiRRT*: This node already exists in the roadmap.");
  map[n] = e;
}

struct WeighedNode_t {
  NodePtr_t node;
  EdgePtr_t parent;
  value_type cost;
  bool operator<(const WeighedNode_t& other) const { return cost < other.cost; }
  WeighedNode_t(NodePtr_t node, EdgePtr_t parent, value_type cost)
      : node(node), parent(parent), cost(cost) {}
};
typedef std::priority_queue<WeighedNode_t> Queue_t;

ParentMap_t computeParentMap(NodePtr_t root) {
  typedef std::map<NodePtr_t, WeighedNode_t> Visited_t;
  typedef Visited_t::iterator ItV_t;
  Visited_t visited;

  Queue_t queue;
  queue.push(WeighedNode_t(root, EdgePtr_t(), 0));

  while (!queue.empty()) {
    WeighedNode_t current(queue.top());
    queue.pop();

    std::pair<ItV_t, bool> res(
        visited.insert(std::make_pair(current.node, current)));
    bool addChildren(res.second);
    if (!addChildren) {
      // Not inserted because already visited. Check if path is better.
      // Normally, it is not possible that the children of current are
      // visited before all the best way of reaching current is found.
      if (res.first->second.cost > current.cost) {
        res.first->second.cost = current.cost;
        res.first->second.parent = current.parent;
        // Re-add to priority queue.
        addChildren = true;
      }
    }
    if (addChildren) {
      const Edges_t& edges = current.node->outEdges();
      for (Edges_t::const_iterator _edge = edges.begin(); _edge != edges.end();
           ++_edge) {
        EdgePtr_t edge(*_edge);
        queue.push(WeighedNode_t(edge->to(), edge,
                                 current.cost + edge->path()->length()));
      }
    }
  }

  ParentMap_t result;
  for (ItV_t _v = visited.begin(); _v != visited.end(); ++_v)
    result[_v->first] = _v->second.parent;
  return result;
}

void BiRrtStar::startSolve() {
  Parent_t::startSolve();

  if (roadmap()->goalNodes().size() != 1)
    throw std::invalid_argument("there should be only one goal node.");

  extendMaxLength_ =
      problem()->getParameter("BiRRT*/maxStepLength").floatValue();
  if (extendMaxLength_ <= 0)
    extendMaxLength_ = std::sqrt(problem()->robot()->numberDof());
  gamma_ = problem()->getParameter("BiRRT*/gamma").floatValue();
  minimalPathLength_ =
      problem()->getParameter("BiRRT*/minimalPathLength").floatValue();

  roots_[0] = roadmap()->initNode();
  roots_[1] = roadmap()->goalNodes()[0];

  toRoot_[0].clear();
  toRoot_[1].clear();
  setParent(toRoot_[0], roots_[0], EdgePtr_t(), true);
  setParent(toRoot_[1], roots_[1], EdgePtr_t(), true);
}

void BiRrtStar::oneStep() {
  Configuration_t q = sample();

  if (roadmap()->connectedComponents().size() == 2) {
    if (extend(roots_[0], toRoot_[0], q)) {
      // in the unlikely event that extend connected the two graphs,
      // then one of the connected component is not valid.
      if (roots_[0]->connectedComponent() == roots_[1]->connectedComponent())
        return;
      connect(roots_[1], toRoot_[1], q);
    }

    std::swap(roots_[0], roots_[1]);
    std::swap(toRoot_[0], toRoot_[1]);
  } else {
    if (toRoot_[1].find(roots_[0]) == toRoot_[1].end()) {
      // Fill parent map
      toRoot_[0] = computeParentMap(roots_[0]);
      toRoot_[1] = computeParentMap(roots_[1]);
    }

    assert(toRoot_[0].size() == toRoot_[1].size());
    assert(toRoot_[0].size() == roadmap()->nodes().size());
    improve(q);
  }
}

Configuration_t BiRrtStar::sample() {
  Configuration_t q(problem()->robot()->configSize());
  ConfigurationShooterPtr_t shooter = problem()->configurationShooter();

  if (roadmap()->connectedComponents().size() == 1 &&
      value_type(rand()) / INT_MAX > (value_type)0.2) {
    // Compute best path and find one point
    typedef ParentMap_t::const_iterator It_t;
    int nedges = 0;
    for (It_t current = toRoot_[0].find(roots_[1]); current->second;
         current = toRoot_[0].find(current->second->from())) {
      if (current == toRoot_[0].end()) {
        shooter->shoot(q);
        return q;
      }
      ++nedges;
    }
    if (nedges >= 2) {
      int i = 1 + (rand() % (nedges - 1));
      It_t edge1, edge0;
      for (edge1 = toRoot_[0].find(roots_[1]); i != 1;
           edge1 = toRoot_[0].find(edge1->second->from()))
        --i;
      edge0 = toRoot_[0].find(edge1->second->from());
      if (edge0->second->to() != edge1->second->from())
        throw std::logic_error("BiRRT*: wrong parent map.");

      // qm = (q0 + q2) / 2
      pinocchio::interpolate(problem()->robot(),
                             *edge0->second->from()->configuration(),
                             *edge1->second->to()->configuration(), 0.5, q);

      // q = q1 + alpha * (qm - q1)
      vector_t v(problem()->robot()->numberDof());
      pinocchio::difference(problem()->robot(), q,
                            *edge0->second->to()->configuration(), v);
      v.normalize();

      value_type l(extendMaxLength_ - value_type(rand()) * extendMaxLength_ /
                                          (10 * (value_type)INT_MAX));
      pinocchio::integrate(problem()->robot(),
                           *edge0->second->to()->configuration(), l * v, q);
      return q;
    }
  }

  shooter->shoot(q);
  return q;
}

bool validate(const ProblemConstPtr_t& problem, const PathPtr_t& path) {
  PathPtr_t validPart;
  PathValidationReportPtr_t report;
  return problem->pathValidation()->validate(path, false, validPart, report);
}

PathPtr_t BiRrtStar::buildPath(const Configuration_t& q0,
                               const Configuration_t& q1, value_type maxLength,
                               bool validatePath) {
  PathPtr_t path = problem()->steeringMethod()->steer(q0, q1);
  if (!path) return path;
  if (problem()->pathProjector()) {  // path projection
    PathPtr_t projected;
    problem()->pathProjector()->apply(path, projected);
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
  problem()->pathValidation()->validate(path, false, validPart, report);
  return validPart;
}

bool BiRrtStar::extend(NodePtr_t target, ParentMap_t& parentMap,
                       Configuration_t& q) {
  ConnectedComponentPtr_t cc(target->connectedComponent());

  value_type dist;
  NodePtr_t near = roadmap()->nearestNode(q, cc, dist);
  if (dist < 1e-16) return false;

  if (problem()->constraints() && !problem()->constraints()->apply(q))
    return false;

  PathPtr_t path = buildPath(*near->configuration(), q, extendMaxLength_, true);
  if (!path || path->length() < minimalPathLength_) return false;
  q = path->end();

  value_type n((value_type)roadmap()->nodes().size());
  NodeVector_t nearNodes = roadmap()->nodesWithinBall(
      q, cc,
      std::min(
          gamma_ * std::pow(std::log(n) / n,
                            1. / (value_type)problem()->robot()->numberDof()),
          extendMaxLength_));

  value_type cost_q(computeCost(parentMap, near) + path->length());
  std::vector<ValidatedPath_t> paths;
  paths.reserve(nearNodes.size());
  for (NodeVector_t::const_iterator _near = nearNodes.begin();
       _near != nearNodes.end(); ++_near) {
    PathPtr_t near2new;
    if (*_near == near) {
      near2new = path;
      paths.push_back(ValidatedPath_t(true, near2new));
      continue;
    } else {
      near2new = buildPath(*(*_near)->configuration(), q, -1, false);
      paths.push_back(ValidatedPath_t(false, near2new));
    }
    if (!near2new) continue;

    value_type _cost_q = computeCost(parentMap, *_near) + near2new->length();
    if (_cost_q < cost_q) {
      paths.back().first = true;
      // Run path validation
      if (validate(problem(), near2new)) {
        // Path is valid and shorter.
        cost_q = _cost_q;
        near = *_near;
        path = near2new;
      } else
        paths.back().second.reset();
    }
  }

  NodePtr_t qnew = roadmap()->addNode(make_shared<Configuration_t>(q));
  EdgePtr_t edge = roadmap()->addEdge(near, qnew, path);
  roadmap()->addEdge(qnew, near, path->reverse());
  assert(parentMap.find(near) != parentMap.end());
  if (parentMap.count(qnew)) return false;
  setParent(parentMap, qnew, edge, true);

  for (std::size_t i = 0; i < nearNodes.size(); ++i) {
    if (nearNodes[i] == near || !paths[i].second) continue;

    value_type cost_q_near = cost_q + paths[i].second->length();
    if (cost_q_near < computeCost(parentMap, nearNodes[i])) {
      bool pathValid = paths[i].first;
      if (!pathValid)  // If path validation has not been run
        pathValid = validate(problem(), paths[i].second);
      if (pathValid) {
        roadmap()->addEdge(nearNodes[i], qnew, paths[i].second);
        edge =
            roadmap()->addEdge(qnew, nearNodes[i], paths[i].second->reverse());
        setParent(parentMap, nearNodes[i], edge, false);
      }
    }
  }
  return true;
}

bool BiRrtStar::connect(NodePtr_t b, ParentMap_t& parentMap,
                        const Configuration_t& q) {
  Configuration_t qnew;
  // while extend did not reach q
  while (roadmap()->connectedComponents().size() == 2) {
    qnew = q;
    if (!extend(b, parentMap, qnew))  // extend failed
      return false;
  }
  return true;
}

bool BiRrtStar::improve(const Configuration_t& q) {
  value_type dist;
  const NodePtr_t nearQ = roadmap()->nearestNode(q, dist);
  if (dist < 1e-16) return false;

  const PathPtr_t nearQ_qnew =
      buildPath(*nearQ->configuration(), q, extendMaxLength_, true);
  if (!nearQ_qnew || nearQ_qnew->length() < minimalPathLength_) return false;

  const Configuration_t qnew(nearQ_qnew->end());

  const value_type n((value_type)roadmap()->nodes().size());
  NodeVector_t nearNodes = roadmap()->nodesWithinBall(
      qnew, roots_[0]->connectedComponent(),
      std::min(
          gamma_ * std::pow(std::log(n) / n,
                            1. / (value_type)problem()->robot()->numberDof()),
          extendMaxLength_));

  const NodePtr_t nnew = roadmap()->addNode(make_shared<Configuration_t>(qnew));

  std::vector<ValidatedPath_t> paths;
  paths.reserve(nearNodes.size());

  for (int k = 0; k < 2; ++k) {
    paths.clear();

    NodePtr_t bestParent(nearQ);
    PathPtr_t best_qnew(nearQ_qnew);
    value_type cost_q(computeCost(toRoot_[k], nearQ) + nearQ_qnew->length());

    for (NodeVector_t::const_iterator _near = nearNodes.begin();
         _near != nearNodes.end(); ++_near) {
      PathPtr_t near2new;
      if (*_near == nearQ) {
        near2new = nearQ_qnew;
        paths.push_back(ValidatedPath_t(true, near2new));
        continue;
      } else {
        near2new = buildPath(*(*_near)->configuration(), qnew, -1, false);
        paths.push_back(ValidatedPath_t(false, near2new));
      }
      if (!near2new) continue;

      value_type _cost_q = computeCost(toRoot_[k], *_near) + near2new->length();
      if (_cost_q < cost_q) {
        paths.back().first = true;
        // Run path validation
        if (validate(problem(), near2new)) {
          // Path is valid and shorter.
          cost_q = _cost_q;
          bestParent = *_near;
          best_qnew = near2new;
        } else
          paths.back().second.reset();
      }
    }

    EdgePtr_t edge = roadmap()->addEdge(bestParent, nnew, best_qnew);
    roadmap()->addEdge(nnew, bestParent, best_qnew->reverse());
    assert(toRoot_[k].find(bestParent) != toRoot_[k].end());
    if (toRoot_[k].count(nnew)) continue;
    setParent(toRoot_[k], nnew, edge, true);

    for (std::size_t i = 0; i < nearNodes.size(); ++i) {
      if (nearNodes[i] == bestParent || !paths[i].second) continue;

      value_type cost_q_near = cost_q + paths[i].second->length();
      if (cost_q_near < computeCost(toRoot_[k], nearNodes[i])) {
        bool pathValid = paths[i].first;
        if (!pathValid)  // If path validation has not been run
          pathValid = validate(problem(), paths[i].second);
        if (pathValid) {
          roadmap()->addEdge(nearNodes[i], nnew, paths[i].second);
          edge = roadmap()->addEdge(nnew, nearNodes[i],
                                    paths[i].second->reverse());
          assert(toRoot_[k].find(nnew) != toRoot_[k].end());
          setParent(toRoot_[k], nearNodes[i], edge, false);
        }
      }
    }
  }
  return true;
}

// ----------- Declare parameters ------------------------------------- //

HPP_START_PARAMETER_DECLARATION(BiRrtStar)
Problem::declareParameter(ParameterDescription(
    Parameter::FLOAT, "BiRRT*/maxStepLength",
    "The maximum step length when extending. If negative, uses sqrt(dimension)",
    Parameter(-1.)));
Problem::declareParameter(ParameterDescription(Parameter::FLOAT, "BiRRT*/gamma",
                                               "", Parameter(1.)));
Problem::declareParameter(ParameterDescription(
    Parameter::FLOAT, "BiRRT*/minimalPathLength",
    "The minimum length between 2 configurations in the roadmap.",
    Parameter(1e-4)));
HPP_END_PARAMETER_DECLARATION(BiRrtStar)
}  // namespace pathPlanner
}  // namespace core
}  // namespace hpp
