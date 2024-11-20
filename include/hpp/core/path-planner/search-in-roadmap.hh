//
// Copyright (c) 2024 CNRS
// Authors: Florent Lamiraux
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

#ifndef HPP_CORE_PATH_PLANNER_SEARCH_IN_ROADMAP_HH
#define HPP_CORE_PATH_PLANNER_SEARCH_IN_ROADMAP_HH

#include <hpp/core/path-planner.hh>
#include <hpp/core/path-planning-failed.hh>

namespace hpp {
namespace core {
namespace pathPlanner {
HPP_PREDEF_CLASS(SearchInRoadmap);
typedef shared_ptr<SearchInRoadmap> SearchInRoadmapPtr_t;
/// \addtogroup path_planning
/// \{

/// Search in the roadmap without modifying it and without trying a direct
/// connection priorly
///
/// This planner is especially useful in some applications where the roadmap is
/// constructed externally like in coverage planning. This planner will return a
/// path if one exists in the roadmap or throw otherwise.
///
/// \note One shortcoming of this planner (as all other planners), if the
/// initial configuration is the same as a goal configuration, the planner will
/// return a path of length 0. This is not desirable in coverage planning
/// applications.
class SearchInRoadmap : public PathPlanner {
 public:
  static SearchInRoadmapPtr_t createWithRoadmap(
      const ProblemConstPtr_t& problem, const RoadmapPtr_t& roadmap) {
    return SearchInRoadmapPtr_t(new SearchInRoadmap(problem, roadmap));
  }
  /// This methods does nothing
  virtual void tryConnectInitAndGoals() {}
  /// This methods does nothing
  virtual void oneStep() {
    throw path_planning_failed(
        "SearchInRoadmap: no goal configuration in the connected component"
        "of initial configuration.");
  }

 protected:
  SearchInRoadmap(const ProblemConstPtr_t& problem)
      : core::PathPlanner(problem) {}
  SearchInRoadmap(const ProblemConstPtr_t& problem, const RoadmapPtr_t& roadmap)
      : PathPlanner(problem, roadmap) {}
};  /// class SearchInRoadmap

/// \}

}  // namespace pathPlanner
}  // namespace core
}  // namespace hpp

#endif  // HPP_CORE_PATH_PLANNER_SEARCH_IN_ROADMAP_HH
