/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <vector>

#include "problems/vrptw/operators/intra_two_opt.h"

namespace vroom::vrptw {

namespace {
inline std::vector<Index> make_reversed_segment(const std::vector<Index>& route,
                                               Index s_rank,
                                               Index t_rank) {
  std::vector<Index> seg(route.begin() + s_rank, route.begin() + (t_rank + 1));
  std::reverse(seg.begin(), seg.end());
  return seg;
}
} // namespace

IntraTwoOpt::IntraTwoOpt(const Input& input,
                         const utils::SolutionState& sol_state,
                         TWRoute& tw_s_route,
                         Index s_vehicle,
                         Index s_rank,
                         Index t_rank)
  : cvrp::IntraTwoOpt(input,
                      sol_state,
                      static_cast<RawRoute&>(tw_s_route),
                      s_vehicle,
                      s_rank,
                      t_rank),
    _tw_s_route(tw_s_route) {
}

bool IntraTwoOpt::is_valid() {
  bool valid = cvrp::IntraTwoOpt::is_valid();

  if (valid) {
    // Defensive assertions to ensure valid range
    assert(s_rank <= t_rank);
    assert(t_rank < s_route.size());

    // Build the exact sequence that will replace [s_rank, t_rank+1) after reversal.
    std::vector<Index> reversed_segment = make_reversed_segment(s_route, s_rank, t_rank);

    valid = _tw_s_route.is_valid_addition_for_tw(_input,
                                                 delivery,
                                                 reversed_segment.begin(),
                                                 reversed_segment.end(),
                                                 s_rank,
                                                 t_rank + 1);

    if (!valid) {
      return false;
    }

    // PD check using the same reversed_segment (no extra allocation).
    // Verify that the PD constraint check operates on the same range as the TW check
    // (this operator requires at least 2 jobs inside the reversed segment)
    assert(s_rank < t_rank - 1);  // Range [s_rank, t_rank+1) is valid and has at least 2 jobs between s_rank and t_rank
    if (_tw_s_route.would_violate_global_pd_constraint_range(_input,
                                                            s_rank,
                                                            t_rank + 1,
                                                            reversed_segment)) {
      return false;
    }

    return true;
  }

  return false;
}

void IntraTwoOpt::apply() {
  // Must match the exact effect of the CVRP operator:
  // reverse(s_route.begin()+s_rank, s_route.begin()+t_rank+1)
  std::vector<Index> reversed = make_reversed_segment(s_route, s_rank, t_rank);

  _tw_s_route.replace(_input,
                      delivery,
                      reversed.begin(),
                      reversed.end(),
                      s_rank,
                      t_rank + 1);
}

} // namespace vroom::vrptw
