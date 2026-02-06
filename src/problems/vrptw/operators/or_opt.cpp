/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/or_opt.h"

namespace vroom::vrptw {

OrOpt::OrOpt(const Input& input,
             const utils::SolutionState& sol_state,
             TWRoute& tw_s_route,
             Index s_vehicle,
             Index s_rank,
             TWRoute& tw_t_route,
             Index t_vehicle,
             Index t_rank)
  : cvrp::OrOpt(input,
                sol_state,
                static_cast<RawRoute&>(tw_s_route),
                s_vehicle,
                s_rank,
                static_cast<RawRoute&>(tw_t_route),
                t_vehicle,
                t_rank),
    _tw_s_route(tw_s_route),
    _tw_t_route(tw_t_route) {
}

bool OrOpt::is_valid() {
  bool valid =
    cvrp::OrOpt::is_valid() && _tw_s_route.is_valid_removal(_input, s_rank, 2);

  if (valid) {
    // Keep edge direction.
    auto s_start = s_route.begin() + s_rank;
    is_normal_valid =
      is_normal_valid && _tw_t_route.is_valid_addition_for_tw(_input,
                                                              edge_delivery,
                                                              s_start,
                                                              s_start + 2,
                                                              t_rank,
                                                              t_rank);
    // Reverse edge direction.
    auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
    is_reverse_valid = is_reverse_valid &&
                       _tw_t_route.is_valid_addition_for_tw(_input,
                                                            edge_delivery,
                                                            s_reverse_start,
                                                            s_reverse_start + 2,
                                                            t_rank,
                                                            t_rank);

    valid = is_normal_valid || is_reverse_valid;
  }

  if (!valid) {
    return false;
  }

  // Check the global pickup-before-delivery constraint for both routes after the move
  // We need to check both directions (normal and reverse) since the operator
  // will choose the one that gives the best gain

  // Check for normal direction
  bool normal_direction_valid = true;
  if (is_normal_valid) {
    // Check source route after removal
    if (_tw_s_route.would_violate_global_pd_constraint_range(_input, s_rank, s_rank + 2, std::vector<Index>{})) {
      normal_direction_valid = false;
    } else {
      // Check target route after inserting both jobs at t_rank (replace empty interval [t_rank, t_rank))
      std::vector<Index> temp_jobs{s_route[s_rank], s_route[s_rank + 1]};
      if (_tw_t_route.would_violate_global_pd_constraint_range(_input, t_rank, t_rank, temp_jobs)) {
        normal_direction_valid = false;
      }
    }
  }

  // Check for reverse direction
  bool reverse_direction_valid = true;
  if (is_reverse_valid) {
    // Check source route after removal (same as normal)
    if (_tw_s_route.would_violate_global_pd_constraint_range(_input, s_rank, s_rank + 2, std::vector<Index>{})) {
      reverse_direction_valid = false;
    } else {
      // Check target route after adding both jobs in reverse order
      std::vector<Index> temp_jobs{s_route[s_rank + 1], s_route[s_rank]};  // reversed order
      if (_tw_t_route.would_violate_global_pd_constraint_range(_input, t_rank, t_rank, temp_jobs)) {
        reverse_direction_valid = false;
      }
    }
  }

  // The operator is valid if at least one direction preserves the constraint
  return (is_normal_valid && normal_direction_valid) ||
         (is_reverse_valid && reverse_direction_valid);
}

void OrOpt::apply() {
  if (reverse_s_edge) {
    auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
    _tw_t_route.replace(_input,
                        edge_delivery,
                        s_reverse_start,
                        s_reverse_start + 2,
                        t_rank,
                        t_rank);
    _tw_s_route.remove(_input, s_rank, 2);
  } else {
    auto s_start = s_route.begin() + s_rank;
    _tw_t_route
      .replace(_input, edge_delivery, s_start, s_start + 2, t_rank, t_rank);
    _tw_s_route.remove(_input, s_rank, 2);
  }

  // Verify that the global pickup-before-delivery constraint is satisfied after applying the change
  assert(_tw_s_route.has_all_pickups_before_deliveries(_input));
  assert(_tw_t_route.has_all_pickups_before_deliveries(_input));
}

} // namespace vroom::vrptw
