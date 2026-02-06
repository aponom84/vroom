/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/
#include <vector>

#include "problems/vrptw/operators/two_opt.h"

namespace vroom::vrptw {

TwoOpt::TwoOpt(const Input& input,
               const utils::SolutionState& sol_state,
               TWRoute& tw_s_route,
               Index s_vehicle,
               Index s_rank,
               TWRoute& tw_t_route,
               Index t_vehicle,
               Index t_rank)
  : cvrp::TwoOpt(input,
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

bool TwoOpt::is_valid() {
  // First check the original conditions
  if (!cvrp::TwoOpt::is_valid() ||
      !_tw_t_route.is_valid_addition_for_tw(_input,
                                           _s_delivery,
                                           s_route.begin() + s_rank + 1,
                                           s_route.end(),
                                           t_rank + 1,
                                           t_route.size()) ||
      !_tw_s_route.is_valid_addition_for_tw(_input,
                                           _t_delivery,
                                           t_route.begin() + t_rank + 1,
                                           t_route.end(),
                                           s_rank + 1,
                                           s_route.size())) {
    return false;
  }

  // Check the global pickup-before-delivery constraint for both routes after the move efficiently
  // The TwoOpt swaps segments after s_rank+1 and t_rank+1, so we need to validate the resulting routes
  // Check source route after swap: [original_0...original_{s_rank}, target_segment_after_t_rank...]
  std::vector<Index> segment_from_target(t_route.begin() + t_rank + 1, t_route.end());
  if (_tw_s_route.would_violate_global_pd_constraint_range(_input, s_rank + 1, s_route.size(), segment_from_target)) {
    return false;
  }

  // Check target route after swap: [original_0...original_{t_rank}, source_segment_after_s_rank...]
  std::vector<Index> segment_from_source(s_route.begin() + s_rank + 1, s_route.end());
  if (_tw_t_route.would_violate_global_pd_constraint_range(_input, t_rank + 1, t_route.size(), segment_from_source)) {
    return false;
  }

  return true;
}

void TwoOpt::apply() {
  std::vector<Index> t_job_ranks;
  t_job_ranks.insert(t_job_ranks.begin(),
                     t_route.begin() + t_rank + 1,
                     t_route.end());

  _tw_t_route.replace(_input,
                      _s_delivery,
                      s_route.begin() + s_rank + 1,
                      s_route.end(),
                      t_rank + 1,
                      t_route.size());
  _tw_s_route.replace(_input,
                      _t_delivery,
                      t_job_ranks.begin(),
                      t_job_ranks.end(),
                      s_rank + 1,
                      s_route.size());

  // Verify that the global pickup-before-delivery constraint is satisfied after applying the change
  assert(_tw_s_route.has_all_pickups_before_deliveries(_input));
  assert(_tw_t_route.has_all_pickups_before_deliveries(_input));
}

} // namespace vroom::vrptw
