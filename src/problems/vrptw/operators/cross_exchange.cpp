/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/cross_exchange.h"

namespace vroom::vrptw {

CrossExchange::CrossExchange(const Input& input,
                             const utils::SolutionState& sol_state,
                             TWRoute& tw_s_route,
                             Index s_vehicle,
                             Index s_rank,
                             TWRoute& tw_t_route,
                             Index t_vehicle,
                             Index t_rank,
                             bool check_s_reverse,
                             bool check_t_reverse)
  : cvrp::CrossExchange(input,
                        sol_state,
                        static_cast<RawRoute&>(tw_s_route),
                        s_vehicle,
                        s_rank,
                        static_cast<RawRoute&>(tw_t_route),
                        t_vehicle,
                        t_rank,
                        check_s_reverse,
                        check_t_reverse),
    _tw_s_route(tw_s_route),
    _tw_t_route(tw_t_route) {
}

bool CrossExchange::is_valid() {
  bool valid = cvrp::CrossExchange::is_valid();

  if (valid) {
    // Keep target edge direction when inserting in source route.
    auto t_start = t_route.begin() + t_rank;
    s_is_normal_valid =
      s_is_normal_valid && _tw_s_route.is_valid_addition_for_tw(_input,
                                                                target_delivery,
                                                                t_start,
                                                                t_start + 2,
                                                                s_rank,
                                                                s_rank + 2);

    if (check_t_reverse) {
      // Reverse target edge direction when inserting in source route.
      auto t_reverse_start = t_route.rbegin() + t_route.size() - 2 - t_rank;
      s_is_reverse_valid =
        s_is_reverse_valid &&
        _tw_s_route.is_valid_addition_for_tw(_input,
                                             target_delivery,
                                             t_reverse_start,
                                             t_reverse_start + 2,
                                             s_rank,
                                             s_rank + 2);
    }

    valid = s_is_normal_valid || s_is_reverse_valid;
  }

  if (valid) {
    // Keep source edge direction when inserting in target route.
    auto s_start = s_route.begin() + s_rank;
    t_is_normal_valid =
      t_is_normal_valid && _tw_t_route.is_valid_addition_for_tw(_input,
                                                                source_delivery,
                                                                s_start,
                                                                s_start + 2,
                                                                t_rank,
                                                                t_rank + 2);

    if (check_s_reverse) {
      // Reverse source edge direction when inserting in target route.
      auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
      t_is_reverse_valid =
        t_is_reverse_valid &&
        _tw_t_route.is_valid_addition_for_tw(_input,
                                             source_delivery,
                                             s_reverse_start,
                                             s_reverse_start + 2,
                                             t_rank,
                                             t_rank + 2);
    }

    valid = t_is_normal_valid || t_is_reverse_valid;
  }

  if (!valid) {
    return false;
  }

  // Check the global pickup-before-delivery constraint for both routes after the move
  // We need to check both directions for both routes since the operator will choose the best combination

  // Check for source route with normal target insertion
  bool pd_s_normal_valid = true;
  if (s_is_normal_valid) {
    std::vector<Index> target_segment{t_route[t_rank], t_route[t_rank + 1]};
    if (_tw_s_route.would_violate_global_pd_constraint_range(_input, s_rank, s_rank + 2, target_segment)) {
      pd_s_normal_valid = false;
    }
  }

  // Check for source route with reversed target insertion
  bool pd_s_reverse_valid = true;
  if (s_is_reverse_valid) {
    std::vector<Index> target_segment{t_route[t_rank + 1], t_route[t_rank]};  // reversed
    if (_tw_s_route.would_violate_global_pd_constraint_range(_input, s_rank, s_rank + 2, target_segment)) {
      pd_s_reverse_valid = false;
    }
  }

  // Check for target route with normal source insertion
  bool pd_t_normal_valid = true;
  if (t_is_normal_valid) {
    std::vector<Index> source_segment{s_route[s_rank], s_route[s_rank + 1]};
    if (_tw_t_route.would_violate_global_pd_constraint_range(_input, t_rank, t_rank + 2, source_segment)) {
      pd_t_normal_valid = false;
    }
  }

  // Check for target route with reversed source insertion
  bool pd_t_reverse_valid = true;
  if (t_is_reverse_valid) {
    std::vector<Index> source_segment{s_route[s_rank + 1], s_route[s_rank]};  // reversed
    if (_tw_t_route.would_violate_global_pd_constraint_range(_input, t_rank, t_rank + 2, source_segment)) {
      pd_t_reverse_valid = false;
    }
  }

  // The operator is valid if at least one combination of (s_direction, t_direction) is valid
  // considering both TW and PD constraints
  bool nn_valid = s_is_normal_valid && t_is_normal_valid && pd_s_normal_valid && pd_t_normal_valid;
  bool nr_valid = s_is_normal_valid && t_is_reverse_valid && pd_s_normal_valid && pd_t_reverse_valid;
  bool rn_valid = s_is_reverse_valid && t_is_normal_valid && pd_s_reverse_valid && pd_t_normal_valid;
  bool rr_valid = s_is_reverse_valid && t_is_reverse_valid && pd_s_reverse_valid && pd_t_reverse_valid;

  return nn_valid || nr_valid || rn_valid || rr_valid;
}

void CrossExchange::apply() {
  assert(!reverse_s_edge ||
         (_input.jobs[s_route[s_rank]].type == JOB_TYPE::SINGLE &&
          _input.jobs[s_route[s_rank + 1]].type == JOB_TYPE::SINGLE));
  assert(!reverse_t_edge ||
         (_input.jobs[t_route[t_rank]].type == JOB_TYPE::SINGLE &&
          _input.jobs[t_route[t_rank + 1]].type == JOB_TYPE::SINGLE));

  std::vector<Index> t_job_ranks;
  if (!reverse_t_edge) {
    auto t_start = t_route.begin() + t_rank;
    t_job_ranks.insert(t_job_ranks.begin(), t_start, t_start + 2);
  } else {
    auto t_reverse_start = t_route.rbegin() + t_route.size() - 2 - t_rank;
    t_job_ranks.insert(t_job_ranks.begin(),
                       t_reverse_start,
                       t_reverse_start + 2);
  }

  if (!reverse_s_edge) {
    _tw_t_route.replace(_input,
                        source_delivery,
                        s_route.begin() + s_rank,
                        s_route.begin() + s_rank + 2,
                        t_rank,
                        t_rank + 2);
  } else {
    auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
    _tw_t_route.replace(_input,
                        source_delivery,
                        s_reverse_start,
                        s_reverse_start + 2,
                        t_rank,
                        t_rank + 2);
  }

  _tw_s_route.replace(_input,
                      target_delivery,
                      t_job_ranks.begin(),
                      t_job_ranks.end(),
                      s_rank,
                      s_rank + 2);
}

} // namespace vroom::vrptw
