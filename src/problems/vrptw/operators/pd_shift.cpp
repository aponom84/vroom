/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <iterator>
#include <vector>

#include "problems/vrptw/operators/pd_shift.h"
#include "algorithms/local_search/insertion_search.h"

namespace vroom::vrptw {

PDShift::PDShift(const Input& input,
                 const utils::SolutionState& sol_state,
                 TWRoute& tw_s_route,
                 Index s_vehicle,
                 Index s_p_rank,
                 Index s_d_rank,
                 TWRoute& tw_t_route,
                 Index t_vehicle,
                 const Eval& gain_threshold)
  : cvrp::PDShift(input,
                  sol_state,
                  static_cast<RawRoute&>(tw_s_route),
                  s_vehicle,
                  s_p_rank,
                  s_d_rank,
                  static_cast<RawRoute&>(tw_t_route),
                  t_vehicle,
                  gain_threshold),
    _tw_s_route(tw_s_route),
    _tw_t_route(tw_t_route) {
}

void PDShift::compute_gain() {
  // Check for valid removal wrt TW constraints.
  if (const auto delivery_between_pd =
        _tw_s_route.delivery_in_range(_s_p_rank + 1, _s_d_rank);
      !_tw_s_route.is_valid_addition_for_tw(_input,
                                            delivery_between_pd,
                                            s_route.begin() + _s_p_rank + 1,
                                            s_route.begin() + _s_d_rank,
                                            _s_p_rank,
                                            _s_d_rank + 1)) {
    return;
  }

  if (const ls::RouteInsertion rs =
        ls::compute_best_insertion_pd(_input,
                                      _sol_state,
                                      s_route[_s_p_rank],
                                      t_vehicle,
                                      _tw_t_route,
                                      s_gain - stored_gain);
      rs.eval != NO_EVAL) {
    _valid = true;
    t_gain -= rs.eval;
    stored_gain = s_gain + t_gain;
    _best_t_p_rank = rs.pickup_rank;
    _best_t_d_rank = rs.delivery_rank;
    _best_t_delivery = rs.delivery;
  }
  gain_computed = true;
}

bool PDShift::is_valid() {
  // First check the original conditions
  if (!cvrp::PDShift::is_valid()) {
    return false;
  }

  // PDShift::is_valid relies on compute_gain() having run and found a feasible insertion.
  // Use a runtime guard (asserts may be compiled out).
  if (!gain_computed || !_valid) {
    return false;
  }
  assert(gain_computed);

  // Fail closed: add missing rank-range validity checks
  if (_best_t_p_rank > _best_t_d_rank) return false;
  if (static_cast<std::size_t>(_best_t_p_rank) > _tw_t_route.route.size()) return false;
  if (static_cast<std::size_t>(_best_t_d_rank) > _tw_t_route.route.size()) return false;
  if (_s_p_rank > _s_d_rank) return false;
  if (static_cast<std::size_t>(_s_p_rank) >= _tw_s_route.route.size()) return false;
  if (static_cast<std::size_t>(_s_d_rank) >= _tw_s_route.route.size()) return false;

  // Check the global pickup-before-delivery constraint for both routes after the move efficiently
  // Fail closed: if bounds are invalid, return false immediately
  if (static_cast<std::size_t>(_s_p_rank + 2) > _tw_s_route.route.size() && _s_d_rank == _s_p_rank + 1) {
    return false;  // Invalid bounds for simple case
  }
  if (static_cast<std::size_t>(_s_d_rank + 1) > _tw_s_route.route.size() && _s_d_rank != _s_p_rank + 1) {
    return false;  // Invalid bounds for complex case
  }
  if (static_cast<std::size_t>(_best_t_d_rank) > _tw_t_route.route.size()) {
    return false;  // Invalid bounds for target route
  }

  // Check source route after removal - depends on whether pickup and delivery are adjacent
  if (_s_d_rank == _s_p_rank + 1) {
    // Simple case: pickup and delivery are adjacent, remove range [_s_p_rank, _s_p_rank + 2)
    if (_tw_s_route.would_violate_global_pd_constraint_range(_input, _s_p_rank, _s_p_rank + 2, std::vector<Index>{})) {
      return false;
    }
  } else {
    // Complex case: pickup and delivery have jobs in between
    // Remove range [_s_p_rank, _s_d_rank + 1) and replace with jobs in between
    std::vector<Index> source_without_pd;
    for (Index i = _s_p_rank + 1; i < _s_d_rank && i < _tw_s_route.route.size(); ++i) {
      source_without_pd.push_back(_tw_s_route.route[i]);
    }
    if (_tw_s_route.would_violate_global_pd_constraint_range(_input, _s_p_rank, _s_d_rank + 1, source_without_pd)) {
      return false;
    }
  }

  // Check target route after addition
  std::vector<Index> target_jobs_to_add;
  target_jobs_to_add.push_back(_tw_s_route.route[_s_p_rank]);
  for (Index i = _best_t_p_rank; i < _best_t_d_rank && i < _tw_t_route.route.size(); ++i) {
    target_jobs_to_add.push_back(_tw_t_route.route[i]);
  }
  target_jobs_to_add.push_back(_tw_s_route.route[_s_d_rank]);

  if (_tw_t_route.would_violate_global_pd_constraint_range(_input, _best_t_p_rank, _best_t_d_rank, target_jobs_to_add)) {
    return false;
  }

  return true;
}

void PDShift::apply() {
  std::vector<Index> target_with_pd;
  target_with_pd.reserve(_best_t_d_rank - _best_t_p_rank + 2);
  target_with_pd.push_back(s_route[_s_p_rank]);

  std::copy(t_route.begin() + _best_t_p_rank,
            t_route.begin() + _best_t_d_rank,
            std::back_inserter(target_with_pd));
  target_with_pd.push_back(s_route[_s_d_rank]);

  _tw_t_route.replace(_input,
                      _best_t_delivery,
                      target_with_pd.begin(),
                      target_with_pd.end(),
                      _best_t_p_rank,
                      _best_t_d_rank);

  if (_s_d_rank == _s_p_rank + 1) {
    _tw_s_route.remove(_input, _s_p_rank, 2);
  } else {
    std::vector<Index> source_without_pd(s_route.begin() + _s_p_rank + 1,
                                         s_route.begin() + _s_d_rank);

    _tw_s_route.replace(_input,
                        _tw_s_route.delivery_in_range(_s_p_rank + 1, _s_d_rank),
                        source_without_pd.begin(),
                        source_without_pd.end(),
                        _s_p_rank,
                        _s_d_rank + 1);
  }

  // Verify that the global pickup-before-delivery constraint is satisfied after applying the change
  assert(_tw_s_route.has_all_pickups_before_deliveries(_input));
  assert(_tw_t_route.has_all_pickups_before_deliveries(_input));
}

} // namespace vroom::vrptw
