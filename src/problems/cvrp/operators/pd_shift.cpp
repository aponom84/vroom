/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/pd_shift.h"

#include "algorithms/local_search/insertion_search.h"

namespace vroom::cvrp {

PDShift::PDShift(const Input& input,
                 const utils::SolutionState& sol_state,
                 RawRoute& s_route,
                 Index s_vehicle,
                 Index s_p_rank,
                 Index s_d_rank,
                 RawRoute& t_route,
                 Index t_vehicle,
                 const Eval& gain_threshold)
  : Operator(OperatorName::PDShift,
             input,
             sol_state,
             s_route,
             s_vehicle,
             0,
             t_route,
             t_vehicle,
             0),
    _s_p_rank(s_p_rank),
    _s_d_rank(s_d_rank) {
  assert(s_vehicle != t_vehicle);
  assert(s_route.size() >= 2);
  assert(s_p_rank < s_d_rank);
  assert(s_d_rank < s_route.size());
  assert(s_route.route[s_p_rank] + 1 == s_route.route[s_d_rank]);

  s_gain = _sol_state.pd_gains[s_vehicle][_s_p_rank];

  if (t_route.empty()) {
    t_gain.cost -= _input.vehicles[t_vehicle].fixed_cost();
  }

  assert(_input.vehicles[s_vehicle].ok_for_range_bounds(
    _sol_state.route_evals[s_vehicle] - s_gain));

  stored_gain = gain_threshold;
}

void PDShift::compute_gain() {
  if (const ls::RouteInsertion rs =
        ls::compute_best_insertion_pd(_input,
                                      _sol_state,
                                      s_route[_s_p_rank],
                                      t_vehicle,
                                      target,
                                      s_gain - stored_gain);
      rs.eval != NO_EVAL) {
    _valid = true;
    t_gain -= rs.eval;
    stored_gain = s_gain + t_gain;
    _best_t_p_rank = rs.pickup_rank;
    _best_t_d_rank = rs.delivery_rank;
  }

  gain_computed = true;
}

bool PDShift::is_valid() {
  assert(gain_computed);

  if (!_valid) {
    return false;
  }

  // Check the global pickup-before-delivery constraint for both routes after the move efficiently
  // Fail closed: if bounds are invalid, return false immediately
  if (static_cast<std::size_t>(_s_p_rank + 2) > source.route.size() && _s_d_rank == _s_p_rank + 1) {
    return false;  // Invalid bounds for simple case
  }
  if (static_cast<std::size_t>(_s_d_rank + 1) > source.route.size() && _s_d_rank != _s_p_rank + 1) {
    return false;  // Invalid bounds for complex case
  }
  if (static_cast<std::size_t>(_best_t_d_rank) > target.route.size()) {
    return false;  // Invalid bounds for target route
  }

  // Check source route after removal - depends on whether pickup and delivery are adjacent
  if (_s_d_rank == _s_p_rank + 1) {
    // Simple case: pickup and delivery are adjacent, remove range [_s_p_rank, _s_p_rank + 2)
    if (source.would_violate_global_pd_constraint_range(_input, _s_p_rank, _s_p_rank + 2, std::vector<Index>{})) {
      return false;
    }
  } else {
    // Complex case: pickup and delivery have jobs in between
    // Remove range [_s_p_rank, _s_d_rank + 1) and replace with jobs in between
    std::vector<Index> source_without_pd;
    for (Index i = _s_p_rank + 1; i < _s_d_rank && i < s_route.size(); ++i) {
      source_without_pd.push_back(s_route[i]);
    }
    if (source.would_violate_global_pd_constraint_range(_input, _s_p_rank, _s_d_rank + 1, source_without_pd)) {
      return false;
    }
  }

  // Check target route after addition
  std::vector<Index> target_jobs_to_add;
  target_jobs_to_add.push_back(s_route[_s_p_rank]);
  for (Index i = _best_t_p_rank; i < _best_t_d_rank && i < t_route.size(); ++i) {
    target_jobs_to_add.push_back(t_route[i]);
  }
  target_jobs_to_add.push_back(s_route[_s_d_rank]);

  if (target.would_violate_global_pd_constraint_range(_input, _best_t_p_rank, _best_t_d_rank, target_jobs_to_add)) {
    return false;
  }

  return _valid;
}

void PDShift::apply() {
  std::vector<Index> target_with_pd;
  target_with_pd.reserve(_best_t_d_rank - _best_t_p_rank + 2);
  target_with_pd.push_back(s_route[_s_p_rank]);

  std::copy(t_route.begin() + _best_t_p_rank,
            t_route.begin() + _best_t_d_rank,
            std::back_inserter(target_with_pd));
  target_with_pd.push_back(s_route[_s_d_rank]);

  target.replace(_input,
                 target_with_pd.begin(),
                 target_with_pd.end(),
                 _best_t_p_rank,
                 _best_t_d_rank);

  if (_s_d_rank == _s_p_rank + 1) {
    s_route.erase(s_route.begin() + _s_p_rank, s_route.begin() + _s_p_rank + 2);
    source.update_amounts(_input);
  } else {
    std::vector<Index> source_without_pd(s_route.begin() + _s_p_rank + 1,
                                         s_route.begin() + _s_d_rank);
    source.replace(_input,
                   source_without_pd.begin(),
                   source_without_pd.end(),
                   _s_p_rank,
                   _s_d_rank + 1);
  }
}

std::vector<Index> PDShift::addition_candidates() const {
  return {s_vehicle};
}

std::vector<Index> PDShift::update_candidates() const {
  return {s_vehicle, t_vehicle};
}

} // namespace vroom::cvrp
