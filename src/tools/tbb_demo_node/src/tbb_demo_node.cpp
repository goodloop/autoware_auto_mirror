// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "tbb_demo_node/tbb_demo_node.hpp"

#include <iostream>
#include <vector>
#include <algorithm>
#include <execution>
#include <chrono>
#include <tbb/iterators.h>
#include "tbb_demo_node/time_keeper_sequental.hpp"

namespace autoware
{

int32_t tbb_demo_node::print_hello()
{

  TimeKeeperSequental time_keeper_sequental("test_parallel_stl");
  time_keeper_sequental.AddTimePoint("Start");

  const size_t count_nums = 1000000000UL;
  tbb::counting_iterator<uint64_t> cnt0(0), cntN(count_nums);
  time_keeper_sequental.AddTimePoint("Create counting iterators");

  std::vector<size_t> nums(count_nums);
  time_keeper_sequental.AddTimePoint("Allocate vector");

  std::copy(std::execution::par, cnt0, cntN, nums.begin());
  time_keeper_sequental.AddTimePoint("Generate numbers");

  // Sum with par_unseq
  uint64_t x_par_unseq = std::reduce(
    std::execution::par_unseq,
    nums.begin(),
    nums.end(),
    0UL,
    std::plus<>());

  std::cout << "x_par_unseq: " << x_par_unseq << std::endl;
  time_keeper_sequental.AddTimePoint("std::execution::par_unseq");

  // Sum with par
  uint64_t x_par = std::reduce(
    std::execution::par,
    nums.begin(),
    nums.end(),
    0UL,
    std::plus<>());

  std::cout << "x_par: " << x_par << std::endl;
  time_keeper_sequental.AddTimePoint("std::execution::par");

  // Sum with seq
  uint64_t x_seq = std::reduce(
    std::execution::seq,
    nums.begin(),
    nums.end(),
    0UL,
    std::plus<>());

  time_keeper_sequental.AddTimePoint("std::execution::seq");
  std::cout << "x_seq: " << x_seq << std::endl;

  time_keeper_sequental.PrintTimes();

  return 0;
}

}  // namespace autoware
