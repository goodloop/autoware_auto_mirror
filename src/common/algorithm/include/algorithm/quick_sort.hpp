/// \copyright Copyright 2019 Silexica GmbH, Lichtstr. 25, Cologne, Germany
/// All rights reserved.
/// \file
/// \brief This file provides an iterative quick sort implementation.
#ifndef ALGORITHM__QUICK_SORT_HPP_
#define ALGORITHM__QUICK_SORT_HPP_

#include <algorithm>
#include <utility>
#include <vector>

namespace autoware
{

namespace common
{

namespace algorithm
{

namespace detail
{

/// \brief Partition range [first, last], based on pivot element last. After
/// execution all elements smaller than the pivot element are left of it and
/// all bigger elements right of it.
/// \param[in] first Start of the range to partition
/// \param[in] last End (included) of the range to partition, used as pivot
/// \return Iterator to the pivot element in the range
template<typename RandomIt, typename Compare>
RandomIt partition(RandomIt first, RandomIt last, Compare comp)
{
  auto prev = first;

  // Iterate over range and swap whenever element is smaller than the pivot
  // element.
  for (auto it = first; it < last; it++) {
    if (comp(it, last)) {
      ::std::iter_swap(it, prev);
      prev++;
    }
  }

  // Swap the pivot element into place
  ::std::iter_swap(prev, last);
  return prev;
}

/// \brief Iterative quick sort implementation using a stack, sorts
/// range [first, last) using the given comparison function
/// \param[in] first Start of the range to sort
/// \param[in] last End of the range to sort (not included)
/// \param[in] comp The comparison function to base the sorting on
/// \param[in] stack Helper stack, needs to have capacity (last-first)+2
template<typename RandomIt, typename Compare, typename Stack>
void quick_sort_iterative(
  RandomIt first, RandomIt last, Compare comp, Stack & stack)
{
  if (::std::distance(first, last) < 2) {
    return;
  }

  // Make sure we do not accidently have an already partially filled stack,
  // capacity does not change
  stack.clear();

  // Add first interval to the stack for sorting, from here on last is really
  // the last element and not the element after, i.e. not end
  stack.push_back(first);
  stack.push_back(last - 1);

  while (!stack.empty()) {
    last = stack.back();
    stack.pop_back();
    first = stack.back();
    stack.pop_back();

    auto part =
      ::autoware::common::algorithm::detail::partition(first, last, comp);

    if (part > first + 1) {
      stack.push_back(first);
      stack.push_back(part - 1);
    }

    if (part < last - 1) {
      stack.push_back(part + 1);
      stack.push_back(last);
    }
  }
}

}  // namespace detail

/// \brief Iterative quick sort implementation using a stack, sorts
/// range [first, last) using the given comparison function
/// \param[in] first Start of the range to sort
/// \param[in] last End of the range to sort (not included)
/// \param[in] comp The comparison function to base the sorting on
/// \param[in] stack Helper stack, needs to have capacity (last-first)+2
template<typename RandomIt, typename Compare, typename Stack>
void quick_sort_iterative(
  RandomIt first, RandomIt last, Stack & stack, Compare comp)
{
  ::autoware::common::algorithm::detail::quick_sort_iterative(
    first, last, __gnu_cxx::__ops::__iter_comp_iter(comp), stack);
}

/// \brief Iterative quick sort implementation using a stack, sorts
/// range [first, last) using the default less operation
/// \param[in] first Start of the range to sort
/// \param[in] last End of the range to sort (not included)
/// \param[in] stack Helper stack, needs to have capacity (last-first)+2
template<typename RandomIt, typename Stack>
void quick_sort_iterative(RandomIt first, RandomIt last, Stack & stack)
{
  ::autoware::common::algorithm::detail::quick_sort_iterative(
    first, last, __gnu_cxx::__ops::__iter_less_iter(), stack);
}

/// \brief Reserves helper stack capacity for the iterative quick sort
/// algorithm based on the capacity of the container to be sorted such that
/// no heap allocation is done during the algorithm.
template<typename Stack>
void quick_sort_iterative_reserve(Stack & stack, ::std::size_t capacity)
{
  // The maximum partition depth is n/2 + 1, which means we need a maximum
  // capacity of n + 2 to hold store the iterators in the stack.
  stack.reserve(capacity + 2);
}

}  // namespace algorithm

}  // namespace common

}  // namespace autoware

#endif  // ALGORITHM__QUICK_SORT_HPP_
