#include "algorithm/algorithm.hpp"
#include <gtest/gtest.h>

TEST(quick_sort_iterative, empty) {
  ::std::vector<int> vector;
  ::std::vector<::std::vector<int>::iterator> stack;
  ::autoware::common::algorithm::quick_sort_iterative(vector.begin(),
                                                      vector.end(), stack);
  ASSERT_EQ(vector, ::std::vector<int>({}));
  ASSERT_EQ(stack.capacity(), 0);
}

TEST(quick_sort_iterative, single_elem) {
  ::std::vector<int> vector = {42};
  ::std::vector<::std::vector<int>::iterator> stack;
  ::autoware::common::algorithm::quick_sort_iterative_reserve(
      stack, vector.capacity());
  ASSERT_EQ(stack.capacity(), 3);
  ::autoware::common::algorithm::quick_sort_iterative(vector.begin(),
                                                      vector.end(), stack);
  ASSERT_EQ(vector, std::vector<int>({42}));
  ASSERT_EQ(stack.capacity(), 3);
}

TEST(quick_sort_iterative, two_elems) {
  ::std::vector<int> vector = {43, 42};
  ::std::vector<::std::vector<int>::iterator> stack;
  ::autoware::common::algorithm::quick_sort_iterative_reserve(
      stack, vector.capacity());
  ASSERT_EQ(stack.capacity(), 4);
  ::autoware::common::algorithm::quick_sort_iterative(vector.begin(),
                                                      vector.end(), stack);
  ASSERT_EQ(vector, ::std::vector<int>({42, 43}));
  ASSERT_EQ(stack.capacity(), 4);
}

TEST(quick_sort_iterative, already_sorted) {
  ::std::vector<int> vector = {1, 2, 3, 4, 5, 6};
  ::std::vector<::std::vector<int>::iterator> stack;
  ::autoware::common::algorithm::quick_sort_iterative_reserve(
      stack, vector.capacity());
  ASSERT_EQ(stack.capacity(), 8);
  ::autoware::common::algorithm::quick_sort_iterative(vector.begin(),
                                                      vector.end(), stack);
  ASSERT_EQ(vector, ::std::vector<int>({1, 2, 3, 4, 5, 6}));
  ASSERT_EQ(stack.capacity(), 8);
}

TEST(quick_sort_iterative, descending) {
  ::std::vector<int> vector = {6, 5, 4, 3, 2, 1};
  ::std::vector<::std::vector<int>::iterator> stack;
  ::autoware::common::algorithm::quick_sort_iterative_reserve(
      stack, vector.capacity());
  ASSERT_EQ(stack.capacity(), 8);
  ::autoware::common::algorithm::quick_sort_iterative(vector.begin(),
                                                      vector.end(), stack);
  ASSERT_EQ(vector, ::std::vector<int>({1, 2, 3, 4, 5, 6}));
  ASSERT_EQ(stack.capacity(), 8);
}

TEST(quick_sort_iterative, random) {
  ::std::vector<int> vector = {3, 5, 1, 6, 4, 2};
  ::std::vector<::std::vector<int>::iterator> stack;
  ::autoware::common::algorithm::quick_sort_iterative_reserve(
      stack, vector.capacity());
  ASSERT_EQ(stack.capacity(), 8);
  ::autoware::common::algorithm::quick_sort_iterative(vector.begin(),
                                                      vector.end(), stack);
  ASSERT_EQ(vector, ::std::vector<int>({1, 2, 3, 4, 5, 6}));
  ASSERT_EQ(stack.capacity(), 8);
}

TEST(quick_sort_iterative, sub_range_begin) {
  ::std::vector<int> vector = {3, 5, 1, 6, 4, 2};
  ::std::vector<::std::vector<int>::iterator> stack;
  ::autoware::common::algorithm::quick_sort_iterative_reserve(
      stack, vector.capacity());
  ASSERT_EQ(stack.capacity(), 8);
  ::autoware::common::algorithm::quick_sort_iterative(vector.begin(),
                                                      vector.end()-2, stack);
  ASSERT_EQ(vector, ::std::vector<int>({1, 3, 5, 6, 4, 2}));
  ASSERT_EQ(stack.capacity(), 8);
}

TEST(quick_sort_iterative, sub_range_end) {
  ::std::vector<int> vector = {3, 5, 1, 6, 4, 2};
  ::std::vector<::std::vector<int>::iterator> stack;
  ::autoware::common::algorithm::quick_sort_iterative_reserve(
      stack, vector.capacity());
  ASSERT_EQ(stack.capacity(), 8);
  ::autoware::common::algorithm::quick_sort_iterative(vector.begin()+2,
                                                      vector.end(), stack);
  ASSERT_EQ(vector, ::std::vector<int>({3, 5, 1, 2, 4, 6}));
  ASSERT_EQ(stack.capacity(), 8);
}

TEST(quick_sort_iterative, sub_range) {
  ::std::vector<int> vector = {3, 5, 1, 6, 4, 2};
  ::std::vector<::std::vector<int>::iterator> stack;
  ::autoware::common::algorithm::quick_sort_iterative_reserve(
      stack, vector.capacity());
  ASSERT_EQ(stack.capacity(), 8);
  ::autoware::common::algorithm::quick_sort_iterative(vector.begin()+1,
                                                      vector.end()-3, stack);
  ASSERT_EQ(vector, ::std::vector<int>({3, 1, 5, 6, 4, 2}));
  ASSERT_EQ(stack.capacity(), 8);
}