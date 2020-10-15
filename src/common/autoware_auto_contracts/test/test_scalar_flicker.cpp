/*
 * Copyright 2020 Mapless AI, Inc.
 *
 * Proprietary License
 */
#include <gtest/gtest.h>

#include "autoware_auto_contracts/scalar_flicker.hpp"

namespace mapless
{
namespace demo
{
namespace sotif
{

//------------------------------------------------------------------------------

TEST(SOTIF_API_Demo, PeakIndex_indices_are_valid) {
  EXPECT_THROW({(PeakIndex<float, 3>{0, 0, 1});}, std::runtime_error);
  EXPECT_THROW({(PeakIndex<float, 3>{3, 5, 5});}, std::runtime_error);
  EXPECT_THROW({(PeakIndex<float, 3>{3, 2, 1});}, std::runtime_error);
  EXPECT_THROW({(PeakIndex<float, 3>{2, 2, 2});}, std::runtime_error);
  EXPECT_NO_THROW(
        {
          (PeakIndex<float, 3>{0, 1, 2});
          (PeakIndex<float, 3>{0, 1, 20});
          (PeakIndex<float, 3>{3, 5, 17});
        });
}

//------------------------------------------------------------------------------

TEST(SOTIF_API_Demo, PeakIndex_is_valid_for_array) {
  const std::array<float, 3> arr = {0.0f, 1.0f, 0.0f};

  {
    const auto peak = PeakIndex<float, 3>(0, 1, 2);
    EXPECT_TRUE((PeakIndex<float, 3>::is_valid_for_array(arr, peak)));
  }

  {
    const auto peak = PeakIndex<float, 3>(0, 1, 5);
    EXPECT_FALSE((PeakIndex<float, 3>::is_valid_for_array(arr, peak)));
  }
}

//------------------------------------------------------------------------------

TEST(SOTIF_API_Demo, is_peak) {
  EXPECT_TRUE(is_peak(0, 1, 0));
  EXPECT_TRUE(is_peak(0, 3, 2));
  EXPECT_TRUE(is_peak(1, 3, -1));
  EXPECT_FALSE(is_peak(0, 0, 0));
  EXPECT_FALSE(is_peak(0, -1, 0));
  EXPECT_FALSE(is_peak(1, -1, 0));
  EXPECT_FALSE(is_peak(0, -1, 1));
}

//------------------------------------------------------------------------------

TEST(SOTIF_API_Demo, is_valley) {
  EXPECT_FALSE(is_valley(0, 1, 0));
  EXPECT_FALSE(is_valley(0, 3, 2));
  EXPECT_FALSE(is_valley(1, 3, -1));
  EXPECT_FALSE(is_valley(0, 0, 0));
  EXPECT_TRUE(is_valley(0, -1, 0));
  EXPECT_TRUE(is_valley(1, -1, 0));
  EXPECT_TRUE(is_valley(0, -1, 1));
}

//------------------------------------------------------------------------------

TEST(SOTIF_API_Demo, peak_mag) {
  {
    const auto expected_mag = 2.0f;
    const auto mag = peak_mag(1.0f, 3.0f, 0.0f);
    EXPECT_EQ(mag, expected_mag);
  }

  {
    const auto expected_mag = 2.0f;
    const auto mag = peak_mag(0.0f, 3.0f, 1.0f);
    EXPECT_EQ(mag, expected_mag);
  }

  {
    const auto expected_mag = 2.0f;
    const auto mag = peak_mag(-3.0f, 3.0f, 1.0f);
    EXPECT_EQ(mag, expected_mag);
  }

  {
    const auto expected_mag = 6.0f;
    const auto mag = peak_mag(-3.0f, 3.0f, -5.0f);
    EXPECT_EQ(mag, expected_mag);
  }
}

//------------------------------------------------------------------------------

TEST(SOTIF_API_Demo, valley_mag) {
  {
    const auto expected_mag = 6.0f;
    const auto mag = valley_mag(3.0f, -3.0f, 5.0f);
    EXPECT_EQ(mag, expected_mag);
  }

  {
    const auto expected_mag = 3.0f;
    const auto mag = valley_mag(0.0f, -3.0f, 5.0f);
    EXPECT_EQ(mag, expected_mag);
  }

  {
    const auto expected_mag = 3.0f;
    const auto mag = valley_mag(5.0f, -3.0f, 0.0f);
    EXPECT_EQ(mag, expected_mag);
  }

  {
    const auto expected_mag = 3.0f;
    const auto mag = valley_mag(0.0f, -3.0f, 0.0f);
    EXPECT_EQ(mag, expected_mag);
  }
}

//------------------------------------------------------------------------------

TEST(SOTIF_API_Demo, rotate_left_and_set_back) {
  std::array<float, 4> arr = {0.0f, 1.0f, 2.0f, 3.0f};
  const std::array<float, 4> arr_expected = {1.0f, 2.0f, 3.0f, 4.0f};
  rotate_left_and_set_back(arr, 4.0f);
  for (auto i = 0; i < 4; ++i) {
    EXPECT_EQ(arr[i], arr_expected[i]);
  }
}

//------------------------------------------------------------------------------

TEST(SOTIF_API_Demo, greatest_magnitude_peak) {
  const std::array<float, 9> arr = {-1.5f, 1.0f, -2.0f, -3.0f, -1.0f, -5.0f, 0.0f, 1.0f, 2.0f};

  {
    const auto peak = greatest_magnitude_peak(arr);
    const auto mag = get_peak_magnitude(arr, peak);
    EXPECT_EQ(mag, 6.0f) << "peak: " << peak;
  }

  {
    constexpr auto begin = 0;
    constexpr auto end = arr.size();
    const auto peak = greatest_magnitude_peak(arr, begin, end);
    const auto mag = get_peak_magnitude(arr, peak);
    EXPECT_EQ(mag, 3.5f);
  }

  {
    constexpr auto begin = 0;
    constexpr auto end = 3;
    const auto peak = greatest_magnitude_peak(arr, begin, end);
    const auto mag = get_peak_magnitude(arr, peak);
    EXPECT_EQ(mag, 2.5f);
  }

  {
    constexpr auto begin = 6;
    constexpr auto end = arr.size();
    const auto peak = greatest_magnitude_peak(arr, begin, end);
    const auto mag = get_peak_magnitude(arr, peak);
    EXPECT_EQ(mag, 0.0f);
  }
}

//------------------------------------------------------------------------------

TEST(SOTIF_API_Demo, ScalarFlicker) {
  constexpr auto FLICKER_LIMIT = 0.5f;
  auto sf = ScalarFlicker<float, 3>(FLICKER_LIMIT);

  EXPECT_TRUE(sf.no_flicker(0.0f));
  EXPECT_TRUE(sf.no_flicker(1.0f));
  EXPECT_TRUE(sf.no_flicker(2.0f));
  EXPECT_FALSE(sf.no_flicker(1.0f));
  EXPECT_TRUE(sf.no_flicker(0.0f));
  EXPECT_TRUE(sf.no_flicker(-1.0f));
  EXPECT_FALSE(sf.no_flicker(0.0f));
}

//------------------------------------------------------------------------------

}  // namespace sotif
}  // namespace demo
}  // namespace mapless
