Helper Functions: Comparisons
============

The `comparisons.hpp` library is a simple set of functions for performing approximate numerical comparisons. There is additionally an XOR operator. The intent of the library is to improve readability of code and reduce likelihood of typographical errors when using numerical comparisons.

# Target use cases

The approximate comparisons are intended to be used to check whether two numbers lie within same absolute or relative interval. The `exclusive_or` function will test whether two values cast to different boolean values.

The intended use case for the comparison functions is for comparing floating point values. However, the methods will operate on any data type for which comparison and arithmetic operators are defined.

# Assumptions

The library assumes that input data types have all necessary operators (`static_cast<bool>`, comparisons, and arithmetic) are available. The code will not compile if that assumption is not met.

# Example Usage

```c++
#include "geometry/comparisons.hpp"

#include <iostream>

using namespace autoware::common::geometry;

static constexpr auto epsilon = 0.2;
static constexpr auto relative_epsilon = 0.01;

std::cout << exclusive_or(true, false) << "\n";
// Prints: true

std::cout << approx_rel_eq(1.0, 1.1, epsilon, relative_epsilon));
// Prints: true

std::cout << approx_rel_eq(10000.0, 10010.0, epsilon, relative_epsilon));
// Prints: true

std::cout << approx_eq(4.0, 4.2, epsilon);
// Prints: true

std::cout << approx_ne(4.0, 4.2, epsilon);
// Prints: false

std::cout << approx_zero(0.2, epsilon);
// Prints: false

std::cout << approx_lt(4.0, 4.25, epsilon);
// Prints: true

std::cout << approx_le(1.0, 1.2, epsilon);
// Prints: true

std::cout << approx_gt(1.25, 1.0, epsilon);
// Prints: true

std::cout << approx_ge(0.75, 1.0, epsilon);
// Prints: false
```
