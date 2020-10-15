Contracts {#autoware_auto_contracts-package-design}
===========

This package provides data types for use in contract enforcement and acts as a common interface to the [Contracts Lite](https://github.com/ros-safety/contracts_lite) library for use in Autoware.Auto.

The provided data types can be used to perform per-parameter contract enforcement on input/output parameters.
In addition, the types can be used to perform basic dimensional analysis.


# Purpose / Use cases

Suppose there is a function that takes some floating point values as inputs and needs to do error checking on both input and output to ensure proper functioning:

```
float foo(float height, float deg, float scalar, size_t count)
{
   if (height < 0.0f) { throw ... }
   if ((angle_deg < 0.0f) || (angle_deg >= 90.0f)) { throw ... }
   if (!std::isfinite(arg3)) { throw ... }
   if (count > SOME_BOUND) { throw ... }
   
   // Convert degrees to radians
   auto rad = some_conversion_function(deg);
   
   // do some work
   
   if (!std::isfinite(bar)) { throw ... }
   if (bar <= 0.0f) { throw ... }
   return bar;
}
```

These types of checks can be very verbose, making the code difficult to read.
It can also be very error prone to maintain them over time.
The contract types provided by this library allow all of that error checking to be written _once_ and maintained in the _data type_:

```
StrictlyPositiveRealf foo(NonnegativeRealf height, AcuteDegreef deg, Realf scalar, SizeBound<SOME_BOUND> count)
{  
   // Convert degrees to radians
   AcuteRadianf rad = deg;
   
   // do some work
   
   return bar;
}
```

Not only is the code far less verbose, but it is much easier to maintain and the informative names make the code more readable.
Further, conversion between types can be built into the type itself.
For example, in the above code, conversion from degree to radian (or vice versa), is as simple as assigning one type to the other.

# Design

Because the [Contracts Lite](https://github.com/ros-safety/contracts_lite) library uses compiler definitions to determine contract enforcement levels, the contracts library is implemented as a header-only library.
This prevents any mismatch in contract enforcement between a compiled contracts library and a compiled user library.

Each type is essentially a templated wrapper around another type.
For example, one of the types is `Real<T>`, which is intended to represent a finite floating point value.
To improve usability, the types allow implicit constructor conversion.
For example, the `foo` function defined above could be called as follows:

```
size_t count;
float height, deg, scalar;
auto bar = foo(height, deg, scalar, count);
```

Arguments will all be converted to the corresponding contract types and contracts will be enforced at run time.
A cast operator is also provided to aid template type deduction in certain cases:

```
auto bar = foo(height, deg, scalar, count);

float val = 1.0f;
auto max = std::max(val, static_cast<float>(bar));
```

Finally, contract violations will cause the violation handler to either log an error or throw an excpetion (see [Error detection and handling](#error-detection-and-handling)). When `DEFAULT` build level is used, the generated messages are limited to compile-time information, such as file and line number. When `AUDIT` build level is used, run time information, such as variable values, is additionally included.

## Assumptions / Known limits

Users should be aware that contract enforcement, while generally cheap, is not free.
In performance sensitive applications it can make sense to move contract checks out of hot spots.
For example, the below code constructs a contract type at every iteration:

```
Realf foo(Realf base, Realf step) {
  for (auto i=0; i<99999999; ++i) {
    base = base + step;
  }
  return base;
}
```

Unless it's required to enforce the contract at each iteration, a logically equivalent, but more performant version of the code would be:

```
Realf foo(Realf base, Realf step) {
  float b = base;
  float s = step;
  for (auto i=0; i<99999999; ++i) {
    b = b + s;
  }
  return b;
}
```

Also be aware that using the `AUDIT` build level (see [Error detection and handling](#error-detection-and-handling)) can significantly impact performance.
This is because verbose comment message are generated each time a contract is checked.

## Inputs / Outputs / API

To see all available contract types, check the `include` directory of the contracts package.


## Error detection and handling

Contract violations are handled by the violation handler specified in the package.
The behavior of the handler is determined by the compile defines described in the [Contracts Lite](https://github.com/ros-safety/contracts_lite) library.
The two defines control the "continuation mode" and "build level" of the contract enforced.
These concepts and their implementation are described below:

- Continuation mode OFF: Violations are logged with the `RCLCPP_FATAL` macro and the process is terminated. This is the preferred mode for library code.
- Continuation mode ON: Violations result in a runtime error exception being thrown. This mode should only be used for unit testing and only for timing considerations: `EXPECT_DEATH` tests are very slow to run as opposed to `EXPECT_THROW`. On a desktop computer, for example, the unit tests in this package take over `5600ms` to run with continuation mode `OFF` but less than `1ms` with continuation mode `ON`.

- Build level DEFAULT: All default enforced contracts are checked.
- Build level AUDIT: All contracts (default and audit) are checked.
- Build level OFF: Contract enforcement is completely compiled out; no checks are performed.

If no compile time defines are provided, the behavior of contract enforced is continuation mode `OFF` and build level `DEFAULT`.


# References / External links

- [https://github.com/ros-safety/contracts_lite](https://github.com/ros-safety/contracts_lite)
- [http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2016/p0380r1.pdf](http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2016/p0380r1.pdf)
- [http://open-std.org/JTC1/SC22/WG21/docs/papers/2018/p0542r5.html](http://open-std.org/JTC1/SC22/WG21/docs/papers/2018/p0542r5.html)


# Future extensions / Unimplemented parts

Useful types should be added as needed, and functionality of existing types extended as needed.


# Related issues

- [https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/520](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/520)
