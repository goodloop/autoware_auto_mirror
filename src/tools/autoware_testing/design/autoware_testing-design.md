autoware_testing {#autoware_testing-package-design}
===========

This is the design document for the `autoware_testing` package.

# Purpose / Use cases

The package aims to provide a unified way to add standard testing functionality to the package, currently supporting:
- Smoke testing (add_smoke_test): launch a node with default configuration and ensure that it starts up and does not crash.

# Design

Uses `ros_testing` (which is an extension of `launch_testing`) and provides some parametrized, reusable standard tests to run.

## Assumptions / Known limits

Parametrization is limited to package_name. Executable name and expected location of package parameters file are generated in a standardized
manner while the namespace is set as 'test'.

Currently assumes there is a parameters file for the package.

## Inputs / Outputs / API

```
add_smoke_test(<package_name>)
```

# References / External links
- https://en.wikipedia.org/wiki/Smoke_testing_(software)
- https://github.com/ros2/ros_testing
- https://github.com/ros2/launch/blob/master/launch_testing

# Future extensions / Unimplemented parts

Extending parametrization of smoke test.
Adding more types of standard tests.

# Related issues
- Issue #700: add smoke test
