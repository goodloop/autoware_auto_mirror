Proposal for integration tests as cargo-delivery ODD
====================================================

[TOC]

# Motivation

In order to build a robust AV system in a scalable manner, testing at various levels is essential.
In this proposal, suggestions are made for automated integration and system-level tests that can run
in CI.

Much of AV career can be spent on debugging functional issues when putting a system of nodes together (which is no fun),
so unit tests alone do not solidify the algorithm functionality of a whole system or sub-system. An AV system is very
complex, so how different algorithms work together in a system must be tested together as "sub-system" tests. All it
takes is external factors, like wrong calibration, or configuration of the modules working together, or colleagues
working on different parts of the system mis-understanding one another, to make a sub-system fail while the unit tests
pass. Only once such tests have been in place have we observed AV systems starting to become stable with reduced
debugging times in the vehicle itself and more problems being caught early in the CI.

Out of scope: creating simulation scenarios to exercise the complete system within an ODD. This is essential to
development e.g. at Aurora or Waymo but deserves special treatment.

# Current status in Autoware Auto

## Documentation

There is a document about integration testing in the documentation
https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/integration-testing.html

It contains general information about integration testing, which is worth preserving. But it also
spends a great deal about explaining the `integration_tests` framework that spun out of Apex.AI.

There is one paragraph on integration testing in
https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/contributor-guidelines.html#integration-tests-in-ros
that proposes `launch_testing` as the framework for integration, in line with Apex.AI internal
guidelines.

## Available tests

There seems to be no more use of `integration_tests`.

`launch_testing` is used in

- `state_estimation_node`
- `lidar_integration`
- `spinnaker_camera_node`
- `velodyne` driver
- `pure_pursuit`

There are no integration tests for many components of the stack of the AVP system. Relying only on in-vehicle testing is
expensive and error prone, as a commit that breaks a subsystem may go unnoticed for months before the next round of
in-vehicle testing.

# Current status at Apex.AI

The same transition to `launch_testing` was made at Apex.AI. It has helped to create more
integration tests and a few shortcomings have surfaced that need to be kept in mind at Autoware.Auto
as well.

Integration tests typically run on a stressed CI system, with multiple tests potentially executed
concurrently. This means the tests can take much longer than on the system on which the developer
created the test. More to come as we resolve this at Apex.AI; lessons to cope with this environment include:

## Timing

Any test that relies on `sleep` for some time so another node hopefully has enough time to process
is flaky and will eventually lead to stalling pipelines and delaying unrelated merge requests.

Instead, the integration tests should be data-driven in the sense that tests only begin to assert
when actual output is produced. Overall time-outs of a test are still sensible to prevent a test
from running forever because the system under test ended up in a deadlock.

A poor man's fix is to increase `sleep` durations to make a flaky test pass one more time. It
increases the average run time of the test, the cost for CI, and at some point the CI machine is
under so much stress that even the larger duration is not long enough.

A potential cure for the flakiness is to control the time source, ROS time, from the test framework.
This way, time outs in the system under test can be enforced/injected to ensure proper system
behavior in such failure conditions.

## Performance

Tests that run in CI shall not measure the desired performance of the system such as output rates
etc. Dedicated hardware is needed for such tests. Instead, flaky integration tests should simply
assert on receiving any input/output, and potentially not even assert on the numbers of the
input/output.

# Proposed actions

## Documentation

https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/integration-testing.html is revamped

- to drop reference to `integration_tests` framework
- to recommend `launch_testing`

In the contributor-guidelines, the article on integration testing is referenced.

Integration tests are mandatory for adding a new node/feature for the cargo-delivery ODD.

## New code

New nodes should come with unit tests and at least two integration tests:

1. A smoke test ensures that the node can be
    1. launched with its default configuration and doesn't crash
    1. shut down through a `SIGINT` signal with the corresponding process return code.
1. An interface test ensures that a node consumes/produces the desired output in line with the high-level documentation
   of its design document.

## ODD development

As one ore more nodes become part of the ODD system, the system architects should propose reasonable subsystems that
need to be tested in subsytem tests.
