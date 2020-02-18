How to handle errors at the node level {#error-handling}
========

[TOC]

## Goals {#error-handling-1}

This article aims to describe rules on how developers should handle errors in their nodes.  

## Introduction {#how-to-handle-errors-introduction}

For AVP2020, we will start with basic error reporting through ROS2 logging mechanisms.

If you are not yet familiar with the concept of logging in ROS2 and for some examples and demos, see this [ROS2 logging page](https://index.ros.org/doc/ros2/Concepts/Logging/).  

## General tips {#how-to-handle-errors-general}

- Put enough runtime context in the log (including values of crucial variables) to help diagnose the issue (`WARN` and higher levels).

- When writing an `ERROR` or `FATAL` log remember to also handle the situation programmatically: return a meaningful value, throw an exception and handle it somewhere (if exceptions are allowed in your component), transition to an erroneous state in the lifecycle model. Think about the system as a whole and how the situation might affect it and design the best way to adapt, exit or repair & recover.

- When logging in the loop (meaning high frequency spin loops or callbacks of ROS node), use `RCLCPP_DEBUG_THROTTLE` to control how often the message is logged. This can be useful in a situation like logging the ego position not with the frequency of the updates, but in a more human-friendly way (e.g. only each second). If the log would be repetitive (e.g. same `WARN` with each pass), use `RCLCPP_WARN_ONCE` macro.

- Use logging output for automated testing, checking both the normal and erroneous execution. Log values that you can check to determine whether the behavior is as expected (e.g. results of node computations). More on that in a following chapter (link).

- Before submitting a merge requests, check whether your new or changed logs are following the guideline. Eliminate unnecessary logs used when developing a new feature.

## Log severity levels guide {#how-to-handle-errors-severity}

- **`FATAL`** – use this log level when a failure occurs that requires termination of the entire application. `FATAL` errors should only occur in non-optional components and are to be used sparingly. Example: a driver for a crucial piece of hardware fails to initiate or access the device. Note that it is sometimes difficult for a component programmer to determine how component failures affect the entire applications – changes in log severity can be made after a review.
- **`ERROR`** – signals a serious issue with a component, either preventing it from working altogether and thus removing a part of functionality from the entire system or disabling some of component core functionalities. Unlike `FATAL`, `ERROR` logs do not necessarily signal that the entire system went down abnormally. Examples: unable to record drive/sensor data, unable to transform one of sensors’ frames.
- **`WARN`** – signals something unusual or a slight problem which does not cause real harm. It might also inform that a bigger problem is likely to occur in the future, e.g. a resource is running out. Warnings can also signal unusual delays, drop in quality or temporary lack of published data that the component is subscribing to. Warnings can escalate into errors e.g. if delays become unacceptable. `WARN` is different from `INFO` in that it should be investigated, and that our goal is to have zero `WARN`s reported.
- **`INFO`** – informs that some notable (and expected) event occurred, such as a node transition to a distinct state, successful initiation of a component, important service calls. Try to limit the amount of logging on this level and keep it concise and packed with useful information.
- **`DEBUG`** – is to be used for a more detailed information that can assist in debugging. The log output for `DEBUG` level altogether is expected to be quite comprehensive. It helps to find issues by improving visibility of the program flow. The output of `DEBUG` and `INFO` log levels should also be covered with automated testing. `DEBUG` level logs can be used in loops, although developers should consider in each case whether it improves or degrades usability of the entire log.  

## C++ logging macros {#how-to-handle-errors-logging-macros}

Replace `[SEVERITY]` with `DEBUG`, `INFO` etc.
- `RCLCCP_[SEVERITY]` - default macro to use for logging with the use of format string (also accepts a single std::string argument).
- `RCLCPP_[SEVERITY]_STREAM` - default macro to use for using stream (<<) way of constructing the log. This is usually more convenient than using a format string
- `RCLCPP_[SEVERITY]_ONCE` and `RCLCPP_[SEVERITY]_STREAM_ONCE` – use when you only want the log to be called once. Subsequent calls are ignored. This can be useful for warnings, when we are expected to encounter the same situation with each pass.
- `RCLCPP_[SEVERITY]_EXPRESSION` and `RCLCPP_[SEVERITY]_STREAM_EXPRESSION` – log a message when a condition is satisfied (convenient to skip the if clause). Note that it is still necessary to use the explicit if in case you want to do more than log on the condition.
- `RCLCPP_[SEVERITY]_FUNCTION` and `RCLCPP_[SEVERITY]_STREAM_FUNCTION` – log message only when a function returns false (useful with  “isOk()” type of function).
- `RCLCPP_[SEVERITY]_THROTTLE` and `RCLCPP_[SEVERITY]_STREAM_THROTTLE` – log messages, but only as often as indicated. Useful for logging in high frequency loops and callbacks, when we care about how the value changes with time, but only need a much less frequent update.

## Logging and automated testing {#how-to-handle-errors-automated-testing}

Use launch testing as described in [this article](https://github.com/ros2/launch/tree/master/launch_testing#launch_testing) to check whether the node outputs logs as it is expected. You can even extract values from logs and check them against constraints.

A presence of `FATAL` or `ERROR` logs should fail a normal test. Presence of `FATAL` or `ERROR` logs that are different than expected should fail error handling tests. Tech leads can determine whether a presence of `WARN` logs should also fail the test.
