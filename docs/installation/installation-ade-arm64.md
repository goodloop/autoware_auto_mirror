Installation with ADE for arm64 Systems {#installation-ade-arm64}
=======================================

@tableofcontents

# Goals {#installation-and-development-goals-arm64}

This article demonstrates how to launch AutowareAuto using ADE for `arm64` systems and those wishing to develop using `arm64`. This document will cover both native and non-native systems using ADE.

# Native arm64 System {#native-arm64-installation}

//TODO:

# Non-native arm64 System {#non-native-arm64-installation}

The following section describes the process to run multi-architecture systems using Docker,
`binfmt`, and `qemu`.

## Prerequisites

Before alternative architectures can be run on a system, ensure that one can run ADE and
AutowareAuto on the native architecture.
Those with `amd64` systems should follow the instructions in @ref installation-ade and ensure
all dependencies are properly installed.
The following will assume that all ADE and AutowareAuto dependencies have been installed.

@note The emulation library used for this section is currently only compatible with `x86_64`.
Check your system architecture using the following command:
```
$ uname -m
```

To check the systems which Docker is compatible with run the following command:
```
$ docker buildx ls
```

The output the following should look like this:
```
$ docker buildx ls
NAME/NODE DRIVER/ENDPOINT STATUS  PLATFORMS
default * docker
  default default         running linux/amd64, linux/386  
```

To check that your system is currently incompatible with `arm64` systems is by running:
```
$ docker run --rm -t arm64v8/ubuntu uname -m
```
The output of this should error and indicate that libraries were not found.

## Configuring Docker for Multi-architecture Emulation

First, install emulation and binary support libraries that will allow Docker to run multiple
architectures.
The libraries [`qemu`](https://www.qemu.org/) and [`qemu-user-static`](https://github.com/multiarch/qemu-user-static)
provide emulation support allowing Docker to interpret alternative architectures on an `x86_64` environment.
The kernel module [`binfmt-support`](http://binfmt-support.nongnu.org/) allows for the registry
and invocation of binary interpreters at the system administrator level. 
```
$ sudo apt-get install qemu binfmt-support qemu-user-static
```

Finally, invoke the `qemu-user-static` docker image to install and link the interpreters and
architectures for various architectures.
```
$ docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

To check that the installation and registry was successful, run the following command and ensure
that it exits cleanly:
```
$ docker run --rm -t arm64v8/ubuntu uname -m
...
aarch64
```

Additional checks include running the `buildx` option with Docker.
This should output a larger variety of build types available to Docker.

@note There will be an initial warning that the architecture of the image Docker is trying to bring up is different to the architecture of the system.
```
WARNING: The requested image's platform (linux/arm64) does not match the detected host platform (linux/amd64) and no specific platform was requested
```

## Launching ADE

Now that the set-up is complete, the `arm64` ADE image can be launched with no issues
```
ade --rc .aderc-arm64 start --update --enter
```
