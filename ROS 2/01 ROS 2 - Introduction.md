

# ROS 2 Crash Course

Author: methylDragon  
Fairly comprehensive ROS 2 crash course!  
I'll be adapting it from the ROS 2 Tutorials: <https://index.ros.org/doc/ros2/Tutorials/>    
and [some other places](<http://docs.erlerobotics.com/robot_operating_system/ros2>)

------

## Pre-Requisites

- A system with Ubuntu 18.04+ installed
- Linux knowledge
- Python 3 and/or C++
- Prior ROS 1 experience
  - Basic concepts will not be explained so well
  - Knowledge from the [ROS1 tutorial](../ROS) will be presumed (like setting up Ubuntu, etc.)
- Prior DDS experience will help as well



## Table Of Contents



## Introduction

ROS 2 was built to overcome some of the limitations of ROS 1, and as a means to refresh the interfaces and technologies used. (Eg. ROS 2 targets C++11 onwards and Python 3.5 while ROS 1 is usually stuck with Python 2 and C++4)

**It is NOT built on ROS1. It's a completely separate framework altogether!** Although, it can interface with ROS 1. Sometimes people like to make multi-robot fleets, and control individual robots using ROS 1, using ROS 2 to control the overall fleet instead.

It features several key improvements as a result of switching its messaging layer to a DDS implementation as middleware (eProsima's FastRTPS by default, though CycloneDDS might become the standard.)

Paired with the renewed development efforts aimed at properly targeting new use cases, these benefits include:

- The system is truly distributed, supporting peer to peer **discovery**
- Quality of service protocols can be implemented to help with enforcing real time system architectures as well as dealing with packet losses
- It still works with ROS 1! (If you have a bridge.)
- Multi-platform and OS support

> Much has happened since and many situations, initially not covered by the framework, have demanded a solution. ROS 2 address some of these limitations putting specific effort into the following robotic problems:
>
> - **Teams of multiple robots**
> - **Small embedded and deep embedded platforms either microcontroller or microprocessor-based.**
> - **Real-time systems**
> - **Non-ideal networks**
> - **Production environments**
> - **Prescribed patterns for building and structuring systems**
>
> The Robot Operating System (ROS) 2 will define the next decade of robotics so read up!
>
> <http://design.ros2.org/articles/why_ros2.html>

You can read more about changes between ROS 1 and ROS 2 [here](<http://design.ros2.org/articles/changes.html>).

Additionally, since ROS 2 is using a middleware layer, it is fairly simple to switch DDS implementations if the need arises. But that particular functionality will be treated as outside the scope of this tutorial.



## Installation

### Installing ROS 2

This particular tutorial is configured to install ROS 2 Dashing for Ubuntu 18.04.

But if you want a different distribution, or want to install on Windows or Mac, check out [the docs](<https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/>).

**Add Keys**

```shell
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
```

**Install ROS 2 Dashing**

```shell
sudo apt update

# For computer
sudo apt install ros-dashing-desktop

# For robot base
sudo apt install ros-dashing-ros-base
```

**Install Helpers**

```shell
sudo apt install python3-argcomplete
sudo apt install python3-colcon-common-extensions
```

**Setup Sourcing**

> **Note**: If you want to run this alongside ROS1, be sure to selectively source the setup.bashes. They normally can't be sourced together!

```shell
echo "source /opt/ros/dashing/setup.bash" >> ~/.bashrc
```

**Install ROS 1 Bridge**

```shell
sudo apt update
sudo apt install ros-dashing-ros1-bridge
```



### Optional: Installing Additional Middleware Implementations

These are **optional**! They're also not really fully supported.

```shell
sudo apt update

# OpenSplice
sudo apt install ros-dashing-rmw-opensplice-cpp

# RTI Connext (requires license agreement)
sudo apt install ros-dashing-rmw-connext-cpp
```

To use them, ensure the following environment variables are set:

```shell
# For Opensplice
export RMW_IMPLEMENTATION=rmw_opensplice_cpp

# For RTI Connect
export RMW_IMPLEMENTATION=rmw_connext_cpp
```



## colcon

Before we can really get into the nitty gritty of how ROS 2 works, and all the wonderful coding examples that are floating out there. It's important to first learn about how to even build a ROS 2 package in the first place.



### Introduction

[colcon](<https://colcon.readthedocs.io/en/released/user/quick-start.html>) is the [universal build tool](<http://design.ros2.org/articles/build_tool.html>) that iterates on all past build tools (`catkin`, `catkin_make`, and `ament_tools`) that is meant to allow you to build ROS 1 and ROS 2 packages.

> The goal of a unified build tool is to build a set of packages with a single invocation. It should work with ROS 1 packages as well as ROS 2 packages which provide the necessary information in their manifest files. It should also work with packages that do not provide manifest files themselves, given that the necessary meta information can be inferred and/or is provided externally. This will allow the build tool to be utilized for non-ROS packages (e.g. Gazebo including its ignition dependencies, sdformat, etc.).
>
> <http://design.ros2.org/articles/build_tool.html>



### Directories

Like ROS 1 with `catkin`, ROS 2 packages are built within workspaces. These workspaces include several directories.

| Directory | Description                                                 |
| --------- | ----------------------------------------------------------- |
| src       | Package source files                                        |
| build     | Intermediate build files                                    |
| install   | Package installation directory. Each built binary goes here |
| log       | Log information about the build process                     |

> Notably, there is no `devel` directory, unlike in ROS 1's `catkin`.



### Command Line

You can invoke `colcon` using the command line. Invoke them in the workspace directory!

```shell
$ colcon list -g            # List all packages in the workspace and their dependencies
$ colcon build              # Build all packages in the workspace
$ colcon test               # Test all packages in the workspace
$ colcon test-result --all  # Enumerate all test results

$ colcon build --packages-select <name-of-pkg> # Individual build
$ colcon build --packages-up-to <name-of-pkg>  # Selective build
```



### Creating an Example Workspace

Ok! So let's just go ahead and create our example workspace that we'll use to go through the tutorial with. We'll be using the ros2 example repository.

```shell
# Create workspace
mkdir -p ~/ros2_example_ws/src
cd ~/ros2_example_ws

# Clone examples and checkout the relevant branch
git clone https://github.com/ros2/examples src/examples
cd ~/ros2_example_ws/src/examples/
git checkout $ROS_DISTRO
cd ~/ros2_example_ws

# Source the underlay (the underlying ROS 2 installation)
# In this case we're using Dashing, but if you're using something else,
# Go ahead and change it
source /opt/ros/dashing/setup.bash

# Build!
colcon build --symlink-install

# Source the built environment
. install/setup.bash
```

> **Note**: The symlink-install flag creates a symlink to the install space that allows you to change **uncompiled** source files from the source space for faster iteration speed. If you're building compiled sources though, you will still need to rebuild.




```
                            .     .
                         .  |\-^-/|  .    
                        /| } O.=.O { |\
```

​    

------

[![Yeah! Buy the DRAGON a COFFEE!](../assets/COFFEE%20BUTTON%20%E3%83%BE(%C2%B0%E2%88%87%C2%B0%5E).png)](https://www.buymeacoffee.com/methylDragon)