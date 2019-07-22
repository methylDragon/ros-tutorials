

# ROS2 Crash Course

Author: methylDragon  
Fairly comprehensive ROS2 crash course!  
I'll be adapting it from the ROS2 Tutorials: <https://index.ros.org/doc/ros2/Tutorials/>    
and [some other places](<http://docs.erlerobotics.com/robot_operating_system/ros2>)

------

## Pre-Requisites

- A system with Ubuntu 18.04+ installed
- Linux knowledge
- Python 3 and/or C++
- Prior ROS1 experience
  - Basic concepts will not be explained so well
  - Knowledge from the [ROS1 tutorial](../ROS) will be presumed (like setting up Ubuntu, etc.)
- Prior DDS experience will help as well



## Table Of Contents



## Introduction

ROS2 was built to overcome some of the limitations of ROS1, and as a means to refresh the interfaces and technologies used. (Eg. ROS2 targets C++11 onwards and Python 3.5 while ROS1 is usually stuck with Python2 and C++4)

**It is NOT built on ROS1. It's a completely separate framework altogether!** Although, it can interface with ROS1. Sometimes people like to make multi-robot fleets, and control individual robots using ROS1, using ROS2 to control the fleet instead.

It features several key improvements as a result of switching its messaging layer to a DDS implementation as middleware (eProsima's FastRTPS by default, though CycloneDDS might become the standard.)

Paired with the renewed development efforts aimed at properly targeting new use cases, these benefits include:

- The system is truly distributed, supporting peer to peer **discovery**
- Quality of service protocols can be implemented to help with enforcing real time system architectures as well as dealing with packet losses
- It still works with ROS1! (If you have a bridge.)
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

You can read more about changes between ROS1 and ROS2 [here](<http://design.ros2.org/articles/changes.html>).

Additionally, since ROS2 is using a middleware layer, it is fairly simple to switch DDS implementations if the need arises. But that particular functionality will be treated as outside the scope of this tutorial.



## Setup and Basic Concepts

### Installing ROS2

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

> **Note**: If you want to run this alongside ROS1, be sure to selectively source the setup.bashes. They normally can't be sourced together.

```shell
echo "source /opt/ros/dashing/setup.bash" >> ~/.bashrc
```

**Install ROS 1 Bridge**

```shell
sudo apt update
sudo apt install ros-dashing-ros1-bridge
```




```
                            .     .
                         .  |\-^-/|  .    
                        /| } O.=.O { |\
```

​    

------

[![Yeah! Buy the DRAGON a COFFEE!](../../_assets/COFFEE%20BUTTON%20%E3%83%BE(%C2%B0%E2%88%87%C2%B0%5E).png)](https://www.buymeacoffee.com/methylDragon)