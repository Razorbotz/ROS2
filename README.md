# Welcome to the Razorbotz NASA Robotic Mining Competition Project!
This page intends to provide a starting point and overview of the project, as well as a roadmap for how to get involved with the project even if you aren't familiar with the code or technology stack. Please note that these links may not be up to date and any links should be followed at your own risk.  If you find any links that no longer work or changes that need to be made, please contact me at andrewburroughs17@gmail.com.  Click [here](https://razorbotz.github.io/ROS2/) to view the documentation for the project.  If you are not familiar with Github and the git cli, please refer to the [Razorbotz Github Intro page](https://github.com/Razorbotz/Test).

## Overview
* [Prerequisites](https://github.com/Razorbotz/ROS2/tree/master#prerequisites---windows)
* [Installing ROS2](https://github.com/Razorbotz/ROS2/tree/master#installing-ros2)
* [Understanding the Codebase](https://github.com/Razorbotz/ROS2/tree/master#understanding-the-codebase)
* [Documentation](https://github.com/Razorbotz/ROS2/tree/master#documentation)
* [Tutorials](https://github.com/Razorbotz/ROS2/tree/master#tutorials)

## Prerequisites - Windows
To begin the project, install VirtualBox or another virtualization software to your computer.  If you are using Linux, disregard the VirtualBox installation.

### Installing VirtualBox
To install VirtualBox, first download the software [here](https://www.virtualbox.org/wiki/Downloads).  After downloading and installing VirtualBox, download an image of [Ubuntu 20.04](http://releases.ubuntu.com/20.04/) and install Ubuntu.  A tutorial on how to do so is located [here](https://linuxhint.com/install_ubuntu_virtualbox_2004/).  To allows the virtual machine to be able to be full screen, enter the following commands in the terminal:

```
sudo apt update

sudo apt install virtualbox-guest-dkms virtualbox-guest-x11 virtualbox-guest-utils
```

After executing the commands, click on the Devices drop down menu at the top of the virtual machine.  Click on Insert Guest Additions Image, then click Run and enter the password when prompted.  For any issues, please consult this [installation guide](https://linuxhint.com/install_ubuntu_virtualbox_2004/#:~:text=Installing%20VirtualBox%20Guest%20Additions%20on%20Ubuntu%2020.04%20LTS).

## Installing ROS2
The project uses the Foxy distribution of ROS2.  To build from source or install on other operating systems, refer to the [Installation Page](https://docs.ros.org/en/foxy/Installation.html).  **To install the Foxy distribution of ROS2 using Debian packages, run the following Linux terminal commands inside the virtual machine or on a native Linux machine.**

### Set locale
```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

### Setup Sources
```
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo sed -i -e 's/ubuntu .* main/ubuntu focal main/g' /etc/apt/sources.list.d/ros2.list
```

### Install ROS2 Packages
```
sudo apt update

sudo apt install ros-foxy-desktop
```

### Run Some Examples
**Run the following commands in one terminal**

```
source /opt/ros/foxy/setup.bash

ros2 run demo_nodes_cpp talker
```

**In a second terminal, run the following commands**

```
source /opt/ros/foxy/setup.bash

ros2 run demo_nodes_py listener
```

If you want to build from source or install on a different operating system, please refer to the [installation guide](https://docs.ros.org/en/foxy/Installation.html).

## Understanding the Codebase

### Structure of the packages
ROS2 packages all contain the following:
* src folder //contains the source code / node files
* CMakeLists.txt //auto-generated file necessary for C++ ROS2 packages
* package.xml //auto-generated file necessary for C++ ROS2 packages

The src folder within a package contains the .cpp files that define nodes and supporting files for classes/objects/functions relevant to that package.  To read more about ROS2 packages, please refer to the [ROS tutorial](https://docs.ros.org/en/foxy/Tutorials/Creating-Your-First-ROS2-Package.html).

The ROS2 packages currently in this project are as follows:
* [Autonomy](https://github.com/Razorbotz/ROS2/tree/master/skinny/src/autonomy)
* [Communication](https://github.com/Razorbotz/ROS2/tree/master/skinny/src/communication)
* [Excavation](https://github.com/Razorbotz/ROS2/tree/master/skinny/src/excavation)
* [Logic](https://github.com/Razorbotz/ROS2/tree/master/skinny/src/logic)
* [Power Distribution Panel](https://github.com/Razorbotz/ROS2/tree/master/skinny/src/poewr_distribution_panel)
* [Talon](https://github.com/Razorbotz/ROS2/tree/master/skinny/src/talon)
* [Zed Tracking](https://github.com/Razorbotz/ROS2/tree/master/skinny/src/zed_tracking)

## Documentation
This project uses [Doxygen](https://www.doxygen.nl/index.html) to generate documentation for the files automatically.  To learn more about the Doxygen formatting, please refer to the [Documenting the code](https://www.doxygen.nl/manual/docblocks.html) section of the Doxygen docs.  The documentation for this project can be found at the project website that is found [here](https://razorbotz.github.io/ROS2/).

### Documentation Template
To standardize the documentation across multiple authors, the following documentation template will be used throughout the project.  To see an example of how files should be commented to generate the documentation correctly, see [Example.cpp](https://github.com/Razorbotz/ROS2/blob/master/docs/Example.cpp).

**Files**
* Description of file
* Topics subscribed to
* Topics published
* Related files

**Functions**
* Description of Function
* Parameters
* Return values
* Related files and/or functions

## Tutorials

To gain a better understanding of ROS2, please refer to the following [tutorials](https://docs.ros.org/en/foxy/Tutorials.html).
* [Configuring Your ROS 2 Environment](https://docs.ros.org/en/foxy/Tutorials/Configuring-ROS2-Environment.html)
* [Understanding ROS 2 Nodes](https://docs.ros.org/en/foxy/Tutorials/Understanding-ROS2-Nodes.html)
* [Understanding ROS 2 Topics](https://docs.ros.org/en/foxy/Tutorials/Topics/Understanding-ROS2-Topics.html)
* [Understanding ROS 2 Parameters](https://docs.ros.org/en/foxy/Tutorials/Parameters/Understanding-ROS2-Parameters.html)
* [Creating a Launch File](https://docs.ros.org/en/foxy/Tutorials/Launch-Files/Creating-Launch-Files.html)
* [Creating a Workspace](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html)
* [Creating a Package](https://docs.ros.org/en/foxy/Tutorials/Creating-Your-First-ROS2-Package.html)
* [Writing a Simple Publisher and Subscriber (C++)](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
* [Writing a Simple Publisher and Subscriber (Python)](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
* [Writing Custom ROS2 msg Files](https://docs.ros.org/en/foxy/Tutorials/Custom-ROS2-Interfaces.html)
* [Using Parameters in a Class (C++)](https://docs.ros.org/en/foxy/Tutorials/Using-Parameters-In-A-Class-CPP.html)
* [Using Parameters in a Class (Python)](https://docs.ros.org/en/foxy/Tutorials/Using-Parameters-In-A-Class-Python.html)
* [Using ROS2 Launch](https://docs.ros.org/en/foxy/Tutorials/Launch-Files/Using-ROS2-Launch-For-Large-Projects.html)