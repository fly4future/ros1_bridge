# Bridge between ROS Noetic and ROS 2 Humble with MRS Custom Message Support

This fork of `ros2/ros1_bridge` integrates support for **MRS-specific custom messages and services**, and includes essential patches to ensure full compatibility between ROS 1 (Noetic) and ROS 2 (Humble).

## Modifications from Original `ros2/ros1_bridge`

This fork currently includes the following modification:

### Added Conversion for `std_msgs/Time` ↔ `builtin_interfaces/msg/Time`

The original bridge did not provide specializations for bridging time messages, resulting in linker errors like:

```
undefined reference to convert_1_to_2<Time>
```

To fix this, the following template specializations were added in  
`src/ros1_bridge/convert_builtin_interfaces.cpp` and  
`include/ros1_bridge/convert_builtin_interfaces.hpp`:

```cpp
template<>
void convert_1_to_2(
  const std_msgs::Time & ros1_msg, 
  builtin_interfaces::msg::Time & ros2_msg);

template<>
void convert_2_to_1(
  const builtin_interfaces::msg::Time & ros2_msg, 
  std_msgs::Time & ros1_msg);
```

These functions enable proper bidirectional conversion of ROS 1 and ROS 2 time messages, resolving linker issues and allowing seamless communication.

---

## How to build and run the bridge

### Step 1: Prerequisites

This bridge is made to work on Ubuntu 20 for ros noetic and ros2 humble. So first make sure that your local environment runs on Ubuntu 20.

Then, you can install ros noetic by following [the official tutorial](https://wiki.ros.org/noetic/Installation/Ubuntu).

Finally, you will need to install ros2 humble from source following [these instructions](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html). Make sure to select the command for Ubuntu 20.04 LTS in the section **Install development tools and ROS tools**. 


Now that you have a functionning Ubuntu 20 environment that supports both ros noetic and ros2 humble, it is time to start actually building the mrs_msgs bridge.

Run the following command to create the working space we will use in the next steps:

```sh
mkdir -p ~/mrs_bridge
cd ~/mrs_bridge
```

### Step 2: Build the ros noetic workspace

First, we need to build a ros noetic package that contains the mrs_msgs library.

To do so, run the following command:

```sh
cd ~/mrs_bridge
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone -b bridge-noetic git@github.com:ctu-mrs/mrs_msgs.git
cd ..
source /opt/ros/noetic/setup.bash
catkin_make_isolated --install
```

Expect this build to take 1 to 2 minutes. 

### Step 3: Build the ros2 humble workspace

Now, we must build a ros2 humble package that integrates the custom mrs messages and services.

Run this command:

```sh
cd ~/mrs_bridge
mkdir -p colcon_ws/src
cd colcon_ws/src
git clone -b bridge-humble git@github.com:ctu-mrs/mrs_msgs.git
cd ..
source ~/ros2_humble/install/setup.bash
colcon build
```

This build should take 1 to 2 minutes. 

### Step 4: Build ros1_bridge

Now we are finally ready to build ros1_bridge!

Run the following command:

```sh
cd ~/mrs_bridge
mkdir -p bridge_ws/src
cd bridge_ws/src
git clone git@github.com:fly4future/ros1_bridge.git
cd ..
source /opt/ros/noetic/setup.bash
source ../catkin_ws/install_isolated/setup.bash
source ~/ros2_humble/instal/setup.bash
source ../colcon_ws/install/setup.bash
colcon build --packages-select ros1_bridge --cmake-force-configure
```

IMPORTANT: This build is extremely heavy and will take time to proceed. As a reference, it took 16 minutes on my computer. 

To visualize all the bridged messages accross ros noetic and ros2 humble, run:

```sh
source install/setup.bash
ros2 run ros1_bridge dynamic_bridge --print-pairs
```

It should display all the mrs custom messages and services bridged accross ros1 and ros2.

### Step 5: Run the bridge

To run the dynamic bridge, you'll need **three terminals**: one for ROS 1, one for ROS 2, and one for the bridge itself.

#### Terminal 1: Launch ROS 1 Nodes

```sh
cd ~/mrs_bridge/catkin_ws
source /opt/ros/noetic/setup.bash
source devel_isolated/setup.bash
roslaunch your_ros1_package your_node.launch
```

#### Terminal 2: Launch ROS 2 Nodes

```sh
cd ~/mrs_bridge/colcon_ws
source ~/ros2_humble/install/setup.bash
source install/setup.bash
ros2 launch your_ros2_package your_node.launch.py
```

#### Terminal 3: Run the Bridge

```sh
cd ~/mrs_bridge/bridge_ws
source /opt/ros/noetic/setup.bash
source ../catkin_ws/install_isolated/setup.bash
source ~/ros2_humble/instal/setup.bash
source ../colcon_ws/install/setup.bash
source install/setup.bash
ros2 run ros1_bridge dynamic_bridge
```
