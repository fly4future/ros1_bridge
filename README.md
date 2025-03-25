# Bridge between ROS Noetic and ROS 2 Jazzy with MRS Custom Message Support

This fork of `ros2/ros1_bridge` integrates support for **MRS-specific custom messages and services**, and includes essential patches to ensure full compatibility between ROS 1 (Noetic) and ROS 2 (Jazzy).

## 🛠️ Modifications from Original `ros2/ros1_bridge`

This fork currently includes the following modification:

### ✅ Added Conversion for `std_msgs/Time` ↔ `builtin_interfaces/msg/Time`

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

## How to build and run the bridge (to be added)