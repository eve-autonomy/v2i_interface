# vtl_adapter

## Orverview

Change Autoware's ros2 topic to make it common for v2i_interface modules.
In the v2i_interface, update the received common ros2 topic for VTL.

## Input and Output

- input
  - from [autoware.universe](https://github.com/autowarefoundation/autoware.universe/)
    - `/api/external/get/virtual_traffic_light/commands` \[[tier4_v2x_msgs/msg/InfrastructureCommandArray](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_v2x_msgs/msg/InfrastructureCommandArray.msg)\]:<br>Control command to V2I infrastructure. It has an array structure to control multiple infrastructures at the same time.
- output
  - to [autoware.universe](https://github.com/autowarefoundation/autoware.universe/)
    - `/system/v2x/virtual_traffic_light_status` \[[tier4_v2x_msgs/msg/VirtualTrafficLightStateArray](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_v2x_msgs/msg/VirtualTrafficLightStateArray.msg)\]:<br>ROS2 interface from `v2i_status` (UDP).
