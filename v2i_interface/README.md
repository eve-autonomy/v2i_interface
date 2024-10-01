# v2i_interface

## Overview
This node, Convert and send/receive data between ros2 topic and udp communication.

## Input and Output
- input
  - from [v2i_command_muxer](https://github.com/eve-autonomy/v2i_interface)
    - `/v2i/infrastructure_commands` \[[tier4_v2x_msgs/msg/InfrastructureCommandArray](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_v2x_msgs/msg/InfrastructureCommandArray.msg)\]:<br>Control command to V2I infrastructure. It has an array structure to control multiple infrastructures at the same time.
  - from user-defined broadcasting device
    - `v2i status` ([UDP](#v2i-status)) :<br>State from V2I infrastructure. It has an array structure to control the vehicle based on the state of multiple infrastructures.
- output
  - to [cargo_loading_service](https://github.com/eve-autonomy/cargo_loading_service)
    - `/v2i/infrastructer_states` \[[v2i_interface_msgs/msg/InfrastructureStateArray.msg](https://github.com/eve-autonomy/v2i_interface_msgs/blob/main/msg/InfrastructureState.msg)\]:<br>ROS2 interface from `v2i_status` (UDP).
  - to user-defined broadcasting device
    - `v2i command` ([UDP](#v2i-command)) :<br>UDP protocol from `/awapi/tmp/infrastructure_commands`.

## v2i interface ｍodule internal specifications
![internal specifications of node graph](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/eve-autonomy/v2i_interface/main/docs/node_graph_internal_spec.pu)

## Launch arguments

|Name          |Descriptoin|
|:-------------|:----------|
|operation_mode|Select the following operation modes; `product`, `local_test`. This value changes parameter directories.|

## Parameter description
These are mandatory parameters of UDP connection to a user-defined broadcasting device.
|Name          |
|:-------------|
|ip_address    |
|send_port     |
|receive_port  |

If you want to use different set of paremeters, fork the [v2i_interface_params.default](https://github.com/eve-autonomy/v2i_interface_params.default) repository.

## How to use test tools
### install
Requirements
```
pip3 install -r TkEasyGUI
```
### Run 
```
ros2 launch v2i_interface v2i_interface_test.launch.xml
```

## License
This project includes the following third-party software:

- TkEasyGUI Copyright (c) 2024 kujirahand  Licensed under the MIT License.
For more details, please see [https://github.com/kujirahand/tkeasygui-python/blob/main/LICENSE].
