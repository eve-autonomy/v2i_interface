# Vehicle to infrastructure (V2I) interface

## Overview
This node converts V2I communication between the Autoware of ROS2 interface and UDP which is outside of ROS2 interface.

This converter acts with a single external device on a vehicle.

It is necessary to prepare a user-defined broadcasting device, which connects to number of infrastructure devices.

## Input and Output
- input
  - from [autoware.universe](https://github.com/autowarefoundation/autoware.universe/)
    - `/awapi/tmp/infrastructure_commands` \[[tier4_v2x_msgs/msg/InfrastructureCommandArray](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_v2x_msgs/msg/InfrastructureCommandArray.msg)\]:<br>Control command to V2I infrastructure. It has an array structure to control multiple infrastructures at the same time.
  - from [autoware_state_machine](https://github.com/eve-autonomy/autoware_state_machine/)
    - `/autoware_state_machine/state` \[[autoware_state_machine_msgs/msg/StateMachine](https://github.com/eve-autonomy/autoware_state_machine_msgs/blob/main/msg/StateMachine.msg)\]:<br>State of the system.
  - from user-defined broadcasting device
    - `v2i status` ([UDP](#v2i-status)) :<br>State from V2I infrastructure. It has an array structure to control the vehicle based on the state of multiple infrastructures.
- output
  - to [autoware.universe](https://github.com/autowarefoundation/autoware.universe/)
    - `/system/v2x/virtual_traffic_light_status` \[[tier4_v2x_msgs/msg/VirtualTrafficLightStateArray](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_v2x_msgs/msg/VirtualTrafficLightStateArray.msg)\]:<br>ROS2 interface from `v2i_status` (UDP).
  - to user-defined broadcasting device
    - `v2i command` ([UDP](#v2i-command)) :<br>UDP protocol from `/awapi/tmp/infrastructure_commands`.

## Node Graph
![node graph](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/eve-autonomy/v2i_interface/main/docs/node_graph.pu)

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

## UDP protocol
Broadcasting device must meet the following specifications.

### V2I command: infrastructure_commands

```
{
  "seq_num" : uint32,
  "time": {
    "sec": uint64,
    "nanosec": unit32,
  },
  "request_array": [
    {
      "id": uint8,
      "request": uint8
    },
    ...
  ]
}
```

<details><summary>Item description</summary><div>

#### High level description

|Name       |Description|
|:----------|:----------|
|seq_num    |Increment from 0 for each output.|
|time       |UNIX time at the time of output.|
|request_array|Control commands for multiple V2I controllers.|

#### Details of items in request_array

|Name       |Description|
|:----------|:----------|
|id         |ID of the V2I infrastructure.|
|request    |Control command for the V2I infrastructure such as "open / close" and "turn on / off". The lower 4 bits and the upper 4 bits correspond to the states of 4 outputs and 4 inputs, respectively.|

</div></details>

### V2I status: virtual_traffic_light_states

```
{
  "seq_num" : uint32,
  "time": {
    "sec": uint64,
    "nanosec": unit32,
  },
  "id": uint8,
  "status": uint8,
  "detail": uint32.
  "reply_array": [
    {
      "id": uint8,
      "time": {
        "sec": uint64,
        "nanosec": unit32,
      },
      "status" : uint8,
      "packet_time" : {
        "sec" : uint64,
        "msec" : uint16,
      },
      "gpio" : uint8,
      "detail" : uint32,
      "vehicle": {
        "id" : uint8,
        "request" : uint8,
        "delay" : uint16,
        "rssi" : int8,
      },
      "rssi" : int8
    },
    ...
  ]
}
```

<details><summary>Item description</summary><div>

#### High level description
This is mainly about the status of broadcasting device.

|Name       |Description|
|:----------|:----------|
|seq_num    |Increment from 0 for each output.|
|time       |UNIX time at the time of output.|
|id         |ID of the broadcasting device.|
|status     |Error status; 0: Normal, 1: Near the end of life, 2: Error|
|detail     |Error code for details.|
|reply_array|The status of all connected V2I controller.|

#### Details of items in reply_array
This is about the status of each V2I controller.

|Name       |Description|
|:----------|:----------|
|id         |ID of the V2I infrastructure.|
|time       |UNIX time at the time of output.|
|status     |Error status; 0: Normal, 1: Near the end of life, 2: Error|
|packet_time|Unix time when the status of the V2I infrastructure was detected.|
|gpio       |The operating status of the V2I infrastructure such as "open / close" and "turn on / off". The lower 4 bits and the upper 4 bits correspond to the states of 4 outputs and 4 inputs, respectively.|
|detail     |Error code for details.|
|veihcle    |Sender status of the most recently sent V2I infrastructure control command.|
|rssi       |Received signal strength indicator from V2I controller to vehicle.|

#### Details of items in vehicle
This is the sender status of the most recently sent V2I infrastructure control command.

|Name   |Description|
|:------|:----------|
|id     |ID of the broadcasting device.|
|request|The copy of the control command.|
|delay  |Response time to control (msec).|
|rssi   |Received signal strength indicator from vehicle to V2I controller.|

</div></details>

## Vector map configuration
Add every optional tags below to virtual traffic light object.

| Name | Range | Description |
|--|--|--|
| type | eva_beacon_system | Fixed value（To identify from other VirtualTrafficLight objects.） |
| eva_beacon_system:id | 1-254 | ID which set to the equipment side beacon device. |  |
| eva_beacon_system:ref:section | REQUESTING | Selects a section that enables V2I control.<br>- REQUESTING: start_line to ref_line<br>- (Empty): start_line to end_line |
| eva_beacon_system:ref:permit_state | DRIVING | Selects vehicle states which enables V2I control.<br>- DRIVING: During driving<br>- (Empty): Do not care the vehicle states<br>* This can't specify the behivior before a route supplied. |
| eva_beacon_system:ref:request_bit | 0x0-0x0f | Value to be output by GPIO when the V2I control enabled. |
| eva_beacon_system:ref:expect_bit | 0x0-0x0f | Expected value which use with stop control specified by response_type. |
| eva_beacon_system:ref:response_type | ALWAYS<br>AND<br>MATCH | Specifies how the beacon system allow the vehicle to pass. <br>- ALWAYS: Always allows without calculation of value_bit and expect_bit.<br>- AND: Allows when `expect_bit & value_bit ≠0`<br>- MATCH: Allows when `expect_bit = value_bit` |
| eva_beacon_system:ref:mode | FIXED_VALUE<br>TURN_DIRECTION | - FIXED_VALUE<br>Use request_bit and expect_bit as specified.<br>- TURN_DIRECTION<br>Calculate request_bit and expect_bit based on turn_direction value of VirtualTrafficLight lanelet object.<br>※bit0: Straight、bit1: Turn right、bit2: Turn left |
