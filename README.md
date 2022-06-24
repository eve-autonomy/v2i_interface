# Vehicle to infrastructure (V2I) interface

## Overview
This node converts V2I communication between the Autoware of ROS2 interface and UDP which is outside of ROS2 interface.

This converter acts with a single external device on a vehicle.

It is necessary to prepare a user-defined broadcasting device, which connects to number of infrastructure devices.

## Input and Output
- input
  - from [autoware.universe](https://github.com/autowarefoundation/autoware.universe/)
    - `/awapi/tmp/infrastructure_commands` : Control command to V2I infrastructure. It has an array structure to control multiple infrastructures at the same time.
  - from [user-defined broadcasting device](#v2i-status)
    - `v2i status` (UDP) : State from V2I infrastructure. It has an array structure to control the vehicle based on the state of multiple infrastructures.
- output
  - to [autoware.universe](https://github.com/autowarefoundation/autoware.universe/)
    - `/system/v2x/virtual_traffic_light_status` : ROS2 interface from `v2i_status` (UDP).
  - to [user-defined broadcasting device](#v2i-command)
    - `v2i command` (UDP) : UDP protocol from `/awapi/tmp/infrastructure_commands`.

## Node Graph
![node graph](http://www.plantuml.com/plantuml/proxy?src=https://raw.githubusercontent.com/eve-autonomy/v2i_interface/docs/node_graph.pu)

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


If you want to use different value, fork the [v2i_interface_params.default](https://github.com/eve-autonomy/v2i_interface_params.default) repository, create a new repository.

## UDP protocol
The broadcasting device must meet the following communication specifications.

### v2i command

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

#### Top level items of v2i command

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

### v2i status

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

#### Top level items of v2i status
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

