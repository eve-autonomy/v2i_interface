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

| 設定値 | 値の範囲 | 説明 |
|--|--|--|
| type | eva_beacon_system | 固定値（他のVirtualTrafficLightオブジェクトとの区別用） |
| parent_way_id | （対象のway_id値） | 複数のVirtualTrafficLightを組み合わせる場合の親子関係の指定<br>親のway_idを設定する<br>親のVirtualTrafficLightが許可されるまで、子のVirtualTrafficLightは処理しない（未許可のまま維持）<br>（現状未対応　→10月以降対応予定） |
| eva_beacon_system:ref:id | 1~254 | アクセスする設備の機器番号（設備側netPIのコンフィグで設定する値） |  |
| eva_beacon_system:ref:supplement | （自由記述欄） | コメント記載用（処理判断には用いない） |
| eva_beacon_system:ref:section | REQUESTING | start_line～ref_line～end_lineの間で、ON制御を出す区間を選択（複数選択可能）<br>・REQUESTING：start_line～ref_line<br>・指定なし：start_line～end_line |
| eva_beacon_system:ref:permit_state | DRIVING | ON制御を出す車両状態を選択（複数選択可能）<br>・DRIVING：走行中<br>・指定なし：車両状態にかかわらない<br>※注意：ルート配信前は、インフラ自体を検出しないため指定不可能 |
| eva_beacon_system:ref:request_bit | 0x0~0x0f | ON制御によりHigh信号を出力するGPIO番号0~3のbit組み合わせ |
| eva_beacon_system:ref:expect_bit | 0x0~0x0f | 一時停止判断を実施する際の期待値 |
| eva_beacon_system:ref:response_type | ALWAYS<br>AND<br>MATCH | 一時停止判断で、許可と判断する条件<br>許可の場合は、Autoware側に対し一時停止の解除を指示<br>※High信号が入力されているGPIO番号0~3のbit組み合わせをvalue_bitとした場合<br>・ALWAYS： value_bit、expect_bitにかかわらず常に許可<br>・AND：`expect _bit & value_bit ≠0`で許可<br>・MATCH：`expect _bit = value_bit`で許可 |
| eva_beacon_system:ref:mode | FIXED_VALUE<br>TURN_DIRECTION | ・FIXED_VALUE<br>下記のrequest_bitとexpect_bitをそのまま使用する<br>・TURN_DIRECTION<br>VirtualTraficLightオブジェクトが紐づく経路のturn_direction値に基いて、request_bitとexpect_bitを算出して、ON制御と一時停止判断を行う<br>※bit0:直進、bit1:右折、bit2：左折 |
