#!/bin/bash

# Copyright 2023 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

source `ros2 pkg prefix v2i_tester`/lib/v2i_tester/generate_json_body.bash

if [[ ! -v VTL_COMMAND_MESSAGE ]];then
  VTL_COMMAND_MESSAGE={}
fi

# create infrastructure command array json body
command_array=`echo ${VTL_COMMAND_MESSAGE} | jq -s`
declare -A settings_list=()
settings_list["commands"]="command_array"
INFRA_PUB_BODY=`getJsonFromSettingsList ${settings_list}`
unset settings_list

# create state_machine msg json body
declare -A settings_list=()
settings_list["service_layer_state"]="SRV_STATE"
settings_list["control_layer_state"]="CTL_STATE"
STATE_PUB_BODY=`getJsonFromSettingsList ${settings_list}`
unset settings_list

# publish messages
ros2 topic pub -1 /autoware_state_machine/state autoware_state_machine_msgs/msg/StateMachine "${STATE_PUB_BODY}"
ros2 topic pub /awapi/tmp/infrastructure_commands tier4_v2x_msgs/msg/InfrastructureCommandArray "${INFRA_PUB_BODY}"
