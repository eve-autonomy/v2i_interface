#!/bin/bash

# Copyright 2023 Tier IV, Inc.
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

# USER SETTING PARAMETER ############################
# export VTL_TYPE="eva_beacon_system"
# export SRV_STATE="uint16 (0 ~ 600)"
# export CTL_STATE="uint8 (0 or 1)"
# export VTL_ID="uint16"
# export VTL_STATE="uint8 (0~4)"
# export INFRA_ID="uint8 (1 ~ 254)"
# export INFRA_MODE="FIXED_VALUE or TURN_DIRECTION"
# export INFRA_RESPONSE_TYPE="ALWAYS or AND or MATCH"
# export INFRA_SECTION="REQUESTING or {NO DEFINITION}"
# export INFRA_PERMIT_STATE="DRIVING or {NO DEFINITION}"
# export INFRA_REQUEST_BIT="uint8 (0x00 ~ 0x0f)"
# export INFRA_EXPECT_BIT="uint8 (0x00 ~ 0x0f)"
# export LANELET_TURN_DIRECTION="straight or left or right or {NO DEFINITION}"

source `ros2 pkg prefix v2i_tester`/lib/v2i_tester/generate_json_body.bash

# main process ########################################

# create keyvalue msg json body
declare -A array_settings_list=()
array_settings_list["type"]="VTL_TYPE"
array_settings_list["eva_beacon_system:id"]="INFRA_ID"
array_settings_list["eva_beacon_system:ref:mode"]="INFRA_MODE"
array_settings_list["eva_beacon_system:ref:response_type"]="INFRA_RESPONSE_TYPE"
array_settings_list["eva_beacon_system:ref:section"]="INFRA_SECTION"
array_settings_list["eva_beacon_system:ref:permit_state"]="INFRA_PERMIT_STATE"
array_settings_list["eva_beacon_system:ref:request_bit"]="INFRA_REQUEST_BIT"
array_settings_list["eva_beacon_system:ref:expect_bit"]="INFRA_EXPECT_BIT"
array_settings_list["turn_direction"]="LANELET_TURN_DIRECTION"
array_settings_list["subtype"]="LANELET_SUBTYPE"
array_settings_list["lane_id"]="LANELET_LANE_ID"
custom_tags=`getJsonFromKeyValueArraySettingsList ${array_settings_list}`
unset array_settings_list

# create infrastructure command json body
declare -A settings_list=()
settings_list["type"]="VTL_TYPE"
settings_list["id"]="VTL_ID"
settings_list["state"]="VTL_STATE"
settings_list["custom_tags"]="custom_tags"
command=`getJsonFromSettingsList ${settings_list}`
export VTL_COMMAND_MESSAGE="${VTL_COMMAND_MESSAGE} ${command}"
unset settings_list
