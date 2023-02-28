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

function getJsonFromSettingsList ()
{
  settings_list=$1
  body=`echo {}`
  for settings_key in "${!settings_list[@]}";do
    settings_value=${settings_list[${settings_key}]}
    if [[ -v ${settings_value} ]]; then
      echo ${body} | jq ".${settings_key} = \"${!settings_value}\"" &>/dev/null
      if [ $? -eq 0 ];then
        body=`echo ${body} | jq ".${settings_key} = \"${!settings_value}\""`
      else
        body=`echo ${body} | jq ".${settings_key} = ${!settings_value}"`
      fi
    fi
  done
  echo ${body}
}

function getJsonFromKeyValueArraySettingsList ()
{
  array_settings_list=$1
  body=""
  for k_value in "${!array_settings_list[@]}";do
    v_value=${array_settings_list[${k_value}]}
    if [[ ! -v ${v_value} ]];then
      continue
    fi
    declare -A settings_list=()
    array_settings_key=${k_value}
    settings_list["key"]="array_settings_key"
    settings_list["value"]=${v_value}
    key_value_json_body=`getJsonFromSettingsList ${settings_list}`
    body=`echo ${body} ${key_value_json_body}`
  done
  body=`echo ${body} | jq -s`
  echo ${body}
}
