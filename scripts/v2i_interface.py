#!/usr/bin/env python3
# coding: utf-8

# Copyright 2021 eve autonomy inc. All Rights Reserved.
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

import signal
import threading
import time

from autoware_state_machine_msgs.msg import StateMachine

from tier4_v2x_msgs.msg import (
    InfrastructureCommand, InfrastructureCommandArray,
    VirtualTrafficLightState, VirtualTrafficLightStateArray
)

from v2i_interface import udp_control

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile


class InfrastructureProperty:
    def __init__(self, state, sec, nanosec, keyvalue):
        self._state = state
        self._sec = sec
        self._nanosec = nanosec
        self._keyvalue = keyvalue
        self._approval = False
        self._reply_value = 0
        self._timeout_cnt = 0

    @property
    def state(self):
        return self._state

    @property
    def sec(self):
        return self._sec

    @property
    def nanosec(self):
        return self._nanosec

    @property
    def keyvalue(self):
        return self._keyvalue

    @property
    def approval(self):
        return self._approval

    @property
    def reply_value(self):
        return self._reply_value

    @property
    def timeout_cnt(self):
        return self._timeout_cnt

    @approval.setter
    def approval(self, value: bool):
        self._approval = value

    @reply_value.setter
    def reply_value(self, value: int):
        self._reply_value = value

    @timeout_cnt.setter
    def timeout_cnt(self, value: int):
        self._timeout_cnt = value


class V2iInterfaceNode(Node):
    def __init__(self):
        super().__init__('v2i_interface')
        self._logger = self.get_logger()

        self._infra_dict = {}
        self._current_service_layer_state = StateMachine.STATE_DURING_WAKEUP
        self._current_control_layer_state = StateMachine.MANUAL

        timer_period = 0.1
        self._udp_send_cnt = 0
        self._udp_send_cnt_max = 2
        self._udp_recv_interval = 0.1

        self._infra_command_timeout_cnt_max = 10

        ip_address = self.declare_parameter("ip_address", "127.0.0.1")
        send_port = self.declare_parameter("send_port", 0)
        receive_port = self.declare_parameter("receive_port", 0)
        self._ip_address = ip_address.get_parameter_value().string_value
        self._send_port = send_port.get_parameter_value().integer_value
        self._receive_port = receive_port.get_parameter_value().integer_value
        buffer_size = 4096
        receive_timeout = 0.5

        # QoS Setting
        depth = 1
        cmd_profile = QoSProfile(depth=depth)
        state_profile = QoSProfile(depth=depth)
        autoware_profile = QoSProfile(depth=depth)
        # Option for latching
        # Transient local: the publisher becomes responsible for persisting samples
        #  for "late-joining" subscriptions.
        # Volatile: no attempt is made to persist samples.
        state_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        # pub/sub
        self._command_array_subscriber = self.create_subscription(
            InfrastructureCommandArray, "~/input/command_array",
            self.on_command_array, cmd_profile)
        self._state_code_subscriber = self.create_subscription(
            StateMachine, "/autoware_state_machine/state",
            self.on_autoware_state, autoware_profile)
        self._state_array_publisher = self.create_publisher(
            VirtualTrafficLightStateArray, "~/output/state_array", state_profile)
        # timer
        self._timer = self.create_timer(timer_period, self.operation_timer)

        # UDP
        self._th_close = False
        self._udp = udp_control.UdpControl(
            self._ip_address,
            self._send_port,
            self._receive_port,
            receive_timeout,
            buffer_size)
        self._recv_th = threading.Thread(target=self.recv_loop)
        self._recv_th.start()

        self._logger.info("initialized")

    def __del__(self):
        self.fin()

    def fin(self):
        self._th_close = True
        del self._udp

    def recv_loop(self):
        while True:
            if(self._th_close):
                break

            # recv check
            recv_data = self._udp.recv()
            if(recv_data != -1):
                # get gpio
                for i in range(len(recv_data[6])):
                    reply = recv_data[6][i]
                    for key in self._infra_dict.keys():
                        self._logger.info(
                            "reply {0} : {1} {2} {3}" .format(
                                key,
                                self._infra_dict[key].keyvalue["eva_beacon_system:id"],
                                reply["id"],
                                reply["gpio"]))
                        if(reply["id"] == int(
                                self._infra_dict[key].keyvalue["eva_beacon_system:id"])):
                            # Stores values even for reply data
                            #  that is not addressed to my vehicle.
                            self._infra_dict[key].reply_value = reply["gpio"]

            time.sleep(self._udp_recv_interval)

    def on_command_array(self, command_array):
        if(command_array is None):
            return
        if(command_array.commands is None):
            return
        first_regist_flag = False
        for command in command_array.commands:
            if(command.type != "eva_beacon_system"):
                continue
            if(command.state == InfrastructureCommand.NONE):
                continue
            keyvalue = {}
            for kv in command.custom_tags:
                keyvalue[kv.key] = kv.value
            infra = InfrastructureProperty(
                command.state,
                command.stamp.sec,
                command.stamp.nanosec,
                keyvalue.copy())
            if(command.id in self._infra_dict.keys()):
                # update (timeout_cnt clear)
                infra.approval = self._infra_dict[command.id].approval
                infra.reply_value = self._infra_dict[command.id].reply_value
            else:
                # first registration
                first_regist_flag = True
                if(keyvalue["eva_beacon_system:ref:response_type"] == "ALWAYS"):
                    infra.approval = True

            self._infra_dict[command.id] = infra
            if(self._infra_dict[command.id].state == InfrastructureCommand.FINALIZED):
                del self._infra_dict[command.id]

        if(first_regist_flag):
            # send to virtual_traffic_light
            self.send_to_virtualtrafficlight()
            # self._logger.info(
            #    "publish first virtual_traffic_light_state")

    def on_autoware_state(self, state_code):
        self._current_service_layer_state = state_code.service_layer_state
        self._current_control_layer_state = state_code.control_layer_state

    def operation_timer(self):
        if(self._infra_dict is None):
            return
        if(len(self._infra_dict) < 1):
            return

        request_array_cnt = 0
        request_array = []
        for key in self._infra_dict.keys():
            # self._logger.info(
            #    "command {0} : {1} {2}.{3} {4} {5}" .format(
            #        key,
            #        self._infra_dict[key].state,
            #        self._infra_dict[key].sec,
            #        self._infra_dict[key].nanosec,
            #        self._infra_dict[key].keyvalue,
            #        self._infra_dict[key].approval))

            self._udp_send_cnt += 1
            if(self._udp_send_cnt < self._udp_send_cnt_max):
                self._udp_send_cnt = 0
                req = self.make_udp_senddata(self._infra_dict[key])
                if(req is not None):
                    if(request_array_cnt < 6):
                        request_array_cnt += 1
                        request_array.append(req)
                    else:
                        self._logger.error(
                           "send data overflow")

            self.judge_response(self._infra_dict[key])

        # send to infra
        if(request_array_cnt > 0):
            self._udp.send(request_array)
            self._logger.info("send udp {0}".format(request_array))

        # send to virtual_traffic_light
        self.send_to_virtualtrafficlight()
        # self._logger.info(
        #    "publish virtual_traffic_light_state")

        # check timeout
        self.check_timeout_infra_command()

    def make_udp_senddata(self, infra_dict):
        # check "section"
        section_big = ""
        section = infra_dict.keyvalue.get("eva_beacon_system:ref:section")
        if(section is not None):
            section_big = section.upper()
        if(section_big != ""):
            if(infra_dict.state == InfrastructureCommand.REQUESTING):
                if("REQUESTING" not in section_big):
                    return None
            else:
                return None

        # check "state"
        state_big = ""
        state = infra_dict.keyvalue.get("eva_beacon_system:ref:permit_state")
        if(state is not None):
            state_big = state.upper()
        if(state_big != ""):
            if(self._current_control_layer_state == StateMachine.MANUAL):
                # MANUAL
                if("MANUAL" not in state_big):
                    return None
            else:
                # AUTO
                if(self._current_service_layer_state == StateMachine.STATE_EMERGENCY_STOP):
                    if("EMERGENCY" not in state_big):
                        return None
                elif(self._current_service_layer_state == StateMachine.STATE_ARRIVED_GOAL):
                    if("ARRIVAL_GOAL" not in state_big):
                        return None
                elif(self._current_service_layer_state == StateMachine.STATE_INFORM_ENGAGE):
                    if("ENGAGE" not in state_big):
                        return None
                elif((self._current_service_layer_state >= StateMachine.STATE_RUNNING) &
                        (self._current_service_layer_state < StateMachine.STATE_ARRIVED_GOAL)):
                    if("DRIVING" not in state_big):
                        return None
                else:
                    if("STOP" not in state_big):
                        return None

        # check "request"
        ret_value = {}
        if(infra_dict.keyvalue["eva_beacon_system:ref:mode"].upper() == "FIXED_VALUE"):
            ret_value["id"] = int(infra_dict.keyvalue["eva_beacon_system:id"])
            ret_value["request"] = int(infra_dict.keyvalue["eva_beacon_system:ref:request_bit"], 0)
            return ret_value
        elif(infra_dict.keyvalue["eva_beacon_system:ref:mode"].upper() == "TURN_DIRECTION"):
            ret_value["id"] = int(infra_dict.keyvalue["eva_beacon_system:id"])
            if(infra_dict.keyvalue["turn_direction"] == "straight"):
                ret_value["request"] = 1
            elif(infra_dict.keyvalue["turn_direction"] == "right"):
                ret_value["request"] = 2
            elif(infra_dict.keyvalue["turn_direction"] == "left"):
                ret_value["request"] = 4
            else:
                ret_value["request"] = 7
            return ret_value
        else:
            return None

    def judge_response(self, infra_dict):
        expect_value = 0
        compare_value = 0
        if(infra_dict.keyvalue["eva_beacon_system:ref:mode"].upper() == "FIXED_VALUE"):
            expect_value = int(
                infra_dict.keyvalue["eva_beacon_system:ref:expect_bit"], 0)
            compare_value = infra_dict.reply_value >> 4
        elif(infra_dict.keyvalue["eva_beacon_system:ref:mode"].upper() == "TURN_DIRECTION"):
            if(infra_dict.keyvalue["turn_direction"] == "straight"):
                expect_value = 1
            elif(infra_dict.keyvalue["turn_direction"] == "right"):
                expect_value = 2
            elif(infra_dict.keyvalue["turn_direction"] == "left"):
                expect_value = 4
            else:
                expect_value = 0
            compare_value = infra_dict.reply_value >> 4
        else:
            expect_value = 0
            compare_value = 0

        # check for approval
        if(infra_dict.keyvalue["eva_beacon_system:ref:response_type"].upper() == "AND"):
            if((expect_value & compare_value) != 0):
                infra_dict.approval = True
        elif(infra_dict.keyvalue["eva_beacon_system:ref:response_type"].upper() == "MATCH"):
            if(expect_value == compare_value):
                infra_dict.approval = True
        else:
            pass

    def send_to_virtualtrafficlight(self):
        vtl_state_array = VirtualTrafficLightStateArray()
        for key in self._infra_dict.keys():
            ret_value = VirtualTrafficLightState()
            ret_value.stamp = self.get_clock().now().to_msg()
            ret_value.type = "eva_beacon_system"
            ret_value.id = key
            ret_value.approval = self._infra_dict[key].approval
            ret_value.is_finalized = True
            vtl_state_array.states.append(ret_value)

        vtl_state_array.stamp = self.get_clock().now().to_msg()
        self._state_array_publisher.publish(vtl_state_array)

    def check_timeout_infra_command(self):
        delete_list = []
        for key in self._infra_dict.keys():
            self._infra_dict[key].timeout_cnt += 1
            if(self._infra_dict[key].timeout_cnt >= self._infra_command_timeout_cnt_max):
                delete_list.append(key)
        for key in delete_list:
            # self._logger.info("del {}".format(key))
            del self._infra_dict[key]


def shutdown(signal, frame):
    node.fin()
    node.destroy_node()
    rclpy.shutdown()


def main(args=None):
    global node
    signal.signal(signal.SIGINT, shutdown)

    rclpy.init(args=args)

    node = V2iInterfaceNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
