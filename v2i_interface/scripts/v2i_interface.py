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
from rclpy.executors import ExternalShutdownException 
from rclpy.time import Time

from v2i_interface_msgs.msg import (
    InfrastructureCommand, InfrastructureCommandArray,
    InfrastructureState, InfrastructureStateArray
)

import udp_control

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

class V2iInterfaceNode(Node):
    recv_lock = threading.Lock()

    def __init__(self):
        super().__init__('v2i_interface')
        self._logger = self.get_logger()

        timer_period = 0.1
        self._udp_recv_interval = 0.1

        self._data_store_timeout_sec = 1.0

        ip_address = self.declare_parameter("ip_address", "127.0.0.1")
        send_port = self.declare_parameter("send_port", 0)
        receive_port = self.declare_parameter("receive_port", 0)
        self._ip_address = ip_address.get_parameter_value().string_value
        self._send_port = send_port.get_parameter_value().integer_value
        self._receive_port = receive_port.get_parameter_value().integer_value
        self._buffer_size = 4096
        self._receive_timeout = 0.5
        self._receive_thread_close_timeout = 10
        self._socket_reopen_interval = 3

        # QoS Setting
        depth = 1
        cmd_profile = QoSProfile(depth=depth)
        state_profile = QoSProfile(depth=depth)
        state_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        # pub/sub
        self._command_array_subscriber = self.create_subscription(
            InfrastructureCommandArray, "~/input/command_array",
            self.on_command_array, cmd_profile)
        self._state_array_publisher = self.create_publisher(
            InfrastructureStateArray, "~/output/state_array", state_profile)
        # timer
        self._timer = self.create_timer(timer_period, self.output_timer)

        # UDP
        self._th_close = False
        self._udp = udp_control.UdpControl(
            self._ip_address,
            self._send_port,
            self._receive_port,
            self._receive_timeout,
            self._buffer_size)
        self._recv_th = threading.Thread(target=self.recv_loop)
        self._recv_th.start()

        self._logger.info("initialized")
        self._recv_array = []
        self._recv_stamp = self.get_clock().now()
        self._request_array = []
        self._request_stamp = self.get_clock().now()

    def create_new_udp_control(self):
        self._udp = udp_control.UdpControl(
            self._ip_address,
            self._send_port,
            self._receive_port,
            self._receive_timeout,
            self._buffer_size
        )
    def fin(self):
        self._th_close = True
        del self._udp
        self.destroy_node()

    def recv_loop(self):
        while True:
            time.sleep(self._udp_recv_interval)
            if(self._th_close):
                break
            recv_data = self._udp.recv()
            if(recv_data == -1):
                continue
            recv_array = []
            for i in range(len(recv_data[6])):
                reply = recv_data[6][i]
                recv_array.append(reply)
            if(len(recv_array) > 0):
                with self.recv_lock:
                    self._recv_array = recv_array
                    self._recv_stamp = self.get_clock().now()

    def on_command_array(self, command_array):
        if(command_array is None):
            return
        if(command_array.commands is None):
            return
        request_array = []
        for command in command_array.commands:
            if(command.state == InfrastructureCommand.NONE or
               command.state == InfrastructureCommand.INTURRUPT_STOP):
                continue
            state = command.state
            if (command.state == InfrastructureCommand.SEND_ZERO):
                state = 0
            ret_value = {}
            ret_value["id"] = command.id
            ret_value["request"] = state
            request_array.append(ret_value)
        self._request_array = request_array
        self._request_stamp = self.get_clock().now()

    def output_timer(self):
        self.send_udp_command()
        self.publish_infrastructure_states()
        if (len(self._request_array) > 0):
            if (self.is_timeout(self._request_stamp)):
                self._request_array = []
        with self.recv_lock:
            if (self._recv_array is not None):
                if (self.is_timeout(self._recv_stamp)):
                    self._recv_array = []

    def send_udp_command(self):
        if(self._request_array is None):
            return
        if(len(self._request_array) > 0):
            ret = self._udp.send(self._request_array)
            if (ret == -1):
                self._logger.error("send udp error")
                raise RuntimeError

    def publish_infrastructure_states(self):
        with self.recv_lock:
            if (self._recv_array is None):
                return
            recv_array = self._recv_array
        vtl_state_array = InfrastructureStateArray()
        stamp = self.get_clock().now().to_msg()
        for reply in recv_array:
            ret_value = InfrastructureState()
            ret_value.stamp = stamp
            ret_value.id = reply["id"]
            ret_value.state = reply["gpio"]
            self.convert_state(ret_value)
            vtl_state_array.states.append(ret_value)
        vtl_state_array.stamp = stamp
        self._state_array_publisher.publish(vtl_state_array)

    def convert_state(self, state : InfrastructureState):
        state.state = state.state >> 4

    def is_timeout(self, stamp):
        duration = self.get_clock().now() - stamp
        duration_sec = duration.nanoseconds * 1e-9
        return (duration_sec > self._data_store_timeout_sec)

def reopen_socket(node):
    node._th_close = True
    node._recv_th.join(node._receive_thread_close_timeout)
    if node._recv_th.is_alive():
        node.fin()
        rclpy.try_shutdown()
        return
    del node._udp
    time.sleep(node._socket_reopen_interval)
    node.create_new_udp_control()
    node._th_close = False
    node._recv_th = threading.Thread(target=node.recv_loop)
    node._recv_th.start()
    return

def main(args=None):
    try:
        global node
        rclpy.init(args=args)
        node = V2iInterfaceNode()
        while rclpy.ok():
            try:
                rclpy.spin(node)
            except RuntimeError:
                reopen_socket(node)
                continue
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    except RuntimeError:
        pass
    finally:
        node.fin()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()