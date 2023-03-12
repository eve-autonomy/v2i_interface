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

import json
from socket import AF_INET, IPPROTO_UDP, SOCK_DGRAM, socket
import time


class UdpControl:
    def __init__(self, address, send_port, recv_port, timeout, buff_size):
        self._send_seq_num = 0
        self._send_address = address
        self._send_port = send_port
        self._send_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)

        # '0.0.0.0' = INADDR_ANY
        self._recv_address = '0.0.0.0'
        self._recv_port = recv_port
        self._buff_size = buff_size
        self._recv_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)
        self._recv_socket.bind((self._recv_address, self._recv_port))
        self._recv_socket.settimeout(timeout)

    def __del__(self):
        self._send_socket.close()
        self._recv_socket.close()

    def send(self, request_array):
        now_time = time.time_ns()
        sec_time = (int)(now_time / 1000 / 1000 / 1000)
        nanosec_time = now_time - (sec_time * 1000 * 1000 * 1000)

        payload = {
            "seq_num": self._send_seq_num,
            "time": {
                "sec": sec_time,
                "nanosec": nanosec_time},
            "request_array": request_array}

        try:
            self._send_socket.sendto(
                json.dumps(payload).encode("utf-8"),
                (self._send_address, self._send_port))
            self._send_seq_num += 1
        except OSError:
            return -1

        return (self._send_seq_num, now_time)

    def recv(self):
        try:
            message, cli_addr = self._recv_socket.recvfrom(self._buff_size)
        except OSError:
            return -1

        now_time = time.time_ns()
        data = json.loads(message)
        id = data["id"]
        recv_seq_num = data["seq_num"]
        packet_time = (data["time"]["sec"] * 1000 *
                       1000 * 1000) + data["time"]["nanosec"]
        status = data["status"]
        detail = data["detail"]
        reply_array = data["reply_array"]

        return (
            recv_seq_num,
            now_time,
            packet_time,
            id,
            status,
            detail,
            reply_array)
