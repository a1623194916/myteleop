#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""遥操机 -> Jetson 采集服务 的轻量 ZMQ 控制客户端(只发指令, 不传图)。"""
import json

import zmq


class RecordClient:
    def __init__(self, host="192.168.5.27", port=8766, timeout_ms=500.0,
                 state_port=None):
        self._host = host
        self._port = int(port)
        self._timeout_ms = int(timeout_ms)
        self._ctx = zmq.Context()
        self._new_socket()
        # 遥操真值转发: 独占 CNDE 状态流, 推给采集端避免争用(独立 PUSH->PULL)。
        self._state_sock = None
        state_port = int(state_port) if state_port else self._port + 1
        try:
            self._state_sock = self._ctx.socket(zmq.PUSH)
            self._state_sock.setsockopt(zmq.LINGER, 0)
            self._state_sock.setsockopt(zmq.SNDTIMEO, 200)
            self._state_sock.connect(f"tcp://{host}:{state_port}")
        except Exception as e:
            print(f"[ctrl] 遥操状态推送端口不可用({e}); 将只发 START/STOP")
            self._state_sock = None

    def _new_socket(self):
        self._sock = self._ctx.socket(zmq.REQ)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.setsockopt(zmq.RCVTIMEO, self._timeout_ms)
        self._sock.connect(f"tcp://{self._host}:{self._port}")

    def push_state(self, left_deg, right_deg, left_gripper_pct=-1.0,
                   right_gripper_pct=-1.0):
        """把遥操端 CNDE 真值(关节 deg + 夹爪 0~100)推给采集端(尽力而为)。"""
        if self._state_sock is None:
            return
        try:
            self._state_sock.send_string(json.dumps({
                "type": "STATE",
                "left_deg": [float(x) for x in left_deg],
                "right_deg": [float(x) for x in right_deg],
                "left_gripper_pct": float(left_gripper_pct),
                "right_gripper_pct": float(right_gripper_pct),
            }))
        except zmq.ZMQError:
            pass

    def _call(self, payload, timeout_ms=None):
        try:
            if timeout_ms is not None and timeout_ms != self._timeout_ms:
                self._sock.setsockopt(zmq.RCVTIMEO, int(timeout_ms))
            self._sock.send_string(json.dumps(payload))
            parts = self._sock.recv_string()
            return json.loads(parts)
        except zmq.ZMQError as e:
            print(f"[ctrl] 与采集服务器通信失败: {e}")
            try:
                self._new_socket()   # REQ 出错后必须重建
            except Exception:
                pass
            return {"ok": False, "error": repr(e)}
        finally:
            if timeout_ms is not None and timeout_ms != self._timeout_ms:
                try:
                    self._sock.setsockopt(zmq.RCVTIMEO, self._timeout_ms)
                except zmq.ZMQError:
                    pass

    def start(self, task):
        return self._call({"cmd": "START", "task": task})

    def stop(self):
        # STOP waits for the recorder's write queue to drain and done.json to
        # be written; a few seconds is normal for a busy episode, so do not
        # reuse the short control-call timeout here.
        return self._call({"cmd": "STOP"}, timeout_ms=10000)

    def ping(self):
        return self._call({"cmd": "PING"})

    def close(self):
        try:
            if self._state_sock is not None:
                self._state_sock.close(linger=0)
        except Exception:
            pass
        try:
            self._sock.close(linger=0)
            self._ctx.term()
        except Exception:
            pass