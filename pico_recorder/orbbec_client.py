#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ZMQ REQ 客户端: 从 Jetson 上的 Orbbec 相机流服务器拉取最新一帧。

用于遥操机的数据记录线程, 每个采样周期调用一次 fetch_frames()。
"""
import json

import zmq


class OrbbecImageClient:
    def __init__(self, host: str, port: int, timeout_ms: int = 300):
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.REQ)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._sock.connect(f"tcp://{host}:{port}")
        self.last_meta = None

    def fetch_frames(self):
        """拉取所有相机最新一帧。

        Returns:
            (meta, frames)——
              meta:  服务器返回的 meta dict。
              frames: {serial: {"jpg": bytes, "ns": int 本地采集时刻(ns), "w": int, "h": int}}
            任一步失败(无相机/超时)返回 (None, None)。
        """
        try:
            self._sock.send_string("GET")
            parts = self._sock.recv_multipart()
        except zmq.ZMQError:
            return None, None
        if not parts:
            return None, None
        meta = self._loads(parts[0])
        if meta is None:
            return None, None
        self.last_meta = meta
        frames = {}
        for i, sn in enumerate(meta.get("serials", [])):
            info = meta["frames"].get(sn, {})
            payload = parts[i + 1] if (i + 1) < len(parts) else b""
            frames[sn] = {
                "jpg": payload,
                "ns": info.get("ns", 0),
                "w": info.get("w", 0),
                "h": info.get("h", 0),
            }
        return meta, frames

    @staticmethod
    def _loads(raw: bytes):
        try:
            return json.loads(raw.decode("utf-8"))
        except Exception:
            return None

    def close(self):
        try:
            self._sock.close(linger=0)
            self._ctx.term()
        except Exception:
            pass