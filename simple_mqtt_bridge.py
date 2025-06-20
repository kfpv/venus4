import subprocess
import threading
import time
import json
import signal
import os
from typing import Dict, Callable, Optional, Tuple
import queue
class SimpleMQTTBridge:
    def __init__(self):
        self.process = None
        self.subscribers = {}
        self.running = False
        self.receive_thread = None
        self.message_queue = queue.Queue()
    def start(self) -> bool:
        try:
            self.process = subprocess.Popen(
                ['./mqtt', 'daemon'],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )
            time.sleep(0.1)
            self.running = True
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.receive_thread.start()
            print("Simple MQTT Bridge started (daemon mode)")
            return True
        except Exception as e:
            print(f"Failed to start bridge: {e}")
            return False
    def stop(self):
        self.running = False
        if self.process:
            try:
                self.process.stdin.write("QUIT\n")
                self.process.stdin.flush()
                self.process.wait(timeout=2)
            except:
                self.process.terminate()
                try:
                    self.process.wait(timeout=1)
                except:
                    self.process.kill()
            self.process = None
        if self.receive_thread:
            self.receive_thread.join(timeout=1)
        print("Simple MQTT Bridge stopped")
    def publish(self, topic: str, message: str) -> bool:
        if not self.process or not self.running:
            print("Bridge not running")
            return False
        try:
            chunk_size = 255
            chunks = [message[i:i+chunk_size] for i in range(0, len(message), chunk_size)]
            total_chunks = len(chunks)
            for idx, chunk in enumerate(chunks):
                combined = f"{topic}|{total_chunks}|{idx}|{chunk}\n"
                self.process.stdin.write(combined)
                self.process.stdin.flush()
                print(f"Published chunk {idx+1}/{total_chunks} to '{topic}': {chunk[:30]}{'...' if len(chunk)>30 else ''}")
                if idx < total_chunks - 1:
                    time.sleep(1)
            return True
        except Exception as e:
            print(f"Failed to publish: {e}")
            return False
    def subscribe(self, topic: str, callback: Callable[[str, str], None]):
        if topic not in self.subscribers:
            self.subscribers[topic] = []
        self.subscribers[topic].append(callback)
        print(f"Subscribed to '{topic}'")
    def unsubscribe(self, topic: str, callback: Optional[Callable] = None):
        if topic in self.subscribers:
            if callback and callback in self.subscribers[topic]:
                self.subscribers[topic].remove(callback)
            else:
                self.subscribers[topic].clear()
            if not self.subscribers[topic]:
                del self.subscribers[topic]
    def _receive_loop(self):
        while self.running and self.process:
            try:
                self.process.stdin.write("RECEIVE\n")
                self.process.stdin.flush()
                line = self.process.stdout.readline()
                if not line:
                    time.sleep(0.01)
                    continue
                line = line.strip()
                if not line:
                    continue
                if line.startswith("RECEIVED "):
                    msg_part = line[len("RECEIVED "):]
                    if "|" in msg_part:
                        topic, message = msg_part.split("|", 1)
                        self._handle_received_message(topic, message)
                    else:
                        self._handle_received_message("default", msg_part)
                time.sleep(1)
            except Exception as e:
                if self.running:
                    print(f"Receive error: {e}")
                time.sleep(0.1)
    def _handle_received_message(self, topic: str, message: str):
        try:
            parts = message.split('|', 3)
            if len(parts) == 3 and parts[0].isdigit() and parts[1].isdigit():
                total_chunks, chunk_idx, chunk_data = int(parts[0]), int(parts[1]), parts[2]
                if not hasattr(self, '_chunk_buffers'):
                    self._chunk_buffers = {}
                key = (topic,)
                if key not in self._chunk_buffers:
                    self._chunk_buffers[key] = [None] * total_chunks
                self._chunk_buffers[key][chunk_idx] = chunk_data
                if all(x is not None for x in self._chunk_buffers[key]):
                    full_message = ''.join(self._chunk_buffers[key])
                    print(f"Received full message from '{topic}': {full_message[:30]}{'...' if len(full_message)>30 else ''}")
                    self.message_queue.put((topic, full_message))
                    if topic in self.subscribers:
                        for callback in self.subscribers[topic]:
                            try:
                                callback(topic, full_message)
                            except Exception as e:
                                print(f"Callback error: {e}")
                    if "*" in self.subscribers:
                        for callback in self.subscribers["*"]:
                            try:
                                callback(topic, full_message)
                            except Exception as e:
                                print(f"Wildcard callback error: {e}")
                    del self._chunk_buffers[key]
                return
        except Exception as e:
            print(f"Chunk reassembly error: {e}")
        print(f"Received from '{topic}': {message}")
        self.message_queue.put((topic, message))
        if topic in self.subscribers:
            for callback in self.subscribers[topic]:
                try:
                    callback(topic, message)
                except Exception as e:
                    print(f"Callback error: {e}")
        if "*" in self.subscribers:
            for callback in self.subscribers["*"]:
                try:
                    callback(topic, message)
                except Exception as e:
                    print(f"Wildcard callback error: {e}")
    def get_message(self, timeout: float = 1.0) -> Optional[Tuple[str, str]]:
        try:
            return self.message_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    def list_subscriptions(self):
        print("Current subscriptions:")
        for topic, callbacks in self.subscribers.items():
            print(f"  - {topic}: {len(callbacks)} callback(s)")
    def __enter__(self):
        self.start()
        return self
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
MQTTUARTBridge = SimpleMQTTBridge
class MQTTMessage:
    def __init__(self, topic, payload):
        self.topic = topic
        self.payload = payload.encode() if isinstance(payload, str) else payload
class MQTTClient:
    def __init__(self, userdata=None):
        self._bridge = SimpleMQTTBridge()
        self._userdata = userdata
        self.on_connect = None
        self.on_disconnect = None
        self.on_message = None
        self._loop_running = False
        self._subscriptions = set()
        self._thread = None
    def connect(self, *args, **kwargs):
        if self._bridge.start():
            if self.on_connect:
                self.on_connect(self, self._userdata, {}, 0)
            return 0
        else:
            if self.on_connect:
                self.on_connect(self, self._userdata, {}, 1)
            return 1
    def disconnect(self):
        self._bridge.stop()
        if self.on_disconnect:
            self.on_disconnect(self, self._userdata, 0)
    def subscribe(self, topic, qos=0):
        def handler(t, m):
            if self.on_message:
                msg = MQTTMessage(t, m)
                self.on_message(self, self._userdata, msg)
        self._bridge.subscribe(topic, handler)
        self._subscriptions.add(topic)
        return (0, qos)
    def unsubscribe(self, topic):
        self._bridge.unsubscribe(topic)
        self._subscriptions.discard(topic)
        return (0,)
    def publish(self, topic, payload, qos=0, retain=False):
        self._bridge.publish(topic, payload)
        class Result:
            rc = 0
        return Result()
    def loop_start(self):
        if not self._loop_running:
            self._loop_running = True
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()
    def loop_stop(self):
        self._loop_running = False
        if self._thread:
            self._thread.join(timeout=1)
            self._thread = None
    def _loop(self):
        while self._loop_running:
            time.sleep(0.1)
    def user_data_set(self, userdata):
        self._userdata = userdata