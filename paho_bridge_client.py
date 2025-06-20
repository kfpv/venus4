import paho.mqtt.client as paho
import threading
import time

BROKERS = [
    {
        'username': 'robot_8_2',
        'password': 'cyxTestud\\',
        'subscribe': '/pynqbridge/8/send',
        'publish': '/pynqbridge/8/recv',
        'robot': 'robot1',
        'number': '8',
    },
    {
        'username': 'robot_7_2',
        'password': 'yiamNipMym',
        'subscribe': '/pynqbridge/7/send',
        'publish': '/pynqbridge/7/recv',
        'robot': 'robot2',
        'number': '7',
    },
]
MQTT_BROKER = "mqtt.ics.ele.tue.nl"
MQTT_PORT = 1883

class MQTTMessage:
    def __init__(self, topic, payload):
        self.topic = topic
        self.payload = payload.encode() if isinstance(payload, str) else payload

class MQTTClient:
    def __init__(self, userdata=None):
        self._clients = []
        self._userdata = userdata
        self.on_connect = None
        self.on_disconnect = None
        self.on_message = None
        self._loop_running = False
        self._threads = []
        self._subscriptions = set()
        for broker in BROKERS:
            client = paho.Client()
            client.username_pw_set(broker['username'], broker['password'])
            client.on_connect = self._make_on_connect_internal(broker)
            client.on_message = self._make_on_message_internal(broker)
            client.on_disconnect = self._make_on_disconnect_internal(broker)
            self._clients.append({'client': client, 'broker': broker})

    def connect(self, *args, **kwargs):
        for entry in self._clients:
            entry['client'].connect(MQTT_BROKER, MQTT_PORT, 60)
        return 0

    def disconnect(self):
        for entry in self._clients:
            entry['client'].disconnect()

    def subscribe(self, topic=None, qos=0):
        for entry in self._clients:
            entry['client'].subscribe(entry['broker']['subscribe'], qos)
            self._subscriptions.add(entry['broker']['subscribe'])
        return (0, qos)

    def unsubscribe(self, topic=None):
        for entry in self._clients:
            entry['client'].unsubscribe(entry['broker']['subscribe'])
            self._subscriptions.discard(entry['broker']['subscribe'])
        return (0,)

    def publish(self, topic=None, payload=None, qos=0, retain=False):
        if topic is not None and payload is not None:
            message = payload.decode() if isinstance(payload, bytes) else str(payload)
            chunk_size = 255
            chunks = [message[i:i+chunk_size] for i in range(0, len(message), chunk_size)]
            total_chunks = len(chunks)
            for idx, chunk in enumerate(chunks):
                print(f"Publishing chunk {idx+1}/{total_chunks} to '{topic}': {chunk[:30]}{'...' if len(chunk)>30 else ''}")
                combined_payload = f"{topic}|{total_chunks}|{idx}|{chunk}"
                try:
                    topic_str = topic if topic is not None else ''
                    if topic_str.startswith('robot1'):
                        idx_robot = 0
                    elif topic_str.startswith('robot2'):
                        idx_robot = 1
                    else:
                        idx_robot = 0
                    entry = self._clients[idx_robot]
                    entry['client'].publish(entry['broker']['publish'], combined_payload, qos, retain)
                except Exception:
                    self._clients[0]['client'].publish(self._clients[0]['broker']['publish'], combined_payload, qos, retain)
                if idx < total_chunks - 1:
                    time.sleep(1)
            class Result:
                rc = 0
            return Result()
        elif payload is not None:
            combined_payload = payload.decode() if isinstance(payload, bytes) else str(payload)
            try:
                self._clients[0]['client'].publish(self._clients[0]['broker']['publish'], combined_payload, qos, retain)
            except Exception:
                pass
            class Result:
                rc = 0
            return Result()
        else:
            return

    def loop_start(self):
        if not self._loop_running:
            self._loop_running = True
            for entry in self._clients:
                t = threading.Thread(target=entry['client'].loop_forever, daemon=True)
                t.start()
                self._threads.append(t)

    def loop_stop(self):
        self._loop_running = False
        for entry in self._clients:
            entry['client'].loop_stop()
        for t in self._threads:
            t.join(timeout=1)
        self._threads = []

    def user_data_set(self, userdata):
        self._userdata = userdata

    def _make_on_connect_internal(self, broker):
        def _on_connect(client, userdata, flags, rc):
            client.subscribe(broker['subscribe'])
            if self.on_connect:
                self.on_connect(self, self._userdata, flags, rc)
        return _on_connect

    def _make_on_disconnect_internal(self, broker):
        def _on_disconnect(client, userdata, rc):
            if self.on_disconnect:
                self.on_disconnect(self, self._userdata, rc)
        return _on_disconnect

    def _make_on_message_internal(self, broker):
        def _on_message(client, userdata, msg):
            if self.on_message:
                try:
                    payload_str = msg.payload.decode(errors='replace')
                    parts = payload_str.split('|', 3)
                    if len(parts) == 4:
                        virtual_topic, total_chunks, chunk_idx, chunk_data = parts[0], int(parts[1]), int(parts[2]), parts[3]
                        if not hasattr(self, '_chunk_buffers'):
                            self._chunk_buffers = {}
                        key = (virtual_topic, broker['robot'])
                        if key not in self._chunk_buffers:
                            self._chunk_buffers[key] = [None] * total_chunks
                        self._chunk_buffers[key][chunk_idx] = chunk_data
                        if all(x is not None for x in self._chunk_buffers[key]):
                            full_message = ''.join(self._chunk_buffers[key])
                            mqtt_msg = MQTTMessage(virtual_topic, full_message)
                            self.on_message(self, self._userdata, mqtt_msg)
                            del self._chunk_buffers[key]
                        return
                    elif '|' in payload_str:
                        virtual_topic, message = payload_str.split('|', 1)
                        if (broker['robot'] == 'robot1' and virtual_topic.startswith('robot1')) or \
                           (broker['robot'] == 'robot2' and virtual_topic.startswith('robot2')):
                            mqtt_msg = MQTTMessage(virtual_topic, message)
                            self.on_message(self, self._userdata, mqtt_msg)
                except Exception:
                    pass
        return _on_message

    def _on_connect_internal(self, client, userdata, flags, rc):
        client.subscribe(TOPIC_SUBSCRIBE)
        if self.on_connect:
            self.on_connect(self, self._userdata, flags, rc)

    def _on_disconnect_internal(self, client, userdata, rc):
        if self.on_disconnect:
            self.on_disconnect(self, self._userdata, rc)

    def _on_message_internal(self, client, userdata, msg):
        if self.on_message:
            try:
                payload_str = msg.payload.decode(errors='replace')
                if '|' in payload_str:
                    virtual_topic, message = payload_str.split('|', 1)
                    mqtt_msg = MQTTMessage(virtual_topic, message)
                else:
                    mqtt_msg = MQTTMessage(msg.topic, msg.payload)
            except Exception:
                mqtt_msg = MQTTMessage(msg.topic, msg.payload)
            self.on_message(self, self._userdata, mqtt_msg)