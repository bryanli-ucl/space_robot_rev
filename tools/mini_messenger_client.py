#!/usr/bin/env python3
"""PC-side MiniMessenger-compatible MQTT bridge.

This keeps MQTT on the laptop and talks to the robot through the existing
wireless TCP shell on port 7777.

Examples:
    python3 tools/mini_messenger_client.py --broker 192.168.0.74 --group 12 --board Terminator
    python3 tools/mini_messenger_client.py --broker 192.168.0.74 --robot 192.168.1.112

Interactive commands:
    robot task 2
    task 7
    register
    getmap
    pose 1 2 7 3 4 120 2
    seed ABC12345 4 5
    fertile ABC12345
    airlock A ABC12345
    revive 12 1
    mqtt type=hello team_id=12 board_id=Terminator
"""

from __future__ import annotations

import argparse
import asyncio
import re
import signal
import sys
import time
from dataclasses import dataclass


MQTT_CONNECT = 1
MQTT_CONNACK = 2
MQTT_PUBLISH = 3
MQTT_PUBACK = 4
MQTT_SUBSCRIBE = 8
MQTT_SUBACK = 9
MQTT_PINGREQ = 12
MQTT_PINGRESP = 13
MQTT_DISCONNECT = 14


def encode_remaining_length(length: int) -> bytes:
    encoded = bytearray()
    while True:
        byte = length % 128
        length //= 128
        if length:
            byte |= 0x80
        encoded.append(byte)
        if not length:
            return bytes(encoded)


def encode_string(value: str) -> bytes:
    raw = value.encode("utf-8")
    return len(raw).to_bytes(2, "big") + raw


async def read_remaining_length(reader: asyncio.StreamReader) -> int:
    multiplier = 1
    value = 0

    for _ in range(4):
        encoded_byte = (await reader.readexactly(1))[0]
        value += (encoded_byte & 127) * multiplier
        if (encoded_byte & 128) == 0:
            return value
        multiplier *= 128

    raise ValueError("malformed MQTT remaining length")


def read_string(data: bytes, offset: int) -> tuple[str, int]:
    length = int.from_bytes(data[offset : offset + 2], "big")
    offset += 2
    end = offset + length
    return data[offset:end].decode("utf-8", errors="replace"), end


def build_packet(packet_type: int, flags: int, payload: bytes) -> bytes:
    return bytes([(packet_type << 4) | flags]) + encode_remaining_length(len(payload)) + payload


def printable_payload(payload: bytes) -> str:
    try:
        return payload.decode("utf-8")
    except UnicodeDecodeError:
        return payload.hex()


def is_probably_text_payload(payload: bytes) -> bool:
    if not payload:
        return False
    return all(byte in (9, 10, 13) or 32 <= byte <= 126 for byte in payload)


def parse_key_values(text: str) -> dict[str, str]:
    values: dict[str, str] = {}
    for token in text.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        values[key] = value
    return values


def str_bool(value: str | None, fallback: bool = False) -> bool:
    if value is None:
        return fallback
    return value.lower() in ("1", "true", "yes", "on")


def decode_grid_map(payload: bytes, width: int = 9, height: int = 9) -> list[list[int]]:
    cells: list[list[int]] = []
    for y in range(height):
        row: list[int] = []
        for x in range(width):
            cell_index = y * width + x
            bit_index = cell_index * 2
            byte_index = bit_index // 8
            shift = bit_index % 8
            row.append((payload[byte_index] >> shift) & 0x03)
        cells.append(row)
    return cells


def format_grid_map(payload: bytes, width: int = 9, height: int = 9) -> str:
    symbols = {
        0: ".",
        1: "F",
        2: "S",
        3: "?",
    }
    names = {
        0: "sterile",
        1: "fertile",
        2: "seeded",
        3: "unexplored",
    }
    counts = {state: 0 for state in symbols}
    rows = decode_grid_map(payload, width, height)
    lines = ["[server] grid map 9x9 legend .=sterile F=fertile S=seeded ?=unexplored"]

    for y, row in enumerate(rows):
        for state in row:
            counts[state] += 1
        rendered = " ".join(symbols[state] for state in row)
        lines.append(f"[server] y={y} {rendered}")

    summary = " ".join(f"{names[state]}={counts[state]}" for state in range(4))
    lines.append(f"[server] map summary {summary}")
    return "\n".join(lines)


@dataclass
class RobotEvent:
    rfid_uid: str | None = None
    task_done: int | None = None
    task_phase: str | None = None


class MiniMessengerClient:
    def __init__(
        self,
        host: str,
        port: int,
        group_id: str,
        board_id: str,
        register_interval_s: float,
        connect_timeout_s: float,
    ) -> None:
        self.host = host
        self.port = port
        self.group_id = group_id
        self.board_id = board_id
        self.register_interval_s = register_interval_s
        self.connect_timeout_s = connect_timeout_s
        self.client_id = f"g{int(group_id):02d}-b{int(board_id) if board_id.isdigit() else 0:02d}-pc"
        self.reader: asyncio.StreamReader | None = None
        self.writer: asyncio.StreamWriter | None = None
        self.connected = False
        self.last_register_s = 0.0
        self.last_ping_s = 0.0

    @property
    def status_topic(self) -> str:
        return f"lab/g/{self.group_id}/board/{self.board_id}/status"

    @property
    def group_topic(self) -> str:
        return f"lab/g/{self.group_id}/from/{self.board_id}/to/all"

    @property
    def board_subscription(self) -> str:
        return f"lab/g/{self.group_id}/from/+/to/{self.board_id}"

    @property
    def group_subscription(self) -> str:
        return f"lab/g/{self.group_id}/from/+/to/all"

    @property
    def any_group_target_subscription(self) -> str:
        return f"lab/g/{self.group_id}/from/+/to/+"

    async def connect(self) -> None:
        await self.drop_connection()
        print(f"[mqtt] connecting {self.host}:{self.port}", flush=True)
        self.reader, self.writer = await asyncio.wait_for(
            asyncio.open_connection(self.host, self.port),
            timeout=self.connect_timeout_s,
        )

        variable_header = encode_string("MQTT") + bytes([4, 0x06, 0, 60])
        will_payload = b"offline"
        payload = (
            encode_string(self.client_id)
            + encode_string(self.status_topic)
            + len(will_payload).to_bytes(2, "big")
            + will_payload
        )
        self.writer.write(build_packet(MQTT_CONNECT, 0, variable_header + payload))
        await self.writer.drain()

        packet_type, _, payload = await asyncio.wait_for(
            self.read_packet(),
            timeout=self.connect_timeout_s,
        )
        if packet_type != MQTT_CONNACK or len(payload) < 2 or payload[1] != 0:
            raise ConnectionError(f"MQTT connect failed: {payload.hex()}")

        self.connected = True
        print(f"[mqtt] connected {self.host}:{self.port} client_id={self.client_id}", flush=True)
        await self.subscribe_many(
            [
                self.board_subscription,
                self.group_subscription,
                self.any_group_target_subscription,
                "lab/map/grid",
            ]
        )
        await self.publish(self.status_topic, b"online", retain=True, qos=1)
        await self.register()
        await self.get_map()

    async def drop_connection(self) -> None:
        self.connected = False
        if self.writer is None:
            self.reader = None
            return

        writer = self.writer
        self.reader = None
        self.writer = None

        try:
            writer.close()
            await writer.wait_closed()
        except (OSError, ConnectionError, RuntimeError):
            pass

    async def read_packet(self) -> tuple[int, int, bytes]:
        if self.reader is None:
            raise ConnectionError("MQTT reader is not connected")

        first = (await self.reader.readexactly(1))[0]
        remaining = await read_remaining_length(self.reader)
        payload = await self.reader.readexactly(remaining) if remaining else b""
        return first >> 4, first & 0x0F, payload

    async def subscribe(self, topic_filter: str) -> None:
        if self.writer is None:
            raise ConnectionError("MQTT writer is not connected")

        packet_id = int(time.monotonic() * 1000) & 0xFFFF
        payload = packet_id.to_bytes(2, "big") + encode_string(topic_filter) + b"\x00"
        self.writer.write(build_packet(MQTT_SUBSCRIBE, 0x02, payload))
        await self.writer.drain()

        packet_type, _, response = await self.read_packet()
        if packet_type != MQTT_SUBACK or len(response) < 3 or response[-1] == 0x80:
            raise ConnectionError(f"MQTT subscribe failed for {topic_filter}")

        print(f"[mqtt] subscribed {topic_filter}", flush=True)

    async def subscribe_many(self, topic_filters: list[str]) -> None:
        seen: set[str] = set()
        for topic_filter in topic_filters:
            if topic_filter in seen:
                continue
            seen.add(topic_filter)
            await self.subscribe(topic_filter)

    async def publish(self, topic: str, payload: bytes | str, retain: bool = False, qos: int = 0) -> None:
        if self.writer is None:
            raise ConnectionError("MQTT writer is not connected")
        if isinstance(payload, str):
            payload = payload.encode("utf-8")

        flags = (0x01 if retain else 0) | ((qos & 0x03) << 1)
        packet = encode_string(topic) + payload
        self.writer.write(build_packet(MQTT_PUBLISH, flags, packet))
        await self.writer.drain()
        print(f"[mqtt ->] {topic} {printable_payload(payload)}", flush=True)

    async def publish_group(self, payload: str | bytes) -> None:
        await self.publish(self.group_topic, payload)

    async def register(self) -> None:
        await self.publish_group(f"type=register team_id={self.group_id} board_id={self.board_id}")
        self.last_register_s = time.monotonic()

    async def get_map(self) -> None:
        await self.publish_group(f"type=getMap team_id={self.group_id} board_id={self.board_id}")

    async def pose(
        self,
        x: int,
        y: int,
        task: int = 0,
        target_x: int = 0,
        target_y: int = 0,
        score: int = 0,
        seeds: int = 0,
    ) -> None:
        await self.publish_group(
            "type=pose "
            f"team_id={self.group_id} board_id={self.board_id} "
            f"x={x} y={y} task={task} target_x={target_x} target_y={target_y} "
            f"score={score} seeds={seeds}"
        )

    async def seed_planted(self, tag_id: str, x: int = 0, y: int = 0) -> None:
        await self.publish_group(
            f"type=seedPlanted tag_id={tag_id} team_id={self.group_id} "
            f"board_id={self.board_id} x={x} y={y}"
        )

    async def is_fertile(self, tag_id: str) -> None:
        await self.publish_group(
            f"type=isFertile tag_id={tag_id} team_id={self.group_id} board_id={self.board_id}"
        )

    async def open_airlock(self, airlock: str, tag_id: str) -> None:
        airlock = airlock.upper()
        if airlock not in ("A", "B"):
            raise ValueError("airlock side must be A or B")
        await self.publish_group(
            f"type=openAirlock airlock={airlock} tag_id={tag_id} "
            f"team_id={self.group_id} board_id={self.board_id}"
        )

    async def revive_request(self, target_team: str, target_board: str) -> None:
        await self.publish_group(
            f"type=reviveRequest target_team={target_team} target_board={target_board} "
            f"team_id={self.group_id} board_id={self.board_id}"
        )

    async def loop(self) -> None:
        while True:
            try:
                if not self.connected:
                    try:
                        await self.connect()
                    except (OSError, ConnectionError, asyncio.IncompleteReadError) as exc:
                        print(f"[mqtt] reconnect failed: {exc}", flush=True)
                        await asyncio.sleep(2.0)
                        continue

                packet_type, flags, payload = await asyncio.wait_for(self.read_packet(), timeout=0.5)
            except asyncio.TimeoutError:
                try:
                    now = time.monotonic()
                    if now - self.last_register_s >= self.register_interval_s:
                        await self.register()
                    if now - self.last_ping_s >= 20.0:
                        await self.ping()
                except (OSError, ConnectionError, asyncio.IncompleteReadError) as exc:
                    print(f"[mqtt] disconnected during keepalive: {exc}", flush=True)
                    await self.drop_connection()
                    await asyncio.sleep(1.0)
                continue
            except (OSError, ConnectionError, asyncio.IncompleteReadError) as exc:
                print(f"[mqtt] disconnected: {exc}", flush=True)
                await self.drop_connection()
                await asyncio.sleep(1.0)
                continue

            if packet_type == MQTT_PUBLISH:
                topic, offset = read_string(payload, 0)
                body = payload[offset:]
                if body:
                    print(f"[mqtt <-] {topic} {printable_payload(body)}", flush=True)
                else:
                    print(f"[mqtt <-] {topic} <empty>", flush=True)
                self.describe_downlink(body)
            elif packet_type == MQTT_PINGRESP:
                print("[mqtt] ping ok", flush=True)
            elif packet_type == MQTT_PUBACK:
                pass
            else:
                print(f"[mqtt] packet type={packet_type} flags={flags} payload={payload.hex()}", flush=True)

    async def ping(self) -> None:
        if self.writer is None:
            return
        self.writer.write(build_packet(MQTT_PINGREQ, 0, b""))
        await self.writer.drain()
        self.last_ping_s = time.monotonic()

    def describe_downlink(self, payload: bytes) -> None:
        if len(payload) == 21:
            print(format_grid_map(payload), flush=True)
            return

        if not payload:
            return
        if not is_probably_text_payload(payload):
            print(f"[server] binary payload length={len(payload)} hex={payload.hex()}", flush=True)
            return

        text = printable_payload(payload)
        fields = parse_key_values(text)
        message_type = fields.get("type")
        if message_type is None:
            return

        if message_type == "heartbeat":
            print(
                "[server] heartbeat "
                f"enable={fields.get('enable', '?')} seq={fields.get('seq', '?')} "
                f"time_left={fields.get('time_left', '?')}",
                flush=True,
            )
        elif message_type == "disable":
            print(
                "[server] disable "
                f"enabled={fields.get('enabled', '?')} reason={fields.get('reason', '')}",
                flush=True,
            )
        elif message_type == "isFertileReply":
            print(
                "[server] fertile "
                f"fertile={fields.get('fertile', '?')} planted={fields.get('planted', '?')} "
                f"x={fields.get('x', '?')} y={fields.get('y', '?')}",
                flush=True,
            )
        elif message_type == "emergency":
            print(f"[server] emergency enabled={fields.get('enabled', '?')}", flush=True)
        elif message_type == "distress":
            robots = " ".join(f"{key}={value}" for key, value in fields.items() if key.startswith("robot"))
            print(f"[server] distress count={fields.get('count', '?')} {robots}".rstrip(), flush=True)
        elif message_type == "openAirlockReply":
            print(
                "[server] airlock "
                f"airlock={fields.get('airlock', '?')} accepted={fields.get('accepted', '?')} "
                f"queue_enter={fields.get('queue_enter', '?')} queue_exit={fields.get('queue_exit', '?')} "
                f"reason={fields.get('reason', '')}",
                flush=True,
            )
        elif message_type == "reviveReply":
            print(
                "[server] revive "
                f"status={fields.get('status', '?')} target={fields.get('target', '?')}",
                flush=True,
            )

    async def close(self, send_offline: bool = True) -> None:
        if self.writer is None:
            return

        writer = self.writer
        try:
            if send_offline and self.connected:
                await self.publish(self.status_topic, b"offline", retain=True, qos=1)
            writer.write(build_packet(MQTT_DISCONNECT, 0, b""))
            await writer.drain()
        except (OSError, ConnectionError, RuntimeError):
            pass
        finally:
            self.connected = False
            self.reader = None
            self.writer = None
            try:
                writer.close()
                await writer.wait_closed()
            except (OSError, ConnectionError, RuntimeError):
                pass


class RobotTcpShell:
    def __init__(
        self,
        host: str | None,
        port: int,
        connect_timeout_s: float,
        auto_rfid: bool,
        rfid_cooldown_s: float,
    ) -> None:
        self.host = host
        self.port = port
        self.connect_timeout_s = connect_timeout_s
        self.auto_rfid = auto_rfid
        self.rfid_cooldown_s = rfid_cooldown_s
        self.reader: asyncio.StreamReader | None = None
        self.writer: asyncio.StreamWriter | None = None
        self.connected = False
        self.last_rfid_uid: str | None = None
        self.rfid_request_times: dict[str, float] = {}

    async def connect(self) -> None:
        if self.host is None:
            return
        try:
            self.reader, self.writer = await asyncio.wait_for(
                asyncio.open_connection(self.host, self.port),
                timeout=self.connect_timeout_s,
            )
            self.connected = True
            print(f"[robot] connected {self.host}:{self.port}", flush=True)
        except (TimeoutError, OSError) as exc:
            self.reader = None
            self.writer = None
            self.connected = False
            print(f"[robot] connect failed {self.host}:{self.port}: {exc}", flush=True)
            print("[robot] MQTT client is still running; use without robot commands or restart with the right --robot IP", flush=True)

    async def send(self, command: str) -> None:
        if self.writer is None:
            print("[robot] not connected", flush=True)
            return
        self.writer.write(command.encode("utf-8") + b"\n")
        await self.writer.drain()
        print(f"[robot ->] {command}", flush=True)

    async def loop(self, mqtt: MiniMessengerClient) -> None:
        if self.reader is None:
            return

        while True:
            raw = await self.reader.readline()
            if not raw:
                raise ConnectionError("robot TCP disconnected")
            text = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            if text:
                print(f"[robot <-] {text}", flush=True)
                await self.handle_robot_line(text, mqtt)

    async def handle_robot_line(self, text: str, mqtt: MiniMessengerClient) -> None:
        rfid_match = re.search(r"\brfid uid=(\d+) size=(\d+)", text)
        if rfid_match and rfid_match.group(1) != "0":
            self.last_rfid_uid = rfid_match.group(1)
            await self.handle_rfid_uid(self.last_rfid_uid, mqtt)

        done_match = re.search(r"\bmission done task=(\d+) .*elapsed=(\d+)ms", text)
        if done_match:
            await mqtt.publish_group(
                f"type=missionDone team_id={mqtt.group_id} board_id={mqtt.board_id} "
                f"task={done_match.group(1)} elapsed_ms={done_match.group(2)}"
            )

    async def handle_rfid_uid(self, uid: str, mqtt: MiniMessengerClient) -> None:
        if not self.auto_rfid:
            return

        now = time.monotonic()
        last_request = self.rfid_request_times.get(uid, 0.0)
        if now - last_request < self.rfid_cooldown_s:
            return

        self.rfid_request_times[uid] = now
        print(f"[bridge] auto RFID uid={uid}: isFertile + getMap", flush=True)
        await mqtt.is_fertile(uid)
        await mqtt.get_map()


async def stdin_loop(mqtt: MiniMessengerClient, robot: RobotTcpShell) -> None:
    print(
        "[cmd] <robot-shell-cmd> | robot <shell-cmd> | register | getmap | pose x y [task target_x target_y score seeds] | "
        "fertile <tag> | seed <tag> [x y] | airlock <A|B> <tag> | revive <team> <board> | mqtt <payload>",
        flush=True,
    )

    while True:
        line = await asyncio.to_thread(sys.stdin.readline)
        if line == "":
            await asyncio.sleep(0.2)
            continue

        command = line.strip()
        if not command:
            continue

        try:
            parts = command.split()
            name = parts[0].lower()

            if name in ("quit", "exit"):
                raise KeyboardInterrupt
            if name == "robot":
                await robot.send(command[len(parts[0]) :].strip())
            elif name == "task" and len(parts) >= 2:
                await robot.send(command)
            elif name == "register":
                await mqtt.register()
            elif name == "getmap":
                await mqtt.get_map()
            elif name == "pose" and len(parts) >= 3:
                values = [int(value) for value in parts[1:]]
                while len(values) < 7:
                    values.append(0)
                await mqtt.pose(*values[:7])
            elif name == "seed" and len(parts) >= 2:
                x = int(parts[2]) if len(parts) >= 3 else 0
                y = int(parts[3]) if len(parts) >= 4 else 0
                await mqtt.seed_planted(parts[1], x, y)
            elif name == "fertile" and len(parts) >= 2:
                await mqtt.is_fertile(parts[1])
            elif name == "airlock" and len(parts) >= 3:
                await mqtt.open_airlock(parts[1], parts[2])
            elif name == "revive" and len(parts) >= 3:
                await mqtt.revive_request(parts[1], parts[2])
            elif name == "mqtt":
                await mqtt.publish_group(command[len(parts[0]) :].strip())
            elif robot.connected:
                await robot.send(command)
            else:
                print("[cmd] unknown command", flush=True)
        except KeyboardInterrupt:
            raise
        except Exception as exc:
            print(f"[cmd] error: {exc}", flush=True)


async def main_async() -> None:
    parser = argparse.ArgumentParser(description="MiniMessenger-compatible PC MQTT bridge")
    parser.add_argument("--broker", default="192.168.0.74", help="MQTT broker host")
    parser.add_argument("--broker-port", type=int, default=1883, help="MQTT broker port")
    parser.add_argument("--broker-timeout", type=float, default=5.0, help="MQTT connect timeout in seconds")
    parser.add_argument("--group", default="12", help="MiniMessenger group/team id")
    parser.add_argument("--board", default="Terminator", help="MiniMessenger board id")
    parser.add_argument("--robot", default=None, help="Robot TCP shell host, e.g. 192.168.1.112")
    parser.add_argument("--robot-port", type=int, default=7777, help="Robot TCP shell port")
    parser.add_argument("--robot-timeout", type=float, default=3.0, help="Robot TCP connect timeout in seconds")
    parser.add_argument("--auto-rfid", action=argparse.BooleanOptionalAction, default=True, help="Auto-send isFertile/getMap for robot RFID logs")
    parser.add_argument("--rfid-cooldown", type=float, default=5.0, help="Minimum seconds before repeating the same RFID request")
    parser.add_argument("--register-interval", type=float, default=10.0, help="Register interval in seconds")
    args = parser.parse_args()

    mqtt = MiniMessengerClient(
        args.broker,
        args.broker_port,
        args.group,
        args.board,
        args.register_interval,
        args.broker_timeout,
    )
    robot = RobotTcpShell(
        args.robot,
        args.robot_port,
        args.robot_timeout,
        args.auto_rfid,
        args.rfid_cooldown,
    )

    loop = asyncio.get_running_loop()
    stop_event = asyncio.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop_event.set)

    while not mqtt.connected:
        try:
            await mqtt.connect()
        except (OSError, ConnectionError, asyncio.IncompleteReadError) as exc:
            print(f"[mqtt] initial connect failed: {exc}", flush=True)
            await asyncio.sleep(2.0)

    await robot.connect()

    tasks = [
        asyncio.create_task(mqtt.loop()),
        asyncio.create_task(stdin_loop(mqtt, robot)),
    ]
    if robot.connected:
        tasks.append(asyncio.create_task(robot.loop(mqtt)))

    stop_task = asyncio.create_task(stop_event.wait())
    done, pending = await asyncio.wait(tasks + [stop_task], return_when=asyncio.FIRST_COMPLETED)

    for task in pending:
        task.cancel()
    for task in done:
        exc = task.exception()
        if exc is not None:
            print(f"[bridge] stopped by error: {exc}", flush=True)

    await mqtt.close()


def main() -> None:
    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
