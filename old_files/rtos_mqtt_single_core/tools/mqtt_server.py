#!/usr/bin/env python3
"""Small MQTT 3.1.1 broker for local MiniMessenger testing.

Run on the PC that shares WiFi/LAN with the robot:
    python3 tools/mqtt_server.py --host 0.0.0.0 --port 1883

Then set CONFIG::MQTT_BROKER_HOST on the robot to this PC's LAN IP.
"""

from __future__ import annotations

import argparse
import asyncio
import ipaddress
import traceback
import socket
from dataclasses import dataclass, field


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


def read_u16(data: bytes, offset: int) -> tuple[int, int]:
    return int.from_bytes(data[offset : offset + 2], "big"), offset + 2


def read_string(data: bytes, offset: int) -> tuple[str, int]:
    length, offset = read_u16(data, offset)
    end = offset + length
    return data[offset:end].decode("utf-8", errors="replace"), end


def topic_matches(filter_topic: str, topic: str) -> bool:
    filter_parts = filter_topic.split("/")
    topic_parts = topic.split("/")

    for index, part in enumerate(filter_parts):
        if part == "#":
            return index == len(filter_parts) - 1
        if index >= len(topic_parts):
            return False
        if part != "+" and part != topic_parts[index]:
            return False

    return len(topic_parts) == len(filter_parts)


def local_ipv4_addresses() -> list[str]:
    addresses: set[str] = set()
    hostname = socket.gethostname()

    try:
        for info in socket.getaddrinfo(hostname, None, socket.AF_INET):
            addresses.add(info[4][0])
    except socket.gaierror:
        pass

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.connect(("8.8.8.8", 80))
            addresses.add(probe.getsockname()[0])
    except OSError:
        pass

    return sorted(
        address
        for address in addresses
        if not ipaddress.ip_address(address).is_loopback
    )


@dataclass
class Subscription:
    topic_filter: str
    qos: int


@dataclass
class Client:
    reader: asyncio.StreamReader
    writer: asyncio.StreamWriter
    address: str
    write_lock: asyncio.Lock = field(default_factory=asyncio.Lock)
    client_id: str = ""
    subscriptions: list[Subscription] = field(default_factory=list)
    will_topic: str | None = None
    will_payload: bytes = b""
    will_retain: bool = False
    keep_alive_s: int = 60
    disconnected_cleanly: bool = False


class MiniBroker:
    def __init__(self) -> None:
        self.clients: list[Client] = []
        self.retained: dict[str, bytes] = {}
        self._next_packet_id = 1
        self._heartbeat_seq = 0

    async def handle_client(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        peer = writer.get_extra_info("peername")
        address = f"{peer[0]}:{peer[1]}" if peer else "unknown"
        client = Client(reader=reader, writer=writer, address=address)
        self.clients.append(client)
        print(f"[connect] tcp {address}", flush=True)

        try:
            while True:
                packet_type, flags, payload = await self.read_packet(client)
                print(
                    f"[packet] {client.client_id or address} type={packet_type} flags={flags} bytes={len(payload)}",
                    flush=True,
                )
                if packet_type == MQTT_CONNECT:
                    await self.handle_connect(client, payload)
                elif packet_type == MQTT_SUBSCRIBE:
                    await self.handle_subscribe(client, payload)
                elif packet_type == MQTT_PUBLISH:
                    await self.handle_publish(client, flags, payload)
                elif packet_type == MQTT_PINGREQ:
                    await self.write_packet(client, MQTT_PINGRESP, 0, b"")
                elif packet_type == MQTT_DISCONNECT:
                    client.disconnected_cleanly = True
                    break
                else:
                    print(f"[warn] unsupported packet type={packet_type} from {client.client_id or address}", flush=True)
                    break
        except asyncio.TimeoutError:
            print(f"[timeout] waiting for MQTT packet from {client.client_id or address}", flush=True)
        except (asyncio.IncompleteReadError, ConnectionError, OSError) as exc:
            print(f"[close] {client.client_id or address}: {exc}", flush=True)
        except Exception:
            print(f"[error] {client.client_id or address}", flush=True)
            traceback.print_exc()
        finally:
            await self.close_client(client)

    async def read_packet(self, client: Client) -> tuple[int, int, bytes]:
        timeout_s = max(15, int(client.keep_alive_s * 1.5))
        reader = client.reader

        fixed = (await asyncio.wait_for(reader.readexactly(1), timeout=timeout_s))[0]
        remaining_length = 0
        multiplier = 1

        while True:
            byte = (await asyncio.wait_for(reader.readexactly(1), timeout=timeout_s))[0]
            remaining_length += (byte & 0x7F) * multiplier
            if (byte & 0x80) == 0:
                break
            multiplier *= 128
            if multiplier > 128 * 128 * 128:
                raise ConnectionError("malformed remaining length")

        payload = await asyncio.wait_for(reader.readexactly(remaining_length), timeout=timeout_s)
        return fixed >> 4, fixed & 0x0F, payload

    async def write_packet(
        self, client: Client, packet_type: int, flags: int, payload: bytes
    ) -> None:
        async with client.write_lock:
            client.writer.write(bytes([(packet_type << 4) | flags]))
            client.writer.write(encode_remaining_length(len(payload)))
            client.writer.write(payload)
            await client.writer.drain()

    async def handle_connect(self, client: Client, payload: bytes) -> None:
        protocol, offset = read_string(payload, 0)
        level = payload[offset]
        flags = payload[offset + 1]
        keep_alive, _ = read_u16(payload, offset + 2)
        client.keep_alive_s = keep_alive or 60
        offset += 4

        client_id, offset = read_string(payload, offset)
        client.client_id = client_id or f"anonymous-{client.address}"

        print(
            f"[connect-packet] protocol={protocol} level={level} flags=0x{flags:02x} keepalive={client.keep_alive_s}s client_id={client.client_id}",
            flush=True,
        )

        will_flag = bool(flags & 0x04)
        client.will_retain = bool(flags & 0x20)
        if will_flag:
            client.will_topic, offset = read_string(payload, offset)
            will_len, offset = read_u16(payload, offset)
            client.will_payload = payload[offset : offset + will_len]
            offset += will_len
            print(
                f"[will] {client.client_id} topic={client.will_topic} bytes={len(client.will_payload)} retain={client.will_retain}",
                flush=True,
            )

        if protocol != "MQTT" or level != 4:
            await self.write_packet(client, MQTT_CONNACK, 0, b"\x00\x01")
            raise ConnectionError("unsupported MQTT protocol")

        await self.drop_existing_client_id(client)
        await self.write_packet(client, MQTT_CONNACK, 0, b"\x00\x00")
        print(f"[mqtt] {client.client_id} connected from {client.address}", flush=True)

    async def drop_existing_client_id(self, client: Client) -> None:
        for existing in list(self.clients):
            if existing is client:
                continue
            if existing.client_id == client.client_id:
                print(
                    f"[replace] closing old {existing.client_id} from {existing.address}",
                    flush=True,
                )
                existing.disconnected_cleanly = True
                existing.writer.close()
                try:
                    await existing.writer.wait_closed()
                except OSError:
                    pass
                if existing in self.clients:
                    self.clients.remove(existing)

    async def handle_subscribe(self, client: Client, payload: bytes) -> None:
        packet_id, offset = read_u16(payload, 0)
        granted_qos = bytearray()

        while offset < len(payload):
            topic_filter, offset = read_string(payload, offset)
            requested_qos = payload[offset] & 0x03
            offset += 1
            granted = 1 if requested_qos else 0
            client.subscriptions.append(Subscription(topic_filter, granted))
            granted_qos.append(granted)
            print(f"[sub] {client.client_id} -> {topic_filter} qos={granted}", flush=True)

            for topic, retained_payload in self.retained.items():
                if topic_matches(topic_filter, topic):
                    await self.send_publish(client, topic, retained_payload, retain=True, qos=0)

        await self.write_packet(client, MQTT_SUBACK, 0, packet_id.to_bytes(2, "big") + bytes(granted_qos))
        print(f"[suback] {client.client_id} packet_id={packet_id} granted={list(granted_qos)}", flush=True)

    async def handle_publish(self, client: Client, flags: int, payload: bytes) -> None:
        qos = (flags >> 1) & 0x03
        retain = bool(flags & 0x01)
        topic, offset = read_string(payload, 0)
        packet_id = None
        if qos:
            packet_id, offset = read_u16(payload, offset)

        message = payload[offset:]
        text = message.decode("utf-8", errors="replace")
        print(f"[pub] {client.client_id or client.address} {topic}: {text}", flush=True)

        if retain:
            if message:
                self.retained[topic] = message
            else:
                self.retained.pop(topic, None)

        await self.publish(topic, message, retain=retain, sender=client)

        if qos == 1 and packet_id is not None:
            await self.write_packet(client, MQTT_PUBACK, 0, packet_id.to_bytes(2, "big"))
            print(f"[puback] {client.client_id} packet_id={packet_id}", flush=True)

    async def publish(
        self, topic: str, payload: bytes, retain: bool = False, sender: Client | None = None
    ) -> None:
        for client in list(self.clients):
            if client.writer.is_closing():
                continue
            for subscription in client.subscriptions:
                if topic_matches(subscription.topic_filter, topic):
                    await self.send_publish(client, topic, payload, retain=retain, qos=0)
                    break

    async def send_publish(
        self, client: Client, topic: str, payload: bytes, retain: bool, qos: int
    ) -> None:
        flags = 0x01 if retain else 0
        packet_id = b""
        if qos:
            flags |= qos << 1
            packet_id = self.next_packet_id().to_bytes(2, "big")
        await self.write_packet(
            client,
            MQTT_PUBLISH,
            flags,
            encode_string(topic) + packet_id + payload,
        )

    def next_packet_id(self) -> int:
        packet_id = self._next_packet_id
        self._next_packet_id += 1
        if self._next_packet_id > 0xFFFF:
            self._next_packet_id = 1
        return packet_id

    async def close_client(self, client: Client) -> None:
        if client in self.clients:
            self.clients.remove(client)

        if (
            not client.disconnected_cleanly
            and client.will_topic
            and client.will_payload
        ):
            await self.publish(
                client.will_topic,
                client.will_payload,
                retain=client.will_retain,
                sender=client,
            )

        client.writer.close()
        try:
            await client.writer.wait_closed()
        except OSError:
            pass

        print(f"[disconnect] {client.client_id or client.address}", flush=True)

    async def heartbeat_loop(
        self,
        group_id: str,
        target: str,
        interval_s: float,
        enabled: bool,
    ) -> None:
        topic = f"lab/g/{group_id}/from/server/to/{target}"
        log_every = max(1, int(round(5.0 / interval_s)))

        while True:
            self._heartbeat_seq += 1
            payload = (
                f"type=heartbeat enable={1 if enabled else 0} "
                f"seq={self._heartbeat_seq}"
            ).encode("utf-8")
            await self.publish(topic, payload, retain=False)

            if self._heartbeat_seq % log_every == 0:
                print(
                    f"[heartbeat] {topic}: {payload.decode('utf-8')}",
                    flush=True,
                )

            await asyncio.sleep(interval_s)


async def main() -> None:
    parser = argparse.ArgumentParser(description="Tiny MQTT broker for robot testing")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--group", default="12", help="Group ID for server heartbeat topics")
    parser.add_argument(
        "--heartbeat-target",
        default="all",
        help="Heartbeat target board ID, or 'all' for group broadcast",
    )
    parser.add_argument(
        "--heartbeat-interval",
        type=float,
        default=0.2,
        help="Heartbeat interval in seconds",
    )
    parser.add_argument(
        "--heartbeat-enable",
        type=int,
        default=1,
        choices=(0, 1),
        help="Heartbeat enable value sent to robots",
    )
    parser.add_argument(
        "--no-heartbeat",
        action="store_true",
        help="Disable automatic heartbeat publishing",
    )
    args = parser.parse_args()

    broker = MiniBroker()
    server = await asyncio.start_server(broker.handle_client, args.host, args.port)

    print(f"MQTT broker listening on {args.host}:{args.port}", flush=True)
    for address in local_ipv4_addresses():
        print(f"PC LAN IP candidate: {address}", flush=True)
    print("Set CONFIG::MQTT_BROKER_HOST to the PC LAN IP if needed.", flush=True)
    if not args.no_heartbeat:
        print(
            "Heartbeat enabled: "
            f"topic=lab/g/{args.group}/from/server/to/{args.heartbeat_target} "
            f"interval={args.heartbeat_interval}s enable={args.heartbeat_enable}",
            flush=True,
        )
        asyncio.create_task(
            broker.heartbeat_loop(
                group_id=args.group,
                target=args.heartbeat_target,
                interval_s=args.heartbeat_interval,
                enabled=bool(args.heartbeat_enable),
            )
        )

    async with server:
        await server.serve_forever()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nMQTT broker stopped", flush=True)
