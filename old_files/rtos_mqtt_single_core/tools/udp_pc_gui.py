#!/usr/bin/env python3
"""
Simple Tkinter GUI for UDP communication with the robot.

Usage:
    python udp_pc_gui.py

Requirements:
    - Python 3.x
    - tkinter (usually bundled with Python; on Debian/Ubuntu install python3-tk if missing)
"""

import socket
import threading
import queue
import tkinter as tk
from tkinter import ttk, scrolledtext


class UdpGuiApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot UDP Monitor")
        self.root.geometry("750x500")
        self.root.resizable(False, False)

        self.sock = None
        self.recv_thread = None
        self.running = False
        self.msg_queue = queue.Queue()

        # Robot address (auto-detected from first packet)
        self.robot_addr = None

        self._build_ui()
        self.poll_queue()

    def _build_ui(self):
        # ===== Connection Frame =====
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding=5)
        conn_frame.pack(fill=tk.X, padx=10, pady=5)

        ttk.Label(conn_frame, text="Local Port:").grid(row=0, column=0, sticky=tk.W)
        self.entry_local_port = ttk.Entry(conn_frame, width=8)
        self.entry_local_port.insert(0, "8080")
        self.entry_local_port.grid(row=0, column=1, padx=5)

        ttk.Label(conn_frame, text="Target IP:").grid(row=0, column=2, sticky=tk.W, padx=(15, 0))
        self.entry_target_ip = ttk.Entry(conn_frame, width=15)
        self.entry_target_ip.insert(0, "192.168.1.211")
        self.entry_target_ip.grid(row=0, column=3, padx=5)

        ttk.Label(conn_frame, text="Target Port:").grid(row=0, column=4, sticky=tk.W)
        self.entry_target_port = ttk.Entry(conn_frame, width=8)
        self.entry_target_port.insert(0, "8080")
        self.entry_target_port.grid(row=0, column=5, padx=5)

        self.btn_start = ttk.Button(conn_frame, text="Start", command=self.start_server)
        self.btn_start.grid(row=0, column=6, padx=(15, 5))

        self.btn_stop = ttk.Button(conn_frame, text="Stop", command=self.stop_server, state=tk.DISABLED)
        self.btn_stop.grid(row=0, column=7)

        # ===== Sensor Data Frame =====
        sensor_frame = ttk.LabelFrame(self.root, text="Live Sensor Data", padding=10)
        sensor_frame.pack(fill=tk.X, padx=10, pady=5)

        self.sensor_labels = {}
        sensor_items = [
            ("robot_id", "Robot ID"),
            ("front",    "Front (cm)"),
            ("left",     "Left (cm)"),
            ("right",    "Right (cm)"),
            ("yaw",      "Yaw (°)"),
            ("mx",       "Mouse X"),
            ("my",       "Mouse Y"),
            ("ir",       "IR Pos"),
            ("btn",      "Mouse Btn"),
        ]

        for i, (key, name) in enumerate(sensor_items):
            r = i // 3
            c = (i % 3) * 2
            ttk.Label(sensor_frame, text=f"{name}:", font=("Arial", 10, "bold")).grid(row=r, column=c, sticky=tk.W, padx=5, pady=3)
            lbl = ttk.Label(sensor_frame, text="--", foreground="blue")
            lbl.grid(row=r, column=c + 1, sticky=tk.W, padx=5, pady=3)
            self.sensor_labels[key] = lbl

        # ===== Log Frame =====
        log_frame = ttk.LabelFrame(self.root, text="Message Log", padding=5)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        self.txt_log = scrolledtext.ScrolledText(log_frame, height=10, state=tk.DISABLED, wrap=tk.WORD)
        self.txt_log.pack(fill=tk.BOTH, expand=True)

        # ===== Send Frame =====
        send_frame = ttk.Frame(self.root, padding=5)
        send_frame.pack(fill=tk.X, padx=10, pady=5)

        ttk.Label(send_frame, text="Command:").pack(side=tk.LEFT)
        self.entry_send = ttk.Entry(send_frame)
        self.entry_send.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        self.entry_send.bind("<Return>", lambda e: self.send_command())
        self.btn_send = ttk.Button(send_frame, text="Send", command=self.send_command, state=tk.DISABLED)
        self.btn_send.pack(side=tk.LEFT)

        # ===== Status Bar =====
        self.status_var = tk.StringVar(value="Ready")
        status_bar = ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(fill=tk.X, side=tk.BOTTOM)

    def start_server(self):
        try:
            local_port = int(self.entry_local_port.get())
        except ValueError:
            self.append_log("[ERROR] Invalid local port\n")
            return

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.bind(("0.0.0.0", local_port))
        except OSError as e:
            self.append_log(f"[ERROR] Cannot bind port {local_port}: {e}\n")
            self.sock.close()
            self.sock = None
            return

        self.sock.settimeout(1.0)
        self.running = True
        self.recv_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.recv_thread.start()

        self.btn_start.config(state=tk.DISABLED)
        self.btn_stop.config(state=tk.NORMAL)
        self.btn_send.config(state=tk.NORMAL)
        self.status_var.set(f"Listening on 0.0.0.0:{local_port}")
        self.append_log(f"[*] UDP server started on port {local_port}\n")

    def stop_server(self):
        self.running = False
        if self.sock:
            self.sock.close()
            self.sock = None

        self.btn_start.config(state=tk.NORMAL)
        self.btn_stop.config(state=tk.DISABLED)
        self.btn_send.config(state=tk.DISABLED)
        self.status_var.set("Stopped")
        self.append_log("[*] UDP server stopped\n")

    def receive_loop(self):
        while self.running and self.sock:
            try:
                data, addr = self.sock.recvfrom(1024)
                msg = data.decode("utf-8", errors="ignore").strip()
                self.msg_queue.put(("rx", addr, msg))
            except socket.timeout:
                continue
            except OSError:
                break

    def poll_queue(self):
        try:
            while True:
                msg_type, addr, msg = self.msg_queue.get_nowait()
                if msg_type == "rx":
                    self.handle_packet(addr, msg)
        except queue.Empty:
            pass
        self.root.after(100, self.poll_queue)

    def handle_packet(self, addr, msg):
        parts = msg.split(",")
        if len(parts) >= 9:
            self.sensor_labels["robot_id"].config(text=parts[0])
            self.sensor_labels["front"].config(text=parts[1])
            self.sensor_labels["left"].config(text=parts[2])
            self.sensor_labels["right"].config(text=parts[3])
            self.sensor_labels["yaw"].config(text=parts[4])
            self.sensor_labels["mx"].config(text=parts[5])
            self.sensor_labels["my"].config(text=parts[6])
            self.sensor_labels["ir"].config(text=parts[7])
            self.sensor_labels["btn"].config(text=parts[8])

        # Auto-detect robot address for replies
        if self.robot_addr is None or self.robot_addr[0] != addr[0]:
            try:
                robot_port = int(self.entry_target_port.get())
            except ValueError:
                robot_port = addr[1]
            self.robot_addr = (addr[0], robot_port)
            self.status_var.set(f"Robot at {self.robot_addr[0]}:{self.robot_addr[1]}")
            self.append_log(f"[*] Robot detected: {self.robot_addr}\n")

        self.append_log(f"[RX] {addr}: {msg}\n")

    def send_command(self):
        if not self.sock or not self.robot_addr:
            self.append_log("[ERROR] No robot connected\n")
            return

        cmd = self.entry_send.get().strip()
        if not cmd:
            return

        try:
            self.sock.sendto(cmd.encode(), self.robot_addr)
            self.append_log(f"[TX] {self.robot_addr}: {cmd}\n")
            self.entry_send.delete(0, tk.END)
        except OSError as e:
            self.append_log(f"[ERROR] Send failed: {e}\n")

    def append_log(self, text):
        self.txt_log.config(state=tk.NORMAL)
        self.txt_log.insert(tk.END, text)
        self.txt_log.see(tk.END)
        self.txt_log.config(state=tk.DISABLED)


def main():
    root = tk.Tk()
    app = UdpGuiApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
