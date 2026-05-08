#!/usr/bin/env python3
"""
Serial Visualizer for space_robot_rev
Parses chassis and mouse data from Serial1 and visualizes them in real-time.

Dependencies:
    pip install pyserial matplotlib

Usage:
    python tools/serial_visualizer.py
"""

import re
import sys
import threading
import tkinter as tk
from collections import deque
from tkinter import messagebox, ttk

import matplotlib.pyplot as plt
import serial
import serial.tools.list_ports
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ------------------------------------------------------------------
# Config
# ------------------------------------------------------------------
MAXLEN = 200               # how many historical samples to keep
BAUD_DEFAULT = 115200      # match Serial1.begin(115200)
UPDATE_INTERVAL_MS = 50    # matplotlib animation interval

# Regex patterns
CHASSIS_RE = re.compile(
    r"chassis:\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)"
)
MOUSE_RE = re.compile(
    r"mouse:\s*([-\d.]+)\s+([-\d.]+)"
)


# ------------------------------------------------------------------
# GUI App
# ------------------------------------------------------------------
class SerialVisualizer(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Space Robot Serial Visualizer")
        self.geometry("1200x700")
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        # -- Serial --
        self.ser = None
        self.reader_thread = None
        self.running = False

        # -- Data buffers --
        self.t_buffer = deque(maxlen=MAXLEN)
        self.vfl_buffer = deque(maxlen=MAXLEN)
        self.vfr_buffer = deque(maxlen=MAXLEN)
        self.vrl_buffer = deque(maxlen=MAXLEN)
        self.vrr_buffer = deque(maxlen=MAXLEN)
        self.mouse_x_buffer = deque(maxlen=MAXLEN)
        self.mouse_y_buffer = deque(maxlen=MAXLEN)
        self.tick = 0

        self.lock = threading.Lock()

        self._build_ui()
        self._build_plots()

    # ------------------------------------------------------------------
    # UI Construction
    # ------------------------------------------------------------------
    def _build_ui(self):
        control_frame = ttk.Frame(self, padding=10)
        control_frame.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(control_frame, text="Port:").pack(side=tk.LEFT, padx=(0, 5))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(
            control_frame, textvariable=self.port_var, width=30, state="readonly"
        )
        self.port_combo.pack(side=tk.LEFT, padx=(0, 15))

        ttk.Label(control_frame, text="Baud:").pack(side=tk.LEFT, padx=(0, 5))
        self.baud_var = tk.StringVar(value=str(BAUD_DEFAULT))
        ttk.Entry(control_frame, textvariable=self.baud_var, width=10).pack(
            side=tk.LEFT, padx=(0, 15)
        )

        self.btn_refresh = ttk.Button(
            control_frame, text="Refresh", command=self.refresh_ports
        )
        self.btn_refresh.pack(side=tk.LEFT, padx=(0, 10))

        self.btn_connect = ttk.Button(
            control_frame, text="Connect", command=self.toggle_connection
        )
        self.btn_connect.pack(side=tk.LEFT, padx=(0, 10))

        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(control_frame, textvariable=self.status_var).pack(
            side=tk.LEFT, padx=(20, 0)
        )

        self.refresh_ports()

    def _build_plots(self):
        self.fig = plt.Figure(figsize=(12, 6), dpi=100)

        # Left: chassis wheel speeds (time series)
        self.ax_chassis = self.fig.add_subplot(1, 2, 1)
        self.ax_chassis.set_title("Chassis Wheel Speeds")
        self.ax_chassis.set_xlabel("Sample")
        self.ax_chassis.set_ylabel("Speed")
        self.ax_chassis.grid(True, linestyle="--", alpha=0.5)
        (self.line_vfl,) = self.ax_chassis.plot([], [], label="VFL", lw=1.5)
        (self.line_vfr,) = self.ax_chassis.plot([], [], label="VFR", lw=1.5)
        (self.line_vrl,) = self.ax_chassis.plot([], [], label="VRL", lw=1.5)
        (self.line_vrr,) = self.ax_chassis.plot([], [], label="VRR", lw=1.5)
        self.ax_chassis.legend(loc="upper left")

        # Right: mouse 2D trajectory
        self.ax_mouse = self.fig.add_subplot(1, 2, 2)
        self.ax_mouse.set_title("Mouse Trajectory")
        self.ax_mouse.set_xlabel("X")
        self.ax_mouse.set_ylabel("Y")
        self.ax_mouse.grid(True, linestyle="--", alpha=0.5)
        self.scatter_mouse = self.ax_mouse.scatter([], [], c=[], cmap="viridis", s=20)
        # colorbar will be attached once we have data
        self.cbar = None

        self.fig.tight_layout(pad=3.0)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.ani = FuncAnimation(
            self.fig,
            self.update_plot,
            interval=UPDATE_INTERVAL_MS,
            cache_frame_data=False,
        )

    # ------------------------------------------------------------------
    # Serial Handling
    # ------------------------------------------------------------------
    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        port_list = [f"{p.device} - {p.description}" for p in ports]
        self.port_combo["values"] = port_list
        if port_list:
            self.port_combo.current(0)

    def toggle_connection(self):
        if self.ser is not None and self.ser.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        raw = self.port_var.get()
        if not raw:
            messagebox.showwarning("No Port", "Please select a serial port.")
            return
        port = raw.split(" - ")[0]
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showerror("Bad Baud", "Baud rate must be an integer.")
            return

        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
        except serial.SerialException as e:
            messagebox.showerror("Serial Error", str(e))
            return

        self.running = True
        self.reader_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.reader_thread.start()

        self.status_var.set(f"Connected to {port} @ {baud}")
        self.btn_connect.config(text="Disconnect")

    def disconnect(self):
        self.running = False
        if self.reader_thread is not None:
            self.reader_thread.join(timeout=1.0)
        if self.ser is not None:
            self.ser.close()
            self.ser = None
        self.status_var.set("Disconnected")
        self.btn_connect.config(text="Connect")

    def read_loop(self):
        while self.running and self.ser is not None and self.ser.is_open:
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            except Exception:
                continue
            if not line:
                continue
            self.parse_line(line)

    def parse_line(self, line: str):
        chassis_match = CHASSIS_RE.search(line)
        mouse_match = MOUSE_RE.search(line)

        with self.lock:
            self.tick += 1
            t = self.tick

            if chassis_match:
                vfl, vfr, vrl, vrr = map(float, chassis_match.groups())
                self.t_buffer.append(t)
                self.vfl_buffer.append(vfl)
                self.vfr_buffer.append(vfr)
                self.vrl_buffer.append(vrl)
                self.vrr_buffer.append(vrr)

            if mouse_match:
                mx, my = map(float, mouse_match.groups())
                self.mouse_x_buffer.append(mx)
                self.mouse_y_buffer.append(my)

    # ------------------------------------------------------------------
    # Plot Update
    # ------------------------------------------------------------------
    def update_plot(self, _frame):
        with self.lock:
            # --- Chassis subplot ---
            if self.t_buffer:
                t = list(self.t_buffer)
                self.line_vfl.set_data(t, list(self.vfl_buffer))
                self.line_vfr.set_data(t, list(self.vfr_buffer))
                self.line_vrl.set_data(t, list(self.vrl_buffer))
                self.line_vrr.set_data(t, list(self.vrr_buffer))
                self.ax_chassis.set_xlim(t[0], max(t[-1], t[0] + 1))
                all_vals = (
                    list(self.vfl_buffer)
                    + list(self.vfr_buffer)
                    + list(self.vrl_buffer)
                    + list(self.vrr_buffer)
                )
                if all_vals:
                    ymin, ymax = min(all_vals), max(all_vals)
                    margin = max(abs(ymin), abs(ymax), 1.0) * 0.1
                    self.ax_chassis.set_ylim(ymin - margin, ymax + margin)

            # --- Mouse subplot ---
            if self.mouse_x_buffer:
                x = list(self.mouse_x_buffer)
                y = list(self.mouse_y_buffer)
                # use index as color to show time progression
                colors = list(range(len(x)))
                self.scatter_mouse.set_offsets(list(zip(x, y)))
                self.scatter_mouse.set_array(colors)
                self.scatter_mouse.set_clim(0, max(len(x), 1))

                # auto-scale axes
                x_min, x_max = min(x), max(x)
                y_min, y_max = min(y), max(y)
                x_margin = max(abs(x_min), abs(x_max), 1.0) * 0.15
                y_margin = max(abs(y_min), abs(y_max), 1.0) * 0.15
                self.ax_mouse.set_xlim(x_min - x_margin, x_max + x_margin)
                self.ax_mouse.set_ylim(y_min - y_margin, y_max + y_margin)

                if self.cbar is None:
                    self.cbar = self.fig.colorbar(
                        self.scatter_mouse, ax=self.ax_mouse, label="Sample #"
                    )

        self.canvas.draw_idle()
        return (
            self.line_vfl,
            self.line_vfr,
            self.line_vrl,
            self.line_vrr,
            self.scatter_mouse,
        )

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def on_close(self):
        self.disconnect()
        self.ani.event_source.stop()
        plt.close(self.fig)
        self.destroy()
        sys.exit(0)


# ------------------------------------------------------------------
if __name__ == "__main__":
    app = SerialVisualizer()
    app.mainloop()
