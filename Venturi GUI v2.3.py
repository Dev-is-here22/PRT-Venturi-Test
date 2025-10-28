import os
import csv
import threading
import queue
from collections import deque
import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

APP_TITLE = "Venturi Test"
BAUD = 2_000_000
MAX_POINTS = 2000
READ_TIMEOUT = 0.1

CSV_HEADER = [
    "Pressure1(bar)","Pressure2(bar)","Pressure3(bar)","Pressure4(bar)",
    "P4/P3","HallLiters(L)","USLiters(L)","USFlowRate(L/s)","Temperature(°C)","Time(µs)"
]

def unique_csv_name(prefix="TestGUI", ext=".csv"):
    i = 1
    while True:
        name = f"{prefix}_{i}{ext}"
        try:
            with open(name, "x"):
                pass
            return name
        except FileExistsError:
            i += 1

class SerialReader(threading.Thread):
    """Reads lines from an already-open pyserial.Serial and pushes them to a queue."""
    def __init__(self, ser: serial.Serial, line_queue: queue.Queue, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.ser = ser
        self.q = line_queue
        self.stop_event = stop_event

    def run(self):
        buf = bytearray()
        try:
            while not self.stop_event.is_set():
                data = self.ser.read(1024)
                if not data:
                    continue
                buf.extend(data)
                while True:
                    idx = buf.find(b"\n")
                    if idx == -1:
                        break
                    line = buf[:idx].decode(errors="ignore").strip("\r")
                    del buf[:idx+1]
                    self.q.put(("__LINE__", line))
        except Exception as e:
            self.q.put(("__ERR__", f"Serial read error: {e}"))

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(APP_TITLE)
        self.geometry("1200x800")

        # Serial state
        self.ser: serial.Serial | None = None
        self.reader_thread: SerialReader | None = None
        self.reader_stop = threading.Event()
        self.line_q: queue.Queue = queue.Queue()

        # CSV state
        self.csv_file = None
        self.csv_writer = None
        self.is_logging = False

        # Data buffers
        self.t_sec = deque(maxlen=MAX_POINTS)
        self.p1 = deque(maxlen=MAX_POINTS)
        self.p2 = deque(maxlen=MAX_POINTS)
        self.p3 = deque(maxlen=MAX_POINTS)
        self.p4 = deque(maxlen=MAX_POINTS)
        self.ratio = deque(maxlen=MAX_POINTS)
        self.hallL = deque(maxlen=MAX_POINTS)
        self.usFlow = deque(maxlen=MAX_POINTS)

        self.build_controls()
        self.build_plots()
        self.build_console()

        self.after(50, self.process_queue)
        self.after(100, self.update_plots)

    # ---------- UI ----------
    def build_controls(self):
        frm = ttk.Frame(self, padding=8)
        frm.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(frm, text="Port:").pack(side=tk.LEFT)
        self.port_cmb = ttk.Combobox(frm, width=40, state="readonly", values=self.list_ports())
        self.port_cmb.pack(side=tk.LEFT, padx=4)
        if self.port_cmb["values"]:
            self.port_cmb.current(0)

        ttk.Button(frm, text="Refresh", command=self.refresh_ports).pack(side=tk.LEFT, padx=4)
        ttk.Button(frm, text="Connect", command=self.connect).pack(side=tk.LEFT, padx=4)
        ttk.Button(frm, text="Disconnect", command=self.disconnect).pack(side=tk.LEFT, padx=4)

        ttk.Separator(frm, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)

        ttk.Button(frm, text="Start", command=lambda: self.send_char('1')).pack(side=tk.LEFT, padx=4)
        ttk.Button(frm, text="Stop", command=lambda: self.send_char('2')).pack(side=tk.LEFT, padx=4)
        ttk.Button(frm, text="Calibrate", command=lambda: self.send_char('3')).pack(side=tk.LEFT, padx=4)

        ttk.Separator(frm, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)

        self.log_btn = ttk.Button(frm, text="Start Logging", command=self.toggle_logging)
        self.log_btn.pack(side=tk.LEFT, padx=4)

        self.file_lbl = ttk.Label(frm, text="No file")
        self.file_lbl.pack(side=tk.LEFT, padx=12)

        self.status_var = tk.StringVar(value="Idle")
        ttk.Label(frm, textvariable=self.status_var).pack(side=tk.RIGHT)

    def build_plots(self):
        self.fig = Figure(figsize=(10, 6), dpi=100)
        self.ax_press = self.fig.add_subplot(3, 2, (1,2))
        self.ax_ratio = self.fig.add_subplot(3, 2, (3,4))
        self.ax_flow  = self.fig.add_subplot(3, 2, 5)
        self.ax_liter = self.fig.add_subplot(3, 2, 6)

        self.ax_press.set_ylabel("Pressure (bar)")
        self.ax_ratio.set_ylabel("P4/P3")
        self.ax_flow.set_ylabel("Flow")
        self.ax_flow.set_xlabel("Time (s)")
        self.ax_liter.set_ylabel("Cumulative flow (L)")
        self.ax_liter.set_xlabel("Time (s)")

        # Fixed axis limits
        self.ax_press.set_ylim(0, 12)
        self.ax_ratio.set_ylim(0, 1.2)

        self.line_p1, = self.ax_press.plot([], [], label="P1")
        self.line_p2, = self.ax_press.plot([], [], label="P2")
        self.line_p3, = self.ax_press.plot([], [], label="P3")
        self.line_p4, = self.ax_press.plot([], [], label="P4")
        self.ax_press.legend(loc="upper right")

        self.line_ratio, = self.ax_ratio.plot([], [], label="Ratio (P4/P3)")
        self.ax_ratio.axhline(1.0, color='gray', linestyle='--', linewidth=1)
        self.ax_ratio.legend(loc="upper right")

        self.line_us, = self.ax_flow.plot([], [], label="US flow (L/s)")
        self.ax_flow.legend(loc="upper right")

        self.line_hall, = self.ax_liter.plot([], [], label="Hall liters")
        self.ax_liter.legend(loc="upper right")

        canvas = FigureCanvasTkAgg(self.fig, master=self)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas = canvas

    def build_console(self):
        frm = ttk.Frame(self)
        frm.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=False)
        ttk.Label(frm, text="Serial Console:").pack(anchor="w")
        self.console = tk.Text(frm, height=8, wrap="word", bg="#111", fg="#eee")
        self.console.pack(fill=tk.BOTH, expand=True)

    def log_console(self, text):
        self.console.insert(tk.END, text + "\n")
        self.console.see(tk.END)

    # ---------- Serial helpers ----------
    def list_ports(self):
        # Prefer both /dev/tty.* and /dev/cu.* on macOS (some boards appear on one)
        ports = [p.device for p in serial.tools.list_ports.comports()]
        # De-dup keeping order
        seen = set()
        uniq = []
        for p in ports:
            if p not in seen:
                uniq.append(p); seen.add(p)
        return uniq

    def refresh_ports(self):
        self.port_cmb["values"] = self.list_ports()
        if self.port_cmb["values"]:
            self.port_cmb.current(0)

    def connect(self):
        if self.ser and self.ser.is_open:
            self.status("Already connected.")
            return

        port = self.port_cmb.get()
        if not port:
            messagebox.showwarning(APP_TITLE, "Select a serial port first.")
            return

        # Try sequence: cu -> tty sibling -> cu without exclusive -> tty without exclusive
        def sibling(p):
            # swap cu. <-> tty.
            if "/cu." in p:
                return p.replace("/cu.", "/tty.")
            if "/tty." in p:
                return p.replace("/tty.", "/cu.")
            return p

        tried = []
        def try_open(dev, exclusive_flag):
            tried.append(f"{dev} (exclusive={exclusive_flag})")
            try:
                s = serial.Serial(
                    port=dev,
                    baudrate=BAUD,
                    timeout=READ_TIMEOUT,
                    write_timeout=0.5,
                    # exclusive may raise EBUSY if any other process has TIOCEXCL
                    exclusive=exclusive_flag
                )
                # Make sure Arduino unblocks `while (!Serial);`
                s.dtr = True
                s.rts = False
                s.reset_input_buffer()
                s.reset_output_buffer()
                return s
            except Exception as e:
                # Save last error and rethrow
                raise e

        candidates = [
            (port, True),
            (sibling(port), True),
            (port, False),
            (sibling(port), False),
        ]

        last_err = None
        for dev, ex in candidates:
            try:
                self.ser = try_open(dev, ex)
                break
            except Exception as e:
                last_err = e
                self.ser = None
                # If EBUSY on mac/linux, try to reveal the culprit
                try:
                    import platform, subprocess, shlex
                    if platform.system() in ("Darwin", "Linux"):
                        # Pick the device we just attempted
                        dev_to_check = dev
                        # Use lsof to show who owns it (best-effort)
                        cmd = f"lsof {shlex.quote(dev_to_check)}"
                        out = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=1.5)
                        if out.stdout.strip():
                            self.status(f"BUSY: {dev_to_check} in use\n{out.stdout.splitlines()[0]}")
                        else:
                            self.status(f"Attempt failed on {dev_to_check}: {e}")
                    else:
                        self.status(f"Attempt failed on {dev}: {e}")
                except Exception:
                    pass
                continue

        if not self.ser:
            msg = f"Failed to open after tries:\n- " + "\n- ".join(tried) + f"\n\nLast error:\n{last_err}"
            self.status("Connect failed")
            messagebox.showerror(APP_TITLE, msg)
            return

        # Start reader thread with the open handle
        self.reader_stop.clear()
        self.reader_thread = SerialReader(self.ser, self.line_q, self.reader_stop)
        self.reader_thread.start()
        self.status(f"Connected to {self.ser.port}")

    def disconnect(self):
        self.reader_stop.set()
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
        self.reader_thread = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.status("Disconnected")

    def status(self, text: str):
        self.status_var.set(text)

    # ---------- Commands ----------
    def send_char(self, ch: str):
        if not (self.ser and self.ser.is_open):
            self.status("Not connected.")
            return
        try:
            self.ser.write(ch.encode("ascii"))
            self.status(f"Sent '{ch}'")
        except Exception as e:
            self.status(f"Send error: {e}")

    # ---------- Logging ----------
    def toggle_logging(self):
        if not self.is_logging:
            save_dir = os.path.expanduser("~/Documents/ArduinoLogs")
            os.makedirs(save_dir, exist_ok=True)
            name = os.path.join(save_dir, unique_csv_name())
            try:
                self.csv_file = open(name, "w", newline="")
                self.csv_writer = csv.writer(self.csv_file)
                self.csv_writer.writerow(CSV_HEADER)
                self.file_lbl.config(text=name)
                self.is_logging = True
                self.log_btn.config(text="Stop Logging")
                self.status(f"Logging to {name}")
            except Exception as e:
                messagebox.showerror(APP_TITLE, f"Cannot open file: {e}")
        else:
            self.stop_logging()

    def stop_logging(self):
        if self.csv_file:
            try:
                self.csv_file.flush()
                self.csv_file.close()
            except Exception:
                pass
        self.csv_file = None
        self.csv_writer = None
        self.is_logging = False
        self.log_btn.config(text="Start Logging")
        self.status("Logging stopped")

    # ---------- Parsing ----------
    def parse_line(self, line: str):
        parts = [p.strip() for p in line.split(",")]
        if parts and parts[-1] == "":
            parts = parts[:-1]
        if len(parts) != 10:
            return None
        try:
            p1 = float(parts[0]); p2 = float(parts[1]); p3 = float(parts[2]); p4 = float(parts[3])
            ratio = float(parts[4])
            hallL = float(parts[5])
            usLit = float(parts[6])
            usFlow = float(parts[7])
            tempC = float(parts[8])
            t_us = float(parts[9])
        except ValueError:
            return None
        t_sec = t_us / 1e6
        return (p1, p2, p3, p4, ratio, hallL, usLit, usFlow, tempC, t_sec)

    # ---------- Queues & plots ----------
    def process_queue(self):
        try:
            while True:
                tag, payload = self.line_q.get_nowait()
                if tag == "__LINE__":
                    print(payload)
                    self.log_console(payload)
                    data = self.parse_line(payload)
                    if data:
                        p1, p2, p3, p4, ratio, hallL, usLit, usFlow, tempC, t_sec = data
                    
                        # NEW: if Arduino time reset (e.g., after Start), clear the plots
                        if self.t_sec and t_sec < self.t_sec[-1] - 0.2:  # small tolerance
                            self._reset_series()
                        self.t_sec.append(t_sec)
                        self.p1.append(p1); self.p2.append(p2); self.p3.append(p3); self.p4.append(p4)
                        self.ratio.append(ratio)
                        self.hallL.append(hallL)
                        self.usFlow.append(usFlow)
                        if self.is_logging and self.csv_writer:
                            self.csv_writer.writerow([
                                f"{p1:.2f}", f"{p2:.2f}", f"{p3:.2f}", f"{p4:.2f}",
                                f"{ratio:.3f}", f"{hallL:.3f}", f"{usLit:.3f}",
                                f"{usFlow:.3f}", f"{tempC:.2f}", f"{int(t_sec*1e6)}"
                            ])
                elif tag == "__ERR__":
                    self.status(payload)
        except queue.Empty:
            pass
        self.after(50, self.process_queue)

    def update_plots(self):
        if self.t_sec:
            x = list(self.t_sec)

            # Pressures
            self.line_p1.set_data(x, list(self.p1))
            self.line_p2.set_data(x, list(self.p2))
            self.line_p3.set_data(x, list(self.p3))
            self.line_p4.set_data(x, list(self.p4))
            #self.rescale_axis(self.ax_press, x, [self.p1, self.p2, self.p3, self.p4])

            # Ratio
            self.line_ratio.set_data(x, list(self.ratio))
            #self.rescale_axis(self.ax_ratio, x, [self.ratio])

            # Flow
            self.line_us.set_data(x, list(self.usFlow))
            self.rescale_axis(self.ax_flow, x, [self.usFlow])

            #Liter
            self.line_hall.set_data(x, list(self.hallL))
            self.rescale_axis(self.ax_liter, x, [self.hallL])

            self.canvas.draw_idle()
        self.after(100, self.update_plots)

    @staticmethod
    def rescale_axis(ax, x, series_list):
        if not x:
            return
        ax.set_xlim(x[0], x[-1])
        ymin = None; ymax = None
        for s in series_list:
            if not s:
                continue
            smin = min(s); smax = max(s)
            ymin = smin if ymin is None else min(ymin, smin)
            ymax = smax if ymax is None else max(ymax, smax)
        if ymin is None or ymax is None:
            return
        if ymin == ymax:
            pad = 1.0 if ymin == 0 else abs(ymin) * 0.1
            ymin -= pad; ymax += pad
        else:
            rng = ymax - ymin
            ymin -= 0.05*rng; ymax += 0.05*rng
        ax.set_ylim(ymin, ymax)

    def on_close(self):
        try:
            self.disconnect()
            self.stop_logging()
        finally:
            self.destroy()
    def _reset_series(self):
        # wipe plot buffers so a new run starts clean
        self.t_sec.clear()
        self.p1.clear(); self.p2.clear(); self.p3.clear(); self.p4.clear()
        self.ratio.clear()
        self.hallL.clear()
        self.usFlow.clear()

def main():
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()

if __name__ == "__main__":
    main()