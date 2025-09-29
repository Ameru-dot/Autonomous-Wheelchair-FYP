#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Wheelchair GUI & Simulation — Single File
=========================================

Purpose
-------
A **standalone** Python program you can run on a laptop **without Arduino/wheelchair**
that shows the **GUI** and runs **simulated Line Following** and **Object Tracking**
workers. It keeps the exact command protocol you described (e.g. `$0040-007`).

Later, to use on Jetson with the real Arduino+MDD10A and cameras, flip a few
flags in the CONFIG dict below.

Quick Start (Laptop Simulation)
-------------------------------
1) Python 3.9+ recommended
2) Install deps:  
   `pip install opencv-python numpy pyserial`
3) Run:  
   `python wheelchair_sim.py`

Controls
--------
- Mode: **Manual / Line Follow / Object Track** via radio buttons
- Manual presets: **W/A/S/D** or GUI arrows; **Space** to stop
- Speed ± buttons adjust manual speed (0..255)
- Preview windows (OpenCV) show the simulated camera feed for each worker

Notes
-----
- The program uses **threads** for workers to keep it portable on Windows/Linux/Mac.
- Serial is **simulated by default** (prints frames like `$0040-007`).
- To use real serial/cameras later, set the flags in CONFIG accordingly.
"""

import math
import sys
import time
import threading
import queue
from dataclasses import dataclass

try:
    import tkinter as tk
    from tkinter import ttk
except Exception as e:  # pragma: no cover
    print("[FATAL] Tkinter not available:", e)
    sys.exit(1)

import numpy as np
import cv2

# ------------------------------
# Configuration (edit as needed)
# ------------------------------
CONFIG = {
    # Simulation toggles (for laptop)
    "simulate": True,           # True = use synthetic frames, False = use real cameras
    "show_preview": True,       # True = show OpenCV preview windows per worker

    # Real hardware toggles (for Jetson later)
    "use_real_serial": False,   # True = open real serial port
    "serial_port": "/dev/ttyUSB0" if sys.platform != "win32" else "COM3",
    "baudrate": 9600,

    # Cameras (used only when simulate=False)
    "cam_line": 0,
    "cam_object": 1,

    # PID for line following
    "pid": {"Kp": 0.18, "Ki": 0.00, "Kd": 0.06, "dead_zone_px": 10, "clip": 15},

    # GUI
    "gui": {"window_title": "Wheelchair Controller", "base_speed": 120},
}

# ------------------------------
# Protocol (Jetson side)
# ------------------------------
SET_TURN      = 40  # value: -15..+15 (autonomous steering)
SET_BOTHSPEED = 60  # value: -100..100
SET_LSPEED    = 61  # value: -100..100
SET_RSPEED    = 62  # value: -100..100
DRIVE_PRESET  = 70  # packed: preset*1000 + speed  (preset: 0 stop,1 fwd,2 back,3 left,4 right)
HEARTBEAT     = 90
STOP_ALL      = 99

DIGITS = 4

def pack(code: int, value: int, digits: int = DIGITS) -> bytes:
    """Frame as bytes, e.g. code=40, value=-7 -> b"$0040-007""" 
    return ("$" + str(int(code)).zfill(digits) + str(int(value)).zfill(digits)).encode()

# ------------------------------
# Serial transport (simulated or real)
# ------------------------------
class TransportBase:
    def send(self, code: int, value: int):
        raise NotImplementedError

class DummySerialLink(TransportBase):
    def __init__(self, on_tx=None):
        self.on_tx = on_tx  # callback for UI status
    def send(self, code: int, value: int):
        frame = pack(code, value)
        msg = frame.decode("ascii", errors="ignore")
        print("[TX]", msg)
        if self.on_tx:
            self.on_tx(msg)

class RealSerialLink(TransportBase):
    def __init__(self, port: str, baud: int, on_tx=None):
        import serial  # type: ignore
        import serial.tools.list_ports as list_ports  # type: ignore
        self.ser = None
        self.port = port
        self.baud = baud
        self.on_tx = on_tx
        # Try best-effort to open
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(2.0)  # Arduino auto-reset
            print(f"[Serial] Connected {self.port} @ {self.baud}")
        except Exception as e:
            print(f"[Serial] Failed to open {self.port}: {e}\n         Falling back to DummySerialLink.")
            self.ser = None
            self.dummy = DummySerialLink(on_tx)

    def send(self, code: int, value: int):
        if self.ser is None:
            # fallback
            self.dummy.send(code, value)
            return
        try:
            frame = pack(code, value)
            self.ser.write(frame)
            if self.on_tx:
                self.on_tx(frame.decode("ascii", errors="ignore"))
        except Exception as e:
            print("[Serial] write error:", e)

# ------------------------------
# Video sources (real or simulated)
# ------------------------------
class SourceBase:
    def read(self):
        """Return (ok: bool, frame: np.ndarray[BGR])"""
        raise NotImplementedError
    def release(self):
        pass

class CameraSource(SourceBase):
    def __init__(self, cam_index: int, w: int = 640, h: int = 480):
        self.cap = cv2.VideoCapture(cam_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    def read(self):
        ok, frame = self.cap.read()
        return ok, frame
    def release(self):
        try:
            self.cap.release()
        except Exception:
            pass

class SimulationSource(SourceBase):
    """Generate synthetic frames for demo.
    modes: 'line' (vertical black stripe), 'object' (orange dot)
    """
    def __init__(self, mode: str, w: int = 640, h: int = 480):
        self.mode = mode
        self.w, self.h = w, h
        self.t0 = time.time()
        self.frame = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        self.bg = (230, 230, 230)  # light grey background
        self.omega = 2.0  # rad/s
        self.amp = self.w * 0.35

    def _phase(self):
        t = time.time() - self.t0
        return math.sin(self.omega * t)

    def read(self):
        # base background
        frame = np.full((self.h, self.w, 3), self.bg, dtype=np.uint8)
        cx = int(self.w/2 + self.amp * self._phase())
        if self.mode == 'line':
            # Draw a vertical black stripe crossing ROI [300:460]
            x0, x1 = max(0, cx - 16), min(self.w-1, cx + 16)
            cv2.rectangle(frame, (x0, 300), (x1, self.h-1), (0, 0, 0), thickness=-1)
            # draw ROI box
            cv2.rectangle(frame, (0, 300), (self.w-1, 460), (100, 100, 100), 1)
        else:  # 'object'
            # Orange circle that moves left-right
            # BGR approx for orange (~HSV H 10)
            bgr_orange = (0, 140, 255)
            cv2.circle(frame, (cx, int(self.h*0.5)), 30, bgr_orange, -1)
        return True, frame

# ------------------------------
# PID Controller
# ------------------------------
@dataclass
class PID:
    Kp: float
    Ki: float
    Kd: float
    clip: int
    dead: int
    prev: float = 0.0
    acc: float = 0.0

    def step(self, error: float) -> int:
        self.acc += error
        u = self.Kp*error + self.Kd*(error - self.prev) + self.Ki*self.acc
        self.prev = error
        if abs(error) < self.dead:
            u = 0.0
        return int(np.clip(u, -self.clip, self.clip))

# ------------------------------
# Workers
# ------------------------------
class WorkerBase(threading.Thread):
    def __init__(self, name: str, source: SourceBase, transport: TransportBase, stop_evt: threading.Event, show_preview: bool):
        super().__init__(name=name, daemon=True)
        self.source = source
        self.transport = transport
        self.stop_evt = stop_evt
        self.show_preview = show_preview

    def cleanup(self, win_name: str):
        try:
            self.source.release()
        except Exception:
            pass
        if self.show_preview:
            try:
                cv2.destroyWindow(win_name)
            except Exception:
                pass

class LineFollowWorker(WorkerBase):
    def __init__(self, source: SourceBase, transport: TransportBase, stop_evt: threading.Event, pid_cfg: dict, show_preview: bool):
        super().__init__("LineFollowWorker", source, transport, stop_evt, show_preview)
        self.pid = PID(pid_cfg["Kp"], pid_cfg["Ki"], pid_cfg["Kd"], pid_cfg["clip"], pid_cfg["dead_zone_px"])

    def run(self):
        win = "Line Follow"
        while not self.stop_evt.is_set():
            ok, frame = self.source.read()
            if not ok:
                time.sleep(0.01)
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            roi = gray[300:460, :]
            _, mask = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
            M = cv2.moments(mask)
            turn = 0
            if M["m00"] > 10000:
                cx = int(M["m10"] / M["m00"])  # centroid x in ROI
                frame_center = mask.shape[1] // 2
                error = cx - frame_center
                turn = self.pid.step(float(error))
                self.transport.send(SET_TURN, int(turn))
                if self.show_preview:
                    cv2.circle(frame, (cx, 380), 6, (0, 0, 255), -1)
                    cv2.line(frame, (frame_center, 300), (frame_center, 460), (255, 0, 0), 1)
            else:
                self.transport.send(SET_TURN, 0)
            if self.show_preview:
                cv2.putText(frame, f"TURN: {turn:+d}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)
                cv2.imshow(win, frame)
                # Optional: show mask window for debugging thresholds
                try:
                    cv2.imshow("OT Mask", mask)
                except Exception:
                    pass
                if cv2.waitKey(1) & 0xFF == 27:  # ESC closes preview
                    break
        self.cleanup(win)

class ObjectTrackWorker(WorkerBase):
    # Wider HSV for simulated orange (OpenCV H: 0..179). Adjust on real camera.
    LOW = (5, 100, 100)
    HIGH = (30, 255, 255)

    def __init__(self, source: SourceBase, transport: TransportBase, stop_evt: threading.Event, show_preview: bool):
        # Provide a proper thread name and pass required args to base class
        super().__init__("ObjectTrackWorker", source, transport, stop_evt, show_preview)

    def run(self):
        win = "Object Track"
        prev_err = 0.0
        while not self.stop_evt.is_set():
            ok, frame = self.source.read()
            if not ok:
                time.sleep(0.01)
                continue
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array(self.LOW), np.array(self.HIGH))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            turn = 0
            if contours:
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)
                if area > 300:
                    x, y, w, h = cv2.boundingRect(c)
                    cx = x + w // 2
                    err = float(cx - (frame.shape[1] // 2))
                    # Simple PD controller
                    u = 0.15 * err + 0.05 * (err - prev_err)
                    prev_err = err
                    turn = int(np.clip(u, -15, 15))
                    self.transport.send(SET_TURN, int(turn))
                    if self.show_preview:
                        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                        cv2.line(frame, (frame.shape[1]//2, 0), (frame.shape[1]//2, frame.shape[0]-1), (255, 0, 0), 1)
                        cv2.circle(frame, (cx, y + h//2), 5, (0, 0, 255), -1)
                else:
                    self.transport.send(SET_TURN, 0)
            else:
                self.transport.send(SET_TURN, 0)
            if self.show_preview:
                cv2.putText(frame, f"TURN: {turn:+d}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)
                cv2.imshow(win, frame)
                if cv2.waitKey(1) & 0xFF == 27:
                    break
        self.cleanup(win)

# ------------------------------
# GUI
# ------------------------------
class ControlGUI:
    def __init__(self, root: tk.Tk, gui_cfg: dict, outq: queue.Queue):
        self.q = outq
        self.base_speed = int(gui_cfg.get("base_speed", 120))
        root.title(gui_cfg.get("window_title", "Wheelchair Controller"))
        root.geometry("560x420")

        # Mode switch
        self.mode = tk.StringVar(value="manual")
        modes = [("Manual", "manual"), ("Line Follow", "line"), ("Object Track", "object")]
        frm_modes = ttk.LabelFrame(root, text="Mode")
        frm_modes.pack(fill="x", padx=8, pady=8)
        for label, val in modes:
            ttk.Radiobutton(frm_modes, text=label, value=val, variable=self.mode, command=self._on_mode).pack(side="left", padx=8)

        # Manual controls
        frm_manual = ttk.LabelFrame(root, text="Manual Controls")
        frm_manual.pack(fill="both", expand=True, padx=8, pady=8)

        def btn(label, cmd):
            return ttk.Button(frm_manual, text=label, command=cmd, width=8)

        self.lbl_speed = ttk.Label(frm_manual, text=f"Speed: {self.base_speed}")
        self.lbl_speed.grid(row=0, column=1, pady=(8, 4))

        btn("▲", lambda: self._preset(1)).grid(row=1, column=1, padx=6, pady=6)
        btn("◀", lambda: self._preset(3)).grid(row=2, column=0, padx=6, pady=6)
        btn("■", lambda: self._preset(0)).grid(row=2, column=1, padx=6, pady=6)
        btn("▶", lambda: self._preset(4)).grid(row=2, column=2, padx=6, pady=6)
        btn("▼", lambda: self._preset(2)).grid(row=3, column=1, padx=6, pady=6)

        sfrm = ttk.Frame(frm_manual)
        sfrm.grid(row=1, column=3, rowspan=3, padx=12)
        ttk.Button(sfrm, text="-", command=lambda: self._bump(-10), width=4).pack(fill="x", pady=4)
        ttk.Button(sfrm, text="+", command=lambda: self._bump(+10), width=4).pack(fill="x", pady=4)

        # Status
        self.lbl_status = ttk.Label(root, text="Last TX: (none)")
        self.lbl_status.pack(fill="x", padx=8, pady=4)

        # Keyboard bindings
        root.bind("<KeyPress-w>", lambda e: self._preset(1))
        root.bind("<KeyPress-s>", lambda e: self._preset(2))
        root.bind("<KeyPress-a>", lambda e: self._preset(3))
        root.bind("<KeyPress-d>", lambda e: self._preset(4))
        root.bind("<space>",      lambda e: self._preset(0))

    def _on_mode(self):
        self.q.put({"type": "mode", "value": self.mode.get()})

    def _preset(self, code):
        # 0 stop, 1 fwd, 2 back, 3 left, 4 right
        self.q.put({"type": "manual_preset", "value": code, "speed": self.base_speed})

    def _bump(self, delta):
        self.base_speed = max(0, min(255, self.base_speed + delta))
        self.lbl_speed.configure(text=f"Speed: {self.base_speed}")

    def set_status(self, text: str):
        self.lbl_status.configure(text=f"Last TX: {text}")

# ------------------------------
# App / Orchestrator
# ------------------------------
class App:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.root = tk.Tk()
        self.gui = ControlGUI(self.root, cfg.get("gui", {}), outq=queue.Queue())
        self.q: queue.Queue = self.gui.q
        self.mode = "manual"

        # Transport
        if cfg.get("use_real_serial", False):
            self.transport = RealSerialLink(cfg["serial_port"], cfg["baudrate"], on_tx=self.gui.set_status)
        else:
            self.transport = DummySerialLink(on_tx=self.gui.set_status)

        # Worker state
        self.worker: threading.Thread | None = None
        self.stop_evt = threading.Event()

        # Poll GUI queue
        self.root.after(20, self._poll)

        # On close
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---- Worker helpers ----
    def _start_worker(self, mode: str):
        self._stop_worker()
        self.stop_evt.clear()
        show_preview = self.cfg.get("show_preview", True)

        # Choose source (simulate vs real camera)
        if self.cfg.get("simulate", True):
            source = SimulationSource(mode)
        else:
            cam_idx = self.cfg["cam_line"] if mode == "line" else self.cfg["cam_object"]
            source = CameraSource(cam_idx)

        if mode == "line":
            pid_cfg = self.cfg["pid"]
            self.worker = LineFollowWorker(source, self.transport, self.stop_evt, pid_cfg, show_preview)
        else:
            self.worker = ObjectTrackWorker(source, self.transport, self.stop_evt, show_preview)

        self.worker.start()
        print(f"[Worker] Started {mode}")

    def _stop_worker(self):
        if self.worker and self.worker.is_alive():
            self.stop_evt.set()
            self.worker.join(timeout=1.5)
            print("[Worker] Stopped")
        self.worker = None
        self.stop_evt.clear()
        # Neutral command
        self.transport.send(SET_TURN, 0)

    def switch_mode(self, new_mode: str):
        if new_mode == self.mode:
            return
        # Stop current autonomous worker
        if self.mode in ("line", "object"):
            self._stop_worker()
        self.mode = new_mode
        if new_mode in ("line", "object"):
            self._start_worker(new_mode)
        else:
            # Manual mode: neutral
            self.transport.send(SET_TURN, 0)

    # ---- GUI Queue Poll ----
    def _poll(self):
        try:
            while True:
                msg = self.q.get_nowait()
                t = msg.get("type")
                if t == "mode":
                    self.switch_mode(msg["value"])
                elif t == "manual_preset":
                    # Ensure manual mode, do not auto-switch unless already manual
                    if self.mode != "manual":
                        self.switch_mode("manual")
                    speed = int(msg.get("speed", 120))
                    preset = int(msg.get("value", 0))
                    packed = preset * 1000 + min(speed, 255)
                    self.transport.send(DRIVE_PRESET, packed)
                # ignore other msgs
        except queue.Empty:
            pass
        # Schedule next poll
        self.root.after(20, self._poll)

    def _on_close(self):
        # Stop workers, send STOP_ALL
        try:
            self._stop_worker()
        except Exception:
            pass
        try:
            self.transport.send(STOP_ALL, 0)
        except Exception:
            pass
        # Close any OpenCV windows
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        self.root.destroy()

    def run(self):
        self.root.mainloop()

# ------------------------------
# Entrypoint
# ------------------------------
if __name__ == "__main__":
    app = App(CONFIG)
    app.run()
