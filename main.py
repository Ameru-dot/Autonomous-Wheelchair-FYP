## 1) Install deps (Jetson Nano)
#sudo apt-get update
#sudo apt-get install -y python3-opencv python3-serial python3-tk
# 2) Put files in a folder, edit config.json if needed
# 3) Run
#python3 main.py
import json, multiprocessing as mp, time
from serial_link import SerialLink
from protocol import SET_TURN, DRIVE_PRESET, STOP_ALL
from gui_tkinter import start_gui
from line_follow import run_line_follow
from object_track import run_object_track

class Orchestrator:
    def __init__(self, cfg):
        self.cfg = cfg
        self.serial = SerialLink(cfg["serial_port"], cfg["baudrate"])
        self.serial.connect()
        self.gui_q = mp.Queue()
        self.proc = None
        self.stop_evt = None
        self.mode = "manual"

    def send(self, code, value):
        self.serial.send(code, value)

    def _start_worker(self, target, *args): 
        self.stop_evt = mp.Event()
        self.proc = mp.Process(target=target, args=(*args, self.stop_evt), daemon=True)
        self.proc.start()

    def _stop_worker(self):
        if self.stop_evt:
            self.stop_evt.set()
        if self.proc and self.proc.is_alive():
            self.proc.join(timeout=1.0)
        self.proc = None
        self.stop_evt = None

    def switch_mode(self, new_mode):
        if new_mode == self.mode:
            return
        # stop current autonomous worker
        self._stop_worker()
        self.mode = new_mode
        # start requested worker (manual has no worker)
        if new_mode == "line":
            self._start_worker(run_line_follow, self.cfg["cam_line"], self.cfg["pid"], self.send)
        elif new_mode == "object":
            self._start_worker(run_object_track, self.cfg["cam_object"], self.send)
        else:
            # manual mode: ensure neutral
            self.send(SET_TURN, 0)

    def loop(self):
        # start GUI in a subprocess to keep Tk responsive
        gui_p = mp.Process(target=start_gui, args=(self.gui_q, self.cfg.get("gui", {})), daemon=True)
        gui_p.start()
        last_hb = time.time()
        try:
            while True:
                # poll GUI queue
                try:
                    msg = self.gui_q.get_nowait()
                except Exception:
                    msg = None
                if msg:
                    t = msg.get("type")
                    if t == "mode":
                        self.switch_mode(msg["value"])
                    elif t == "manual_preset":
                        self.switch_mode("manual")
                        # forward to Arduino
                        speed = int(msg.get("speed", 120))
                        code = msg.get("value", 0)
                        self.serial.send(DRIVE_PRESET, code*1_000 + min(speed,255))
                # optional: heartbeat every 1s to keep watchdogs alive
                if time.time() - last_hb > 1.0:
                    last_hb = time.time()
                time.sleep(0.01)
        finally:
            self._stop_worker()
            self.serial.send(STOP_ALL, 0)
            gui_p.terminate()

if __name__ == "__main__":
    with open("config.json", "r") as f:
        cfg = json.load(f)
    Orchestrator(cfg).loop()