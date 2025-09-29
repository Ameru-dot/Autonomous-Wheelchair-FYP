import tkinter as tk
from tkinter import ttk
from multiprocessing import Queue

class ControlGUI:
    def __init__(self, root: tk.Tk, gui_cfg: dict, outq: Queue):
        self.q = outq
        self.base_speed = int(gui_cfg.get("base_speed", 120))
        root.title(gui_cfg.get("window_title", "Wheelchair Controller"))
        root.geometry("540x380")

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
            return ttk.Button(frm_manual, text=label, command=cmd)

        self.lbl_speed = ttk.Label(frm_manual, text=f"Speed: {self.base_speed}")
        self.lbl_speed.grid(row=0, column=1)

        btn("▲", lambda: self._preset(1)).grid(row=1, column=1, padx=6, pady=6)
        btn("◀", lambda: self._preset(3)).grid(row=2, column=0, padx=6, pady=6)
        btn("■", lambda: self._preset(0)).grid(row=2, column=1, padx=6, pady=6)
        btn("▶", lambda: self._preset(4)).grid(row=2, column=2, padx=6, pady=6)
        btn("▼", lambda: self._preset(2)).grid(row=3, column=1, padx=6, pady=6)

        sfrm = ttk.Frame(frm_manual)
        sfrm.grid(row=1, column=3, rowspan=3, padx=12)
        ttk.Button(sfrm, text="-", command=lambda: self._bump(-10)).pack(fill="x", pady=4)
        ttk.Button(sfrm, text="+", command=lambda: self._bump(+10)).pack(fill="x", pady=4)

        # Keyboard bindings
        root.bind("<KeyPress-w>", lambda e: self._preset(1))
        root.bind("<KeyPress-s>", lambda e: self._preset(2))
        root.bind("<KeyPress-a>", lambda e: self._preset(3))
        root.bind("<KeyPress-d>", lambda e: self._preset(4))
        root.bind("<space>",        lambda e: self._preset(0))

    def _on_mode(self):
        self.q.put({"type": "mode", "value": self.mode.get()})

    def _preset(self, code):
        # 0 stop, 1 fwd, 2 back, 3 left, 4 right
        self.q.put({"type": "manual_preset", "value": code, "speed": self.base_speed})

    def _bump(self, delta):
        self.base_speed = max(0, min(255, self.base_speed + delta))
        self.lbl_speed.configure(text=f"Speed: {self.base_speed}")


def start_gui(outq: Queue, gui_cfg: dict):
    root = tk.Tk()
    ControlGUI(root, gui_cfg, outq)
    root.mainloop()