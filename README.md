# AI-Powered Autonomous Wheelchair Controller

This project provides a modular control system for an autonomous/smart wheelchair powered by **Jetson Nano + Arduino**.
It supports multiple modes of operation:

* **Manual Control** via GUI (keyboard & buttons)
* **Line Following** using OpenCV + PID
* **Object Tracking** using color/YOLO-style detection

A simulation mode (`wheelchair_sim.py`) is included for testing without hardware.

---

## Features

* 📡 **Serial communication** with Arduino (custom protocol using `$0040-007` format).
* 🎛 **GUI interface (Tkinter)** for mode switching and manual driving.
* 🤖 **Autonomous Line Following** with PID control.
* 🎯 **Autonomous Object Tracking** with HSV-based color detection (can be upgraded to YOLO).
* 🖥 **Simulation mode** for testing without Arduino/hardware.

---

## Repository Structure

```
├── line_follow.py       # Line following worker with PID
├── object_track.py      # Object tracking worker (color-based)
├── serial_link.py       # Serial communication to Arduino
├── protocol.py          # Command codes + packet packing
├── main.py              # Orchestrator (Jetson Nano entrypoint)
├── wheelchair_sim.py    # Simulation (runs without hardware)
├── gui_tkinter.py       # Tkinter GUI frontend
├── config.json          # Configuration (serial, PID, cameras, GUI)
```

---

## Installation

### On Jetson Nano

```bash
sudo apt-get update
sudo apt-get install -y python3-opencv python3-serial python3-tk
```

### On Laptop (Simulation)

```bash
pip install opencv-python numpy pyserial
```

---

## Usage

### Run with Real Hardware

1. Connect Arduino via USB.
2. Update `config.json` with correct **serial port** (e.g. `/dev/ttyUSB0` or `COM3`).
3. Run:

   ```bash
   python3 main.py
   ```

### Run in Simulation Mode (No Arduino Needed)

```bash
python3 wheelchair_sim.py
```

---

## Controls

* **GUI Buttons**: Forward (▲), Back (▼), Left (◀), Right (▶), Stop (■)
* **Keyboard**:

  * `W` = Forward
  * `S` = Backward
  * `A` = Left
  * `D` = Right
  * `Space` = Stop
* **Mode Switching**: Radio buttons for `Manual`, `Line Follow`, `Object Track`
* **Speed Control**: + / – buttons (0–255)

---

## Configuration

Edit `config.json`:

```json
{
  "serial_port": "/dev/ttyUSB0",
  "baudrate": 9600,
  "cam_line": 0,
  "cam_object": 1,
  "pid": {"Kp": 0.18, "Ki": 0.00, "Kd": 0.06, "dead_zone_px": 10, "clip": 15},
  "gui": {"window_title": "Wheelchair Controller", "base_speed": 120}
}
```

---

## Arduino Protocol

Commands are sent as ASCII frames, e.g.:

```
$0040-007
```

Where:

* **0040** = Command code
* **-007** = Value

Defined codes (`protocol.py`):

* `40` → SET_TURN (steering, -15..15)
* `60` → SET_BOTHSPEED
* `61` → SET_LSPEED
* `62` → SET_RSPEED
* `70` → DRIVE_PRESET (0=stop, 1=fwd, 2=back, 3=left, 4=right)
* `90` → HEARTBEAT
* `99` → STOP_ALL

---

## Next Steps

* Replace color-based tracking in `object_track.py` with **YOLOv8** or similar detector.
* Tune PID parameters in `config.json` for smoother line following.
* Expand Arduino firmware to interpret and execute protocol commands.
