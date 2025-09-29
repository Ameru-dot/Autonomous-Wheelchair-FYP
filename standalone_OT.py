#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Object Track → Arduino (No GUI)
- Track objek berdasarkan warna (HSV) dari kamera.
- Kira error relatif ke tengah imej → tukar kepada 'turn' kecil (-15..+15).
- Hantar ke Arduino dengan kod SET_TURN (40) menggunakan bingkai "$0040±xxx".
- Keluar dengan selamat (CTRL+C): hantar STOP_ALL (99).

Keperluan:
  pip install opencv-python pyserial numpy
"""

import cv2, numpy as np, time, serial, sys

# ====== PROTOCOL (selaras dengan protocol.py) ======
SET_TURN      = 40   # value: -15..+15
STOP_ALL      = 99   # emergency stop
DIGITS        = 4

def pack(code: int, value: int, digits: int = DIGITS) -> bytes:
    # Contoh: code=40, value=-7 -> b"$0040-007"
    return ("$" + str(int(code)).zfill(digits) + str(int(value)).zfill(digits)).encode()

# ====== KONFIGURASI UBAH SUAI ======
PORT        = "/dev/ttyUSB0"   # tukar ikut port Arduino (cth "COM3" di Windows)
BAUD        = 9600
CAM_INDEX   = 1                # 0 atau 1 ikut kamera anda
SHOW_PREVIEW= True             # True untuk tengok tingkap video
FPS_LIMIT   = 60               # had FPS supaya stabil
TURN_CLIP   = 15               # clamp arahan stereng -15..+15 (padan Arduino)
# Julat HSV untuk sasaran (contoh 'orange' / boleh tuning)
HSV_LOW     = (5, 100, 100)
HSV_HIGH    = (30, 255, 255)
AREA_MIN    = 800              # kawal “noise” (min area contour)
KP          = 0.15             # penguat P
KD          = 0.05             # penguat D (ringan)
# Nota: Arduino anda sudah guna BASE_SPEED_AUTON dalaman untuk gerak ke depan,
# jadi tak perlu hantar DRIVE_PRESET di sini; SET_TURN sahaja sudah gerakkan motor.

def main():
    # --- buka serial ---
    print(f"[INFO] Opening serial {PORT} @ {BAUD}")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        time.sleep(2.0)  # beri masa Arduino auto-reset
    except Exception as e:
        print(f"[FATAL] Cannot open serial: {e}")
        sys.exit(1)

    # --- buka kamera ---
    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print(f"[FATAL] Cannot open camera index {CAM_INDEX}")
        try:
            ser.write(pack(STOP_ALL, 0))
        except: pass
        sys.exit(1)

    print("[INFO] Running object tracking. Press Ctrl+C to stop.")
    prev_err = 0.0
    t_last = time.time()
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.01)
                continue

            # HSV mask
            hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array(HSV_LOW), np.array(HSV_HIGH))

            # cari contour terbesar
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            turn = 0
            if contours:
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)
                if area > AREA_MIN:
                    x, y, w, h = cv2.boundingRect(c)
                    cx = x + w // 2
                    err = float(cx - (frame.shape[1] // 2))
                    # PD ringkas
                    u = KP * err + KD * (err - prev_err)
                    prev_err = err
                    turn = int(np.clip(u, -TURN_CLIP, TURN_CLIP))
                    # hantar SET_TURN
                    ser.write(pack(SET_TURN, turn))
                    # overlay (opsyenal)
                    if SHOW_PREVIEW:
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.line(frame, (frame.shape[1]//2, 0), (frame.shape[1]//2, frame.shape[0]-1), (255, 0, 0), 1)
                        cv2.circle(frame, (cx, y + h//2), 5, (0, 0, 255), -1)
                else:
                    # Tiada objek “cukup besar” → neutral kecil untuk elak jerk
                    ser.write(pack(SET_TURN, 0))
            else:
                ser.write(pack(SET_TURN, 0))

            # Paparan & throttle FPS
            if SHOW_PREVIEW:
                cv2.putText(frame, f"TURN: {turn:+d}", (10, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)
                cv2.imshow("ObjectTrack (No GUI)", frame)
                if cv2.waitKey(1) & 0xFF == 27:  # ESC
                    break

            # hadkan fps
            if FPS_LIMIT > 0:
                dt = time.time() - t_last
                min_dt = 1.0 / FPS_LIMIT
                if dt < min_dt:
                    time.sleep(min_dt - dt)
                t_last = time.time()

    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt received.")

    finally:
        # Stop selamat
        try:
            ser.write(pack(STOP_ALL, 0))
        except Exception as e:
            print("[WARN] STOP_ALL failed:", e)
        cap.release()
        if SHOW_PREVIEW:
            cv2.destroyAllWindows()
        try:
            ser.close()
        except: pass
        print("[INFO] Exit.")

if __name__ == "__main__":
    main()
