import cv2, numpy as np, time
from protocol import SET_TURN

# Placeholder: simple color tracking. Replace with your detector (YOLO, etc.).
LOW = (0, 120, 120)   # adjust for target color
HIGH = (15, 255, 255)

# This function will continuously process camera frames, detect an object by color,
# and then compute error relative to the image center. It converts error to a small
# steering command (u) and sends it to Arduino via SET_TURN. This is the logic that
# actually makes the motors steer left/right while object tracking.

def run_object_track(cam_idx: int, send_fn, stop_event):
    cap = cv2.VideoCapture(cam_idx)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    prev_err = 0
    while not stop_event.is_set():
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.01)
            continue
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(LOW), np.array(HIGH))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > 1000:
                x,y,w,h = cv2.boundingRect(c)
                cx = x + w//2
                err = cx - (frame.shape[1]//2)
                # simple P controller; tune or reuse PID class
                u = int(np.clip(0.15*err + 0.05*(err - prev_err), -15, 15))
                prev_err = err
                send_fn(SET_TURN, u)
                continue
        send_fn(SET_TURN, 0)  # nothing detected
    cap.release()
