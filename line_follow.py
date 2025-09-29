import cv2, numpy as np, time
from protocol import SET_TURN

class PID:
    def __init__(self, Kp, Ki, Kd, clip, dead):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.clip, self.dead = clip, dead
        self.prev = 0
        self.sum = 0
    def step(self, error):
        self.sum += error
        u = self.Kp*error + self.Kd*(error - self.prev) + self.Ki*self.sum
        self.prev = error
        if abs(error) < self.dead:
            u = 0
        return int(np.clip(u, -self.clip, self.clip))


def run_line_follow(cam_idx: int, pid_cfg: dict, send_fn, stop_event):
    cap = cv2.VideoCapture(cam_idx)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    pid = PID(pid_cfg["Kp"], pid_cfg["Ki"], pid_cfg["Kd"], pid_cfg["clip"], pid_cfg["dead_zone_px"])

    while not stop_event.is_set():
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.01)
            continue

        # grayscale and threshold for dark line (adjust as needed)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        roi = gray[300:460, :]  # bottom band
        _, mask = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

        M = cv2.moments(mask)
        if M["m00"] > 10000:
            cx = int(M["m10"] / M["m00"])  # centroid x in ROI
            frame_center = mask.shape[1] // 2
            error = cx - frame_center
            u = pid.step(error)
            send_fn(SET_TURN, u)
        else:
            # line lost -> slow stop
            send_fn(SET_TURN, 0)

    cap.release()