# Simple command code registry. Keep codes aligned with Arduino sketch.
SET_TURN       = 40   # value: -15..+15 (used by autonomous modes)
SET_BOTHSPEED  = 60   # value: -100..100 (manual: same speed both wheels)
SET_LSPEED     = 61   # value: -100..100 (manual left)
SET_RSPEED     = 62   # value: -100..100 (manual right)
DRIVE_PRESET   = 70   # value: 0 stop, 1 fwd, 2 back, 3 left, 4 right (manual presets)
HEARTBEAT      = 90   # optional watchdog
STOP_ALL       = 99   # emergency stop

DIGITS = 4

def pack(code: int, value: int, digits: int = DIGITS) -> bytes:
    # Example: code=40, value=-7  -> b"$0040-007"
    return ("$" + str(int(code)).zfill(digits) + str(int(value)).zfill(digits)).encode()