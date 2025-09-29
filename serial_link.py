import serial, time
from protocol import pack

class SerialLink:
    def __init__(self, port: str, baud: int):
        self.port_name = port
        self.baud = baud
        self.ser = None

    def connect(self):
        while True:
            try:
                self.ser = serial.Serial(self.port_name, self.baud, timeout=0.1)
                time.sleep(2)  # allow Arduino reset
                break
            except Exception as e:
                print(f"[SerialLink] Retry open {self.port_name}: {e}")
                time.sleep(1)

    def send(self, code: int, value: int):
        if not self.ser:
            return
        try:
            self.ser.write(pack(code, value))
        except Exception as e:
            print(f"[SerialLink] send error: {e}")
            try:
                self.ser.close()
            except: pass
            self.connect()