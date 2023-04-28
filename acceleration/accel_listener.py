import serial
import time
import threading as thr


class Accel:
    def __init__(self):
        self.port = '/dev/ttyUSB0'
        self.timeout = 1
        self.is_connected = False
        self.ser = None

        self.last_data = None
        self.is_data_available = False

        self.connect()

        self.lock = thr.Lock()

        self.stream = thr.Thread(target=self.run_stream, args=())
        self.stream.start()
        # self.stream.join()
        # self.run_stream()

    def __del__(self):
        if self.ser:
            self.ser.close()

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, 9600, timeout=self.timeout)
            self.is_connected = True
        except serial.serialutil.SerialException as err:
            print(f"(Serial):\n    Unexpected {err=}, {type(err)=}\n")
            print(f"(Serial):\n    Can't listen port {self.port}\n")
            self.is_connected = False
        time.sleep(1)

    def run_stream(self):
        while self.is_connected:
            is_data_available, period, get_time = self.wait_until_data()
            if not is_data_available:
                self.is_connected = False
                break

            line = self.ser.readline()
            if line[0] != 88:
                print(str(line, "utf-8"))
                continue
            accel = list(map(float, line.strip().split()[1::2]))
            #print(f'(Serial):\n    Get Accelerometer data: X={accel[0]}, Y={accel[1]}, Z={accel[2]}, delay={period}\n')
            self.lock.acquire()
            self.last_data = [accel, period, get_time]
            self.is_data_available = True
            self.lock.release()

        print("(Serial):\n    Stream is shout down!\n")

    def wait_until_data(self, period=0.1):
        start_time = time.time()
        time_ceiling = start_time + self.timeout
        while time.time() < time_ceiling:
            get_time = time.time()
            if self.ser.in_waiting:
                #print(f"Data is ready! get_time: {get_time} \nperiod {get_time-start_time}")
                return True, get_time-start_time, get_time
            time.sleep(period)
        print(f"(Serial):\n    Timeout {self.port}\n")
        return False, None, None

    def get_data(self):
        out = None
        self.lock.acquire()
        if self.is_data_available:
            self.is_data_available = False
            out = self.last_data
        self.lock.release()
        return out


if __name__ == "__main__":
    test = Accel()
    if not test.is_connected:
        del test

    while True:
        t = test.get_data()
        if t:
            print(t)

