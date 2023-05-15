import socket
import sys
import time
import threading as thr
import numpy as np


class Client:
    def __init__(self, host='192.168.88.21', port=6543, info=True):
        self.host = host
        self.port = port
        self.info = info

        self.client_socket = socket.socket()
        self.connected = True
        try:
            self.client_socket.connect((self.host, self.port))
        except socket.error:
            if self.connected:
                print("(Client)\n    Server is down")
                self.connected = False
            sys.exit(1)

        self.connected = True
        print("(Client)\n    Connected to the server!")

        self.latest_data = None
        self.is_data_available = False

        self.recdata_lock = thr.Lock()
        self.recdata_thr = thr.Thread(target=self.interaction, args=())
        self.recdata_thr.start()

        self.calibration_weight = (0, 0)
        self.calibration_data = [[0], [0]]
        self.velocity = 0
        self.vel_proj = (0, 0)
        self.vel_updated = False

        self.vel_lock = thr.Lock()
        self.vel_thr = thr.Thread(target=self.velocity_stream, args=())
        print("(Client)\n    Calibration ...")
        self.vel_thr.start()

    def __del__(self):
        if self.connected:
            self.client_socket.sendall(b"Stop")
        self.client_socket.close()
        print("(Client)\n    Connection lost")

    def interaction(self):
        while True:
            self.client_socket.sendall(b"Give me acceleration")
            data = self.client_socket.recv(1024).decode()   # accX, accY, accZ, period between mesures, getting_time
            receiving_time = time.time()
            processed_data = list(map(float, data.split(", ")))
            if self.info:
                print(f"    Processed data: X:{processed_data[0]}, Y:{processed_data[1]}, Z:{processed_data[2]}\n"
                      f"    Getting time: {processed_data[4]}, Delay: {receiving_time - processed_data[4]}")

            self.recdata_lock.acquire()
            self.latest_data = [[processed_data[0], processed_data[1], processed_data[2]],
                                processed_data[3], processed_data[4]]
            self.is_data_available = True
            self.recdata_lock.release()

    def velocity_stream(self, calibration_time=30):
        acc_x, acc_y = 0, 0
        counter = 0
        while True:
            if not self.is_data_available:
                time.sleep(0.05)
                continue
            self.is_data_available = False

            period = self.latest_data[1]
            rec_time = self.latest_data[2]
            raw_acc = self.latest_data[0][0],  self.latest_data[0][1]

            if len(self.calibration_data[0]) < calibration_time:
                # print(len(self.calibration_data[0]))
                self.calibration_data[0].append(raw_acc[0])
                self.calibration_data[1].append(raw_acc[1])
                if len(self.calibration_data[0]) == calibration_time:

                    weight1 = np.quantile(self.calibration_data[0][2:], 0.2), \
                              np.quantile(self.calibration_data[1][2:], 0.2)
                    weight2 = np.quantile(self.calibration_data[0][2:], 0.4), \
                              np.quantile(self.calibration_data[1][2:], 0.4)
                    weight3 = np.quantile(self.calibration_data[0][2:], 0.6), \
                              np.quantile(self.calibration_data[1][2:], 0.6)
                    weight4 = np.quantile(self.calibration_data[0][2:], 0.8), \
                              np.quantile(self.calibration_data[1][2:], 0.8)

                    self.calibration_weight = (weight1[0] + weight2[0] + weight3[0] + weight4[0])/4, \
                                              (weight1[1] + weight2[1] + weight3[1] + weight4[1])/4
                else:
                    continue

            raw_acc = raw_acc[0] - self.calibration_weight[0], raw_acc[1] - self.calibration_weight[1]

            acc_x, acc_y = self.filter_acc(raw_acc, (acc_x, acc_y))
            # print(acc_x, acc_y)
            if abs(acc_x) < 0.095:
                acc_x = 0
            if abs(acc_y) < 0.095:
                acc_y = 0
            print("                                                 Accel:", acc_x, acc_y)

            vel_x = self.vel_proj[0] + acc_x * period
            vel_y = self.vel_proj[1] + acc_y * period
            vel_x, vel_y = self.filter_acc((vel_x, vel_y), self.vel_proj, k=0.5)

            if abs(acc_x) < 0.1 and abs(acc_y) < 0.1:
                counter += 1
                if counter == 2:
                    vel_x, vel_y = 0, 0
                    counter = 0
            else:
                counter = 0

            velocity = np.linalg.norm([vel_x, vel_y])

            with self.vel_lock:
                self.vel_proj = vel_x, vel_y
                self.velocity = velocity
                self.vel_updated = True

    @staticmethod
    def filter_acc(acc_raw, acc_opt, k=0.8):
        acc_x, acc_y = k * acc_raw[0] + (1 - k) * acc_opt[0], k * acc_raw[1] + (1-k) * acc_opt[1]
        return acc_x, acc_y

    def get_velocity(self):
        while not self.vel_updated:
            time.sleep(0.05)

        with self.vel_lock:
            self.vel_updated = False
        return self.velocity


if __name__ == "__main__":
    pc = Client(host='192.168.88.24', info=False)

    while True:
        vel = pc.get_velocity()
        print(vel)
