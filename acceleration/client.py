import socket
import time
import threading as thr
import numpy as np


class Client:
    def __init__(self, host='192.168.88.21', port=6543):
        self.host = host
        self.port = port
        self.client_socket = socket.socket()
        self.client_socket.connect((self.host, self.port))
        print("(Client)\n    Connected to the server!")

        self.latest_data = None
        self.is_data_available = False

        self.recdata_lock = thr.Lock()
        self.recdata_thr = thr.Thread(target=self.interaction(), args=())
        self.recdata_thr.start()

        self.velocity = 0
        self.vel_proj = (0, 0)
        self.vel_updated = False

        self.vel_lock = thr.Lock()
        self.vel_thr = thr.Thread(target=self.interaction(), args=())
        self.vel_thr.start()

    def __del__(self):
        self.client_socket.sendall(b"Stop")
        self.client_socket.close()
        print("(Client)\n    Connection lost")

    def interaction(self):
        while True:
            self.client_socket.sendall(b"Give me acceleration")
            data = self.client_socket.recv(1024).decode()   # accX, accY, accZ, period between mesures, getting_time
            receiving_time = time.time()
            print(f"(Client)\n    Receiving Time: {receiving_time}")
            processed_data = list(map(float, data.split(", ")))
            print(f"    Processed data: X:{processed_data[0]}, Y:{processed_data[1]}, Z:{processed_data[2]}\n"
                  f"    Getting time: {processed_data[4]}, Delay: {receiving_time - processed_data[4]}")

            self.recdata_lock.acquire()
            self.latest_data = [[processed_data[0], processed_data[1], processed_data[2]],
                                processed_data[3], processed_data[4]]
            self.is_data_available = True
            self.recdata_lock.release()

    def velocity_stream(self):
        while True:
            if not self.is_data_available:
                time.sleep(0.05)
                continue
            self.is_data_available = False

            period = self.latest_data[1]
            rec_time = self.latest_data[2]
            acc_x, acc_y = self.latest_data[0][0], self.latest_data[0][1]

            vel_x = self.vel_proj[0] + acc_x * period
            vel_y = self.vel_proj[1] + acc_y * period

            velocity = np.linalg.norm([vel_x, vel_y])

            with self.vel_lock:
                self.vel_proj = vel_x, vel_y
                self.velocity = velocity
                self.vel_updated = True

    def get_velocity(self):
        while not self.vel_updated:
            time.sleep(0.05)

        with self.vel_lock:
            self.vel_updated = False
        return self.velocity


if __name__ == "__main__":
    pc = Client()
