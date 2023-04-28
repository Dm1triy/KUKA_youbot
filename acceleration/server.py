import socket
import time
from accel_listener import Accel
import threading as thr


class Server:
    def __init__(self, host='192.168.88.24', port=6543):
        self.accel_stream = Accel()
        self.host = host
        self.port = port
        self.server_socket = socket.socket()
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen()
        print("(Server):\n    Port opened\n")

        self.server_is_running = True

        conn, address = self.server_socket.accept()
        self.conn = conn
        self.conn_addres = address
        print(f"(Server):\n    Connection from: {str(address)}\n")

        self.interaction()

    def __del__(self):
        self.accel_stream.is_connected = False
        self.server_socket.close()
        self.conn.close()
        print("(Server):\n    Port closed\n")

    def interaction(self):
        while self.server_is_running:
            data = self.conn.recv(1024).decode()
            if data == "Give me acceleration":
                accel = self.get_accel()    #accelerations, period between to measures, getting time
                sending_time = time.time()
                print(f"(Server):\n    Sending time: {sending_time}, Delay {time.time()-accel[2]}\n")
                msg = f'{accel[0][0]}, {accel[0][1]}, {accel[0][2]}, {accel[1]}, {accel[2]}'
                print(f"    {msg}")
                self.conn.sendall(str.encode(msg))
                continue
            if data == "Stop":
                self.server_is_running = False
                return

    def get_accel(self):
        while True:
            data = self.accel_stream.get_data()
            if data:
                return data


if __name__ == "__main__":
    serv = Server(host=socket.gethostname())
    del serv
