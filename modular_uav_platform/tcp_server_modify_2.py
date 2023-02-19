import socket
import threading
import struct
import time
import queue
import numpy as np
# from test_read_txt import read_txt_file, read_txt_file_2
# from test_write_txt import  write_txt_file

from utils import bin2int, bin2nparray


class TcpServer(object):

    def __init__(self, host, port):
        self.host_ = host
        self.port_ = port

        self.sock_ = None
        self.q_ = queue.Queue()

        self.quit_event_ = threading.Event()

    def launch(self):
        print("Server launched at {}:{}".format(self.host_, self.port_))

        self.sock_ = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock_.bind((self.host_, self.port_))
        self.sock_.listen(1)

        self.quit_event_.clear()

        self.start_server_()

    def stop(self):
        self.quit_event_.set()

        if self.sock_ is not None:
            socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect((self.host_, self.port_))
            self.sock_.close()
            self.sock_ = None

    def get(self):
        return self.q_.get()

    def start_server_(self):
        # listen to connection request
        while not self.quit_event_.is_set():
            # blocked for next connection
            conn, addr = self.sock_.accept()
            thread = threading.Thread(target=self.handle_connection_, args=(conn, addr))
            thread.start()

    # This function need to be override by its child class
    def handle_connection_(self, conn, addr):
        conn_id = "{}:{}".format(addr[0], addr[1])
        print('New connection from {}'.format(conn_id))

        while not self.quit_event_.is_set():
            pack_size = conn.recv(4)

            # end of Connection
            if not pack_size:
                break

            pack_size = bin2int(pack_size)
            # fetch data package
            data = self.recv_all_(conn, pack_size)
            self.q_.put(data)

        conn.close()
        # print("Connection {}: closed".format(conn_id))

    def recv_all_(self, sock, msg_length):
        data = b""
        size_left = msg_length

        while len(data) < msg_length and not self.quit_event_.is_set():
            recv_data = sock.recv(size_left)
            size_left -= len(recv_data)
            data += recv_data

        return data


class Vicon(object):
    def __init__(self):
        self.t1 = None
        self.server = TcpServer("192.168.10.11", port=8800)

        self.position = np.zeros(3)
        self.rotation = np.zeros(4)
        self.rpy = np.zeros(3)
        self.position_prev = np.zeros(3)
        self.rotation_prev = np.zeros(4)
        self.velocity = np.zeros(3)
        self.rotation_rate = np.zeros(3)

        self.t = threading.Thread(target=self.server.launch)
        self.t.start()
        self.run()

    def F(self):
        previous_time = time.time()
        while True:
            data = self.server.get()
            # print(data)
            data = bin2nparray(data)
            self.position = data[0:3]
            self.velocity = data[3:6]

            self.rotation_rate = data[6:9]
            self.rotation = data[9:]
            current_time = time.time()
            # print(current_time - previous_time)
            #print(self.position)
            previous_time = current_time

    def run(self):
        self.t1 = threading.Thread(target=self.F, args=())
        self.t1.start()


if __name__ == "__main__":
    vicon = Vicon()
    # while True:
    #     print("==== pos ====")
    #     print(vicon.position)
    #     print("==== quan ====")
    #     print(vicon.rotation)
    #     print("==== vel ====")
    #     print(vicon.velocity)

    #     print("==== angular vel ====")
    #     print(vicon.rotation_rate)
    #     time.sleep(0.5)
