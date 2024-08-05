import socket
import threading
import select
from v2x_intf_pkg.v2x_const import V2XConstants as v2xconst

class TcpConnectionManager:
    def __init__(self):

        self.obu_ip =  v2xconst.OBU_IP 
        self.obu_port = v2xconst.OBU_PORT

        self.obu_connected = False
        self.receive_buffer = b''
        self.lock = threading.Lock()
        self.client_socket = None

        try :
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.obu_ip, self.obu_port))
            self.obu_connected = True
        except Exception as e:
            print('Error:', str(e))
            self.client_socket = None
            self.obu_connected = False

    def open_connection(self):
        with self.lock:
            if self.client_socket is not None:
                print(f"Previous connection exists with self.obu_connected = {self.obu_connected}")
                return self.obu_connected
            
            try:
                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client_socket.connect((self.obu_ip, self.obu_port))
                self.obu_connected = True
                return self.obu_connected
            except Exception as e:
                print('open_connection Error:', str(e))
                self.client_socket = None
                self.obu_connected = False
                return self.obu_connected

    def send_data(self, data):
        if self.obu_connected :
            with self.lock:
                try:
                    self.client_socket.send(data)
                except Exception as e:
                    print('send_data Error:', str(e))
                    self.client_socket.close()
                    self.client_socket = None
                    self.obu_connected = False
                    return None

    def receive_data(self):
        with self.lock:
            ready_to_read, _, _ = select.select([self.client_socket], [], [], 0.1)
            if ready_to_read:                
                received_data = self.client_socket.recv(1024)
                return received_data #.decode()
            return None

    def close_connection(self):
        with self.lock:
            self.client_socket.close()
            self.obu_connected = False
