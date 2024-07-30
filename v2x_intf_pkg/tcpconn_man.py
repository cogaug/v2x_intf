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
            self.obu_connected = False

    def send_data(self, data):
        if self.obu_connected :
            with self.lock:
                try:
                    # Serialize and send the data to the server
                    serialized_data = self.serialize_data(data)  # Implement serialization function
                    self.client_socket.send(serialized_data)
                    
                except Exception as e:
                    print('Error:', str(e))
                    return None


    def receive_data(self):
        with self.lock:
            ready_to_read, _, _ = select.select([self.client_socket], [], [], 0.1)
            if ready_to_read:
                received_data = self.client_socket.recv(1024)
                # self.receive_buffer += received_data
                return received_data.decode()
            return None

    def close_connection(self):
        with self.lock:
            self.client_socket.close()

    def serialize_data(self, data):
        # Implement serialization logic for your data here
        # For example, you can use pickle or another serialization method
        # Return the serialized data as bytes
        return bytes(str(data), 'utf-8')

