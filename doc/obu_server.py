# 이 프로그램을 AWS에서 실행하고 ROS2 노드와 통신하도록 하여 패킷 송수신을 테스트할 수 있다.
# AWS 정보
#   obu-ip : 3.34.3.66
#   obu-port : 9201
# ROS2 노드 실행
#  $ ros2 run v2x_intf_pkg v2x_intf_node --obu-ip 3.34.3.66 --obu-port 9201


import socket
import threading

# Define the server address and port
HOST = '0.0.0.0'  # localhost
PORT = 9201         # Port for both clients

# Lock to ensure synchronized socket access
socket_lock = threading.Lock()

def handle_client(client_socket, other_client_socket):
    try:
        while True:
            data = client_socket.recv(1024)
            if not data:
                print("Client disconnected, waiting for reconnection...")
                break
            with socket_lock:
                if other_client_socket.fileno() != -1:  # Check if the socket is still valid
                    other_client_socket.sendall(data)
                    print("Forwarded received data")
                else:
                    print("Other client socket is closed. Stopping forwarding...")
                    break
    except (BrokenPipeError, ConnectionResetError):
        print("Connection lost, stopping forwarding...")
    except OSError as e:
        print(f"Socket error: {e}")
    finally:
        with socket_lock:
            try:
                client_socket.close()
            except OSError:
                pass  # Ignore errors from double closing


def accept_and_forward(server_socket, first_client_socket=None):
    first_client_socket = None
    
    while True:
        # Accept a connection
        client_socket, client_address = server_socket.accept()
        print(f"Client connected from {client_address}")

        if first_client_socket is None:
            first_client_socket = client_socket
        else:
            second_client_socket = client_socket
            # Start threads to handle communication in both directions
            thread_a_to_b = threading.Thread(target=handle_client, args=(first_client_socket, second_client_socket), daemon=True)
            thread_b_to_a = threading.Thread(target=handle_client, args=(second_client_socket, first_client_socket), daemon=True)

            thread_a_to_b.start()
            thread_b_to_a.start()

            first_client_socket = None  # Reset for the next pair

# Create and configure the server socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow reuse of the port

try:
    server_socket.bind((HOST, PORT))
    server_socket.listen()
    print(f"Server listening on {HOST}:{PORT}")

    # Accept and forward connections
    accept_and_forward(server_socket)

except KeyboardInterrupt:
    print("Server shutting down...")
finally:
    server_socket.close()
