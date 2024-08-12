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

def handle_client(client_socket, other_client_socket):
    while True:
        try:
            # Receive data from the client
            data = client_socket.recv(1024)
            if not data:
                print("Client disconnected, waiting for reconnection...")
                break
            # Forward the data to the other client
            other_client_socket.sendall(data)
        except ConnectionResetError:
            print("Connection reset by peer, waiting for reconnection...")
            break

def accept_and_forward(server_socket, first_client_socket=None):
    while True:
        # Accept a connection
        client_socket, client_address = server_socket.accept()
        print(f"Client connected from {client_address}")

        if first_client_socket is None or not first_client_socket:
            first_client_socket = client_socket
        else:
            # Start threads to handle communication in both directions
            thread_a_to_b = threading.Thread(target=handle_client, args=(first_client_socket, client_socket))
            thread_b_to_a = threading.Thread(target=handle_client, args=(client_socket, first_client_socket))

            thread_a_to_b.start()
            thread_b_to_a.start()

            thread_a_to_b.join()
            thread_b_to_a.join()

            first_client_socket = None  # Reset after clients disconnect


# Create a socket for the server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    # Bind the socket to the address and port
    server_socket.bind((HOST, PORT))

    # Listen for incoming connections
    server_socket.listen()
    print(f"Server listening on {HOST}:{PORT}")

    # Accept connections and forward data
    accept_and_forward(server_socket)

finally:
    # Close the server socket
    server_socket.close()
