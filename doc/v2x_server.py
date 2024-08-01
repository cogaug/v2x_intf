import socket

def start_server():
    # Define server address and port
    server_address = '127.0.0.1'
    server_port = 9201  # Port to listen on (non-privileged ports are > 1023)

    # Create a TCP/IP socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        # Bind the socket to the address and port
        server_socket.bind((server_address, server_port))
        print(f"Server started at {server_address} on port {server_port}")

        # Listen for incoming connections
        server_socket.listen()

        while True:
            # Wait for a connection
            print("Waiting for a connection...")
            connection, client_address = server_socket.accept()
            with connection:
                print(f"Connected by {client_address}")

                while True:
                    data = connection.recv(1024)
                    if not data:
                        break
                    print(f"Received from {client_address}")

                    # TODO here, parse and display the received data


                    connection.sendall(data)  # Echo back the received data
                    print(f"Sent back to {client_address}")

if __name__ == "__main__":
    start_server()
