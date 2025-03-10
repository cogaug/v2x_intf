import socket
import threading

HOST = '0.0.0.0'
PORT1 = 9201
PORT2 = 9202

lock = threading.Lock()
client1 = None
client2 = None

def forward(src, dst, client_label):
    try:
        while True:
            data = src.recv(1024)
            if not data:
                break
            dst.sendall(data)
    except Exception as e:
        print(f"{client_label} forwarding error: {e}")
    finally:
        with lock:
            try:
                src.close()
            except:
                pass
            try:
                dst.close()
            except:
                pass
            global client1, client2
            if src == client1:
                client1 = None
                print("Client1 disconnected.")
            elif src == client2:
                client2 = None
                print("Client2 disconnected.")

def handle_connection(client_socket, client_id):
    global client1, client2

    with lock:
        if client_id == 1:
            if client1:
                print("Replacing existing Client1.")
                try: client1.close()
                except: pass
            client1 = client_socket
            print("Client1 ready.")
        elif client_id == 2:
            if client2:
                print("Replacing existing Client2.")
                try: client2.close()
                except: pass
            client2 = client_socket
            print("Client2 ready.")

        if client1 and client2:
            print("Both clients connected, starting forwarding...")
            threading.Thread(target=forward, args=(client1, client2, "Client1->Client2"), daemon=True).start()
            threading.Thread(target=forward, args=(client2, client1, "Client2->Client1"), daemon=True).start()

def listen(port, client_id):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, port))
    server_socket.listen()
    print(f"Listening on port {port} for Client {client_id}...")

    while True:
        client_socket, addr = server_socket.accept()
        print(f"Client {client_id} connected from {addr}")
        threading.Thread(target=handle_connection, args=(client_socket, client_id), daemon=True).start()

def main():
    threading.Thread(target=listen, args=(PORT1, 1), daemon=True).start()
    threading.Thread(target=listen, args=(PORT2, 2), daemon=True).start()

    print("Server is running. Waiting for connections...")
    while True:
        threading.Event().wait(10)

if __name__ == '__main__':
    main()
