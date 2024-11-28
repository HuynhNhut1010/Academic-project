import socket
import threading
import time

HEADER = 1024
PORT = 5050
SERVER = socket.gethostbyname(socket.gethostname())
ADDR = (SERVER, PORT)
FORMAT = "utf-8"
DISCONNECT_MESSAGE = "!DISCONNECT"

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(ADDR)


clients = []
nicknames = []


def handle_client(client):
    connected = True
    while connected:
        try:
            message = client.recv(HEADER)
            broadcast(message)
        except:
            index = clients.index(client)
            clients.remove(client)
            client.close()
            nickname = nicknames[index]
            nicknames.remove(nickname)
            broadcast(f'{nickname} left the chat'.encode(FORMAT))
            break
        # msg_length = conn.recv(HEADER).decode(FORMAT)
        # if msg_length:
        #     msg_length = int(msg_length)
        #     msg = conn.recv(msg_length).decode(FORMAT)
        #
        #     if msg == DISCONNECT_MESSAGE:
        #         connected = False
        #
        #     print(f"[{addr}] {msg}")
        #     conn.send("message received".encode(FORMAT))

    # conn.close()




def broadcast(message):
    for client in clients:
        client.send(message)


def start():
    server.listen()
    print(f"[LISTENING] Server is listing on {SERVER} ")
    while True:
        client, addr = server.accept()
        print(f"Welcome to chat {str(addr)}")
        client.send('NICK'.encode(FORMAT))
        nickname = client.recv(HEADER).decode(FORMAT)
        nicknames.append(nickname)
        clients.append(client)
        print(f"Nickname of client is {nickname}")
        thread = threading.Thread(target=handle_client, args=(client, ))
        thread.start()

print("[STARTING] server is starting ....")
start()
