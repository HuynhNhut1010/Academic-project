import socket
import threading


HEADER = 1024
PORT = 5050

SERVER = socket.gethostbyname(socket.gethostname())
#SERVER = '192.168.1.2'
ADDR = (SERVER, PORT)
FORMAT = 'ascii'
DISCONN_MSG = "DISCONNECT"

print(ADDR)

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(ADDR)

clients = []
nicknames = []

def broadcast(message):
    for client in clients:
        client.send(message)
        print(f"Sent message to sender: {message.decode(FORMAT)}" )
        
    
def send(conn, msg):
    message = str(msg).encode(FORMAT)
    conn.send(message)

def handle_client(conn, addr):
    connect = True
    while connect:
        try:
            msg = conn.recv(HEADER)
            print(f'{msg.decode(FORMAT)}')
            broadcast(msg)
        except:
            index = clients.index(conn)
            print(f"[index]: {index} ")
            clients.remove(conn)
            nickname = nicknames[index]
            broadcast(f"{nickname} out chat")
            nicknames.remove(nickname)
            print(f"{nickname} was out chat")
            conn.close()

def start():
    server.listen()
    print(f"[LISTENNING] listen on {ADDR}")
    while True:
        conn, addr = server.accept()
        #send(conn, "Welcome to box chat")
        conn.send('NICK'.encode(FORMAT))
        print("Sent NICK")
        nickname = conn.recv(HEADER).decode(FORMAT)
        print(f"{nickname} entered chat")

        nicknames.append(nickname)
        clients.append(conn)
        
        broadcast(f"{nickname} joined chat".encode(FORMAT))
        thread = threading.Thread(target=handle_client, args=(conn, addr))
        thread.start()
        print(f"[ACTIVE THREAD] {threading.active_count() - 1}")
        for client in clients:
            print(f"[Client] {client}")

print("[STARTING] server is started")
start()
