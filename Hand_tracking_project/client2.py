import socket
import threading

HEADER = 1024
PORT = 5050
SERVER = "192.168.0.106"
ADDR = (SERVER, PORT)
FORMAT = "utf-8"
DISCONNECT_MESSAGE = "!DISCONNECT"

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(ADDR)

nickname = input("Choose a nickname: ")


# def send(msg):
#     message = msg.encode(FORMAT)
#     msg_length = len(message)
#     send_length = str(msg_length).encode(FORMAT)
#     send_length += b' ' * (HEADER - len(send_length))
#     client.send(send_length)
#     client.send(message)
#     print(client.recv(22).decode(FORMAT))

# send("hello")
# input()
# send("hello how are u")
# input()
# send("Im fine")
# input()
# send(DISCONNECT_MESSAGE)


def receive():
    while True:
        try:
            message = client.recv(HEADER).decode(FORMAT)
            if message == 'NICK':
                client.send(nickname.encode(FORMAT))
            else:
                message_split = message.splitlines()
                print(message_split)

        except:
            print("Error occurred")
            client.close()
            break
    return message_split


def write():
    while True:
        message = f'{nickname}: {input("")}'



receive_thread = threading.Thread(target=receive)
receive_thread.start()

write_thread = threading.Thread(target=write)
write_thread.start()
