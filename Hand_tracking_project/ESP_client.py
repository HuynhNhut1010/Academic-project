import socket
import time
import network
from machine import Pin

HEADER = 1024
PORT = 5050
SERVER = "192.168.1.2" # it will depend on your sever address
ADDR = (SERVER, PORT)
FORMAT = "ascii"
DISCONNECT_MESSAGE = "!DISCONNECT"
timeout = 0
ssid = 'New Wifi'
password = '123dat123'

# Initialize and connect to Wi-Fi
wifi = network.WLAN(network.STA_IF)
wifi.active(False)
time.sleep(0.5)
wifi.active(True)
wifi.connect(ssid, password)

while not wifi.isconnected() and timeout < 10:
    print(10 - timeout)
    timeout += 1
    time.sleep(1)

if wifi.isconnected():
    print('Connected to Wi-Fi')
    print('Network config', wifi.ifconfig())
else:
    print('Failed to connect to Wi-Fi')
    # Handle the case where Wi-Fi connection failed

nickname = 'esp32'
led_pin_number = [25, 26, 27, 14, 12]
led_pin = [Pin(pin, Pin.OUT) for pin in led_pin_number]

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(ADDR)

def receive():
    while True:
        try:
            message = client.recv(HEADER).decode(FORMAT)
            if message == 'NICK':
                client.send(nickname.encode(FORMAT))
            else:
                message_split = message.splitlines()
                print(message_split)

                for i in range(5):
                    if message_split[i] == '1':
                        led_pin[i].on()
                    else:
                        led_pin[i].off()

        except Exception as e:
            print("Error occurred:", str(e))
            client.close()
            break

receive()  # Start receiving messages (no need for threading in this case)
