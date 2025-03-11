import socket
import threading
import cv2
import mediapipe as mp
import time
import math

HEADER = 1024
PORT = 5050
SERVER = '192.168.56.1'
ADDR = (SERVER, PORT)
FORMAT = 'ascii'
DISCONN_MSG = "DISCONNECT"
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(ADDR)
nickname = "Hand tracking"


cap = cv2.VideoCapture(0)

mpHand = mp.solutions.hands
hand = mpHand.Hands()

handIDy = []
handIDx = []

mpDraw = mp.solutions.drawing_utils

def send(message):
    while True:
        message = input()
        client.send(message.encode(FORMAT))

def recive():
    while True:
        message_recv = client.recv(1024).decode(FORMAT)
        if message_recv == 'NICK':
            client.send(nickname.encode(FORMAT)) 
        else:
            print(f"{message_recv}")



recive_thread = threading.Thread(target=recive)
recive_thread.start()

while True:
    success, img = cap.read()
    
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    result = hand.process(imgRGB)
    h, w, c = img.shape
    #print(result.multi_hand_landmarks)

    if result.multi_hand_landmarks:
        handIDy.clear()
        handIDx.clear()
        for handLand in result.multi_hand_landmarks:
            mpDraw.draw_landmarks(img, handLand, mpHand.HAND_CONNECTIONS)
            handIDy = []
            handIDx = []
            for id, lm in enumerate(handLand.landmark):
                cx, cy = int(lm.x*w) , int(lm.y*h)
                handIDy.append(cy)
                handIDx.append(cx) 
            
        length1 = (int)(math.hypot(handIDy[0] - handIDy[4], handIDx[0] - handIDx[4]))
        length2 = (int)(math.hypot(handIDy[0] - handIDy[8], handIDx[0] - handIDx[8]))
        length3 = (int)(math.hypot(handIDy[0] - handIDy[12], handIDx[0] - handIDx[12]))
        length4 = (int)(math.hypot(handIDy[0] - handIDy[16], handIDx[0] - handIDx[16]))
        length5 = (int)(math.hypot(handIDy[0] - handIDy[20], handIDx[0] - handIDx[20]))

        #length1 = abs(handIDx[0] - handIDx[4])
        #length2 = handIDy[0] - handIDy[8]
        #length3 = handIDy[0] - handIDy[12]
        #length4 = handIDy[0] - handIDy[16]
        #length5 = handIDy[0] - handIDy[20]
        message_finger = ''
        
        if length1 > 100:
            message_finger = message_finger + '1\n'
        else:
            message_finger = message_finger + '0\n'

        if length2 > 120:
            message_finger = message_finger + '1\n'
        else:
            message_finger = message_finger + '0\n'

        if length3 > 120:
            message_finger = message_finger + '1\n'
        else:
            message_finger = message_finger + '0\n'

        if length4 > 120:
            message_finger = message_finger + '1\n'
        else:
            message_finger = message_finger + '0\n'

        if length5 > 120:
            message_finger = message_finger + '1\n'
        else:
            message_finger = message_finger + '0\n'

        if message_finger == '':
            message_finger = '0'

        #print(length1, length2, length3, length4,length5)
        print(length1)
        print(message_finger)
        #send_thread = threading.Thread(target=send, args=message_finger)
        #send_thread.start()
        client.send(message_finger.encode(FORMAT))

    cv2.imshow("Image", img)
    cv2.waitKey(1)

