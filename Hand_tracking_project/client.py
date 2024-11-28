import socket
import threading
import cv2
import mediapipe as mp
import time
import array
import numpy as np
import math
import pyautogui
import pycaw

cap = cv2.VideoCapture(0)
myHand = mp.solutions.hands
hands = myHand.Hands()
mpDraw = mp.solutions.drawing_utils

pTime = 0
cTime = 0
handIDy = []
handIDx = []

HEADER = 1024
PORT = 5050
SERVER = "192.168.0.106"
ADDR = (SERVER, PORT)
FORMAT = "utf-8"
DISCONNECT_MESSAGE = "!DISCONNECT"

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(ADDR)

nickname = "Hand"


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
                print(message)
        except:
            print("Error occurred")
            client.close()
            break


def write(message):
    # while True:
    # message = f'{nickname}: {message_finger}'
    client.send(message.encode(FORMAT))


while True:
    success, img = cap.read()
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
    result = hands.process(imgRGB)
    receive_thread = threading.Thread(target=receive)
    receive_thread.start()
    # print(result.multi_hand_landmarks)
    if result.multi_hand_landmarks:
        for handLms in result.multi_hand_landmarks:
            mpDraw.draw_landmarks(img, handLms, myHand.HAND_CONNECTIONS)
        for id, lm in enumerate(handLms.landmark):
            h, w, c = img.shape
            cx, cy = int(lm.x * w), int(lm.y * h)
            handIDx.append(cx)
            handIDy.append(cy)
            if id == 4:
                cx_4 = cx
                cy_4 = cy
            if id == 8:
                cx_8 = cx
                cy_8 = cy
        # cv2.circle(img,(cx_8, cy_8),10,[255,0,255],cv2.FILLED)
        # cv2.circle(img, (cx_4, cy_4), 10, [255, 0, 255], cv2.FILLED)
        # cv2.line(img,(cx_4, cy_4),(cx_8, cy_8),[255, 0, 100],2)
        length = math.hypot(handIDy[8] - handIDy[4], handIDx[8] - handIDx[4])
        length1 = abs(handIDx[0] - handIDx[4])
        length2 = handIDy[0] - handIDy[8]
        length3 = handIDy[0] - handIDy[12]
        length4 = handIDy[0] - handIDy[16]
        length5 = handIDy[0] - handIDy[20]
        message_finger = ''
        if length1 > 90:
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

        write(message_finger)
        print(f"length1: {length1}")
        print(f"length2: {length2}")
        print(f"length3: {length3}")
        print(f"length4: {length4}")
        print(f"length5: {length5}")

        handIDx.clear()
        handIDy.clear()

        # devices = AudioUtilities.GetSpeakers()
        # interface = devices.Activate(
        #     IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
        # volume = interface.QueryInterface(IAudioEndpointVolume)
        # #volume.GetMute()
        # volume.GetMasterVolumeLevel()
        # vol = np.interp(length,[15,202],[-74,0])
        # print(volume.GetVolumeRange())
        # volume.SetMasterVolumeLevel(vol, None)
        # print(vol)
        # index_finger_x = handLms.landmark[myHand.HandLandmark.INDEX_FINGER_TIP].x
        # thumb_x = handLms.landmark[myHand.HandLandmark.THUMB_TIP].x
        # index_finger_y = handLms.landmark[myHand.HandLandmark.INDEX_FINGER_TIP].y
        # thumb_y = handLms.landmark[myHand.HandLandmark.THUMB_TIP].y
        # distance = math.sqrt((index_finger_x-thumb_x)**2 + (index_finger_y-thumb_y)**2)

    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime

    cv2.putText(img, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255))
    cv2.imshow("Image", img)
    cv2.waitKey(1)
