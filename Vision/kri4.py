import numpy as np
import cv2
#import serial
#import syslog
from time import sleep
import time

def ambil():
    # Serial Communication Initialization
    # ser = serial.Serial("/dev/OpenCM9.04", 9600, timeout=5)  # Open port with baud rate
    # ser = serial.Serial ("COM13", 9600)

    data = 'S'
    data1 = 'A'
    video_capture = cv2.VideoCapture(0)
    sleep(0.3)
    video_capture.set(3, 640)
    video_capture.set(4, 480)
    ctx = 0
    cty = 0
    cy = 0
    cx = 0
    a = 1

    # variable buat filtering dan kepala
    cx31 = 320  # nilai tengah servo kepala buat pandangan lurus
    print("HAHA")
    # ser.write(str.encode(str(int(cx31))))  # str encode itu karna biasa dipake,cx31 itu float karna hasil dari filternya, makannya diubah jadi int. trus diubah lagi jadi str karna str.encode perlu value str.
    alpha = 0.8  # ini buat parameter filter. 0 sangat responsif, 1 sangat tidak responsif

    # variable untuk kirim data delay
    period = 0.1  # dalam second


    while (True):
        # sekarang = time.time()

        # Capture the frames
        ret, frame = video_capture.read()

        # Crop the image
        crop_img = frame[80:320, 0:640]

        # Convert to grayscale
        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)

        # Convert to HSV
        img_hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Make Yellow Color Range
        lower_yellow = np.array([20, 100, 100], dtype="uint8")
        upper_yellow = np.array([30, 255, 255], dtype="uint8")

        # Make Yellow Mask
        mask_yellow = cv2.inRange(img_hsv, lower_yellow, upper_yellow)

        # Make White Mask
        mask_white = cv2.inRange(gray, 200, 255)

        # Make Mask of White or Yellow
        mask_yw = cv2.bitwise_or(mask_white, mask_yellow)

        # Make Mask of Gray and (White or Yellow)
        mask_yw_image = cv2.bitwise_and(gray, mask_yw)

        # Gaussian blur
        blur = cv2.GaussianBlur(mask_yw_image, (5, 5), 0)

        # Color thresholding
        ret, thresh = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY)

        # For delete noise
        # The erosion makes the object in white smaller.
        thresh = cv2.erode(thresh, None, iterations=2)
        # The dilatation makes the object in white bigger.
        thresh = cv2.dilate(thresh, None, iterations=2)

        # Find the contours of the frame
        contours, hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)[-2:]

        # Find the biggest contour (if detected)
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            cty = cy
            if M['m00'] != 0:
                # Menentukan Center dari Contour
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
            else:
                cx = cx
                cy = cy
            if (cty != 0 and (cy - cty) > 10):
                a = cx
                cx = ctx
            else:
                a = 0

            # Membuat Garis Center
            cv2.line(crop_img, (cx, 0), (cx, 720), (255, 0, 0), 1)
            cv2.line(crop_img, (0, cy), (1280, cy), (255, 0, 0), 1)
            cv2.drawContours(crop_img, contours, -1, (0, 255, 0),
                             1)  # cx = data dari openCV. berupa int titik koordinat horizontal
            # cx2 = cx*255/640.0 #cx2 buat scaling
            # print(cx2)
            if cx < 640 and cx > 0:
                print("On Track!")
                data = 'W'
            else:
                print("I don't see the line")
                data = 'S'
                print("S")

            cx3 = str(int(cx))  # ini buat ngubah jadi string si cx nya
            # cx4 = cx3.to_bytes(1,'little')
            # alpha = 0.5 #ini buat parameter filter
            # setTemp1 = cx2.encode('utf-8')
            # ser.flush()
            # setTemp = str(setTemp1)

            cx31 = (alpha * cx31) + ((1 - alpha) * cx)  # rumus filternya
            print("hasil: ")
            print(cx, cx3, cx31)
            print(cx, cy)
            # print(cx)
            # if(time.time() > sekarang + period):
            # ser.write(str.encode(str(int(cx31)))) <-- inii yg pentingg

        # ser.write(str.encode(cx3))
        # sekarang = time.time()
        # print("data dikirim")
        # time.sleep(0.5)

        # time.sleep(6)
        # print(cx)
        # data = cx

        # Mengupdate data hanya jika terjadi perubahan
        if (data1 != data):
            data1 = data
            # Mengirim data secara serial
            # ser.write(cx,cy)
        # Display the resulting frame
        cv2.imshow('mask_yw_image', mask_yw_image)
        cv2.imshow('contours', thresh)
        cv2.imshow('frame', crop_img)

        cv2.waitKey(1) & 0xFF

delay = 200  ###for 10 seconds delay
sekarang = time.time()
close_time = time.time() + delay
while True:
    ambil()
    if time.time() > close_time:
        break
cv2.destroyAllWindows()
