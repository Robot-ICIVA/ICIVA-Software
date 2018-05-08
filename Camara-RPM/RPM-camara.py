"""
* #################################INFORMACION###############################
*     Filename    : ConsolaCMU.py
*     Project     : ICIVA-ROBOT
*     Board       : Demoqe
*     Autor       : Luis Lujano (13-10775)
*     GitHub      : https://github.com/Lujano
*     Sensors     : CMUcam1 (camera)
* ###########################################################################

Descripcion:
    Este programa permite la comunicacion via serial entre la camara CMUcam1 y la pc mediante el empleo de python (3.6).
    Commandos:
    -Los commandos se ingresan por consola.
    Data:
    - Implementada la decodificacion de todos los tipos de paquete enviados por la camara
    - Se permite la visualizacion de los paquetes recibidos en formato string y como arreglo de enteros
    - Se puede visualizar la imagen tomada por la camara mediante el comando DF, y matplotlib (tiempo aprox 6 seg)
"""

# Librerias
import cv2
import numpy as np
import time

# Valores minimos y maximos de la region de color a segmentar (HSV)

lower = np.array([0, 0, 0], dtype = "uint8")
upper = np.array([180, 150, 150], dtype = "uint8") # MAX = 180, 255, 255

lower_white = np.array([9, 0, 0], dtype = "uint8")
upper_white = np.array([20, 255, 255], dtype = "uint8") # MAX = 180, 255, 255

def main():
    cap = cv2.VideoCapture('Video/3_26/output2.avi')
    while (cap.isOpened()):
        ret, frame = cap.read()
        frame = frame[:479, 200:400] # y, x
        # frame = cv2.flip(frame, 1)
        #frame = cv2.imread("Imagenes/Imagen1.jpg")
        w, h = frame.shape[1], frame.shape[0]
        centerx = int(round(w / 2))
        centery = int(round(h / 2))
        # Sharpened
        # image = cv2.GaussianBlur(frame, (0, 0), 3)
        # image = cv2.addWeighted(frame, 1.5, image, -0.5, 0, image)
        # frame = image
        img_blur = cv2.medianBlur(frame, 5)
        converted = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        skinMask = cv2.inRange(converted, lower, upper)



        kernel = np.ones((5, 5), np.uint8)
        skinMask = cv2.medianBlur(skinMask, 5)
        img = frame
        thresh1 = skinMask
        im3, contours, hierarchy = cv2.findContours(thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        drawing = np.zeros(img.shape, np.uint8)
        max_area = 0
        ci = 0

        if len(contours) > 0:
            for i in range(len(contours)):
                cnt = contours[i]
                area = cv2.contourArea(cnt)
                if (area > max_area):
                    max_area = area
                    ci = i
            cnt = contours[ci]
            cv2.drawContours(drawing, [cnt], 0, (255, 255, 255), -1)


            mask_cnt_Color= cv2.bitwise_and(drawing, img_blur)
                # mask1 = np.zeros((h+2 , w +2), np.uint8)
                # cv2.floodFill(drawing, mask1,  (0, 0), 255)  # line 27, no es necesario, el contorno contiene al centro de la rueda

        converted = cv2.cvtColor(mask_cnt_Color, cv2.COLOR_BGR2HSV)
        skinMask2 = cv2.inRange(converted, lower_white, upper_white)

        kernel = np.ones((5, 5), np.uint8)
        skinMask = cv2.medianBlur(skinMask2, 5)
        img = mask_cnt_Color
        thresh1 = skinMask
        im3, contours, hierarchy = cv2.findContours(thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        drawing2= np.zeros(img.shape, np.uint8)
        max_area = 0
        ci = 0

        if len(contours) > 0:
            for i in range(len(contours)):
                cnt = contours[i]
                area = cv2.contourArea(cnt)
                if (area > max_area):
                    max_area = area
                    ci = i
            cnt = contours[ci]
            cv2.drawContours(drawing2, [cnt], 0, (255, 255, 255), 2)
        cv2.imshow("original", frame)
        cv2.imshow('mask', mask_cnt_Color)
        cv2.imshow('mask2', drawing2)
        # cv2.imshow("Original", frame)
        # cv2.imshow("Ver", image)
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if (type(frame) != type(None)):
            cv2.imshow('frame', frame)
            if cv2.waitKey(1000) & 0xFF == ord('q'):
                break
        else:
            print("Finished")
            break

if __name__ == "__main__": main()
