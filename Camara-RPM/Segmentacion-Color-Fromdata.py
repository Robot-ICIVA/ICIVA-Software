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
upper = np.array([180, 100, 100], dtype = "uint8") # MAX = 180, 255, 255

def main():

    frame = cv2.imread("Imagenes/Imagen1.jpg")
    w, h = frame.shape[1], frame.shape[0]
    centerx = int(round(w / 2))
    centery = int(round(h / 2))

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
        mask_cnt = cv2.bitwise_and(frame, drawing)
            # mask1 = np.zeros((h+2 , w +2), np.uint8)
            # cv2.floodFill(drawing, mask1,  (0, 0), 255)  # line 27



    while True:
        cv2.imshow('frame', frame)
        cv2.imshow('drawing', drawing)
        cv2.imshow('mask', mask_cnt)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            break
        if cv2.waitKey(1) & 0xFF == ord("t"):
            cv2.imwrite("Imagenes/Imagen1.jpg", frame)

if __name__ == "__main__": main()