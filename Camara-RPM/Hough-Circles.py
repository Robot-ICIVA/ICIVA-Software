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

def main():
    camera = cv2.VideoCapture(0)
    time.sleep(2)
    while True:
        grabbed, frame = camera.read()
        img = frame
        img = cv2.medianBlur(img, 5)
        cimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cv2.imshow('detected circles', cimg)

        circles = cv2.HoughCircles(cimg, cv2.HOUGH_GRADIENT, 1.2, 10)
        if type(circles) != type(None):
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                # draw the outer circle
                cv2.circle(cimg, (i[0], i[1]), i[2], (0, 255, 0), 2)
                # draw the center of the circle
                cv2.circle(cimg, (i[0], i[1]), 2, (0, 0, 255), 3)

        cv2.imshow('detected circles', cimg)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            break

if __name__ == "__main__": main()