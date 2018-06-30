"""#################################INFORMACION###############################
*     Filename    : ConsolaCMU.py
*     Project     : ICIVA
*     Board       : Demoqe
*     Autor       : Luis Lujano (13-10775)
*     GitHub      : https://github.com/Lujano
*     Sensors     : CMUcam1 (camera)
* ###########################################################################"""

"""
Descripcion:
    Este programa permite la comunicacion via serial entre la camara CMUcam1 y la pc mediante el empleo de python (3.6).
    Commandos:
    -Los commandos se ingresan por consola.
    Data:
    - Implementada la decodificacion de todos los tipos de paquete enviados por la camara
    - Se permite la visualizacion de los paquetes recibidos en formato string y como arreglo de enteros
    - Se puede visualizar la imagen tomada por la camara mediante el comando DF, y matplotlib (tiempo aprox 6 seg)
"""

import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import cv2
from CamaraLib import *
#import winsound


def open_port():
    ser = serial.Serial('COM3', 130000) # o "COM12" en windows
    return ser


def close_port(port):
    port.close()


def write(port, command):
    packet_size = len(command)+1 # Comando + \r
    Trama_Camara = np.ones(packet_size+4, dtype="uint8")
    Trama_Camara[0] = 0xff
    Trama_Camara[1] = 0x00
    Trama_Camara[2] = packet_size
    Trama_Camara[3] = 0x02 #comando camara
    i = 4
    if command != "":
        for c in command:
            Trama_Camara[i] = ord(c)
            i =i +1
    Trama_Camara[i] = ord('\r')

    print(Trama_Camara)
    port.write(bytearray(Trama_Camara))

    return

def main():
    port = open_port()
    port.reset_input_buffer()
    print("Bienvenido\nEscriba el comando a enviar a la camara: (q para salir)")
    command = input()
    write(port, command)
    while command != "q":
        ack = comfirm_ACK(port)
        print("ACK = {}".format(ack))
        if ack == 1:
            tini = time.time()
            buff = read_buffer(port) # leer buffer serial de entrada
            print("Buffer = {}".format(buff))
            print("Tiempo de lectura = {} s".format(time.time() - tini))
            idle, packet = idle_state(buff)
            print("Idle = {}".format(idle))
            print("Paquete = {}".format(packet))

            if command == "DF": # DF\r, Dump File
                if idle == 1:
                    image = decode(packet)
                    image_raw = cv2.flip(image, -1) # Reajuste a la imagen original vista por la camara
                    plt.figure("CMUcam1")
                    image = cv2.flip(image, 0)
                    plt.subplot(1, 2, 1)
                    plt.title("Imagen cruda")
                    plt.imshow(image_raw[..., ::-1])
                    plt.subplot(1, 2, 2)
                    plt.title("Imagen Flip")
                    plt.imshow(image[..., ::-1])
                    plt.show()

            elif command == "": # Comando \r
                print("HO!")

            elif command == "GV":
                print("packet = "+packet2string(packet))
            elif command == "GM":

                write(port, "") # finalizar stream
                print("packet = " + packet2string(packet))
                if idle == 0:
                    Rmean, Gmean, Bmean, Rdev, Gdev, Bdev = decode(packet)
                    print("Rmean={}, Gmean={}, Bmean={}, Rdev={}, Gdev={}, Bdev={}".format(Rmean, Gmean, Bmean, Rdev, Gdev, Bdev))
                    force_idle(port)
            elif command[:2] ==  "TC":
                print("packet = " + packet2string(packet))
                if idle == 0:
                    mx, my, x1, y1, x2, y2, pixels, confidence = decode(packet)
                    print("mx={}, my={}, x1={}, y1={}, x2={}, y2={}".format(mx, my, x1, y1, x2, y2))
                    print("pixels = {}, confidende = {}".format(pixels, confidence))
                    force_idle(port)

                write(port, "DF")
                ack = comfirm_ACK(port)
                buff = read_buffer(port)  # leer buffer serial de entrada
                idle, packet = idle_state(buff)
                print(idle)
                if idle == 1:
                    image = decode(packet)
                    image_raw = cv2.flip(image, -1)  # Reajuste a la imagen original vista por la camara
                    cv2.rectangle(image_raw, (x1, y1), (x2, y2), (255, 0, 0), 1)
                    cv2.circle ( image_raw, (mx, my), 3, (255, 0, 0), -1)
                    plt.figure("CMUcam1")
                    image = cv2.flip(image, 0)
                    plt.subplot(1, 2, 1)
                    plt.title("Imagen cruda")
                    plt.imshow(image_raw[..., ::-1])
                    plt.subplot(1, 2, 2)
                    plt.title("Imagen Flip")
                    plt.imshow(image[..., ::-1])
                    plt.show()


            else:
                print("packet = " + packet2string(packet))
                print("Other command")
        else:
            print("Error in ack")

        port.reset_input_buffer()
        print("\nEscriba otro comando a enviar a la camara: (q para salir)")
        command = input()
        write(port, command)

    print("Finished")
    close_port(port)




if __name__ == "__main__": main()
