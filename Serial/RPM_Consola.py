"""
* #################################INFORMACION###############################
*     Filename    : Trayectoria.py
*     Project     : ICIVA-ROBOT
*     Board       : Demoqe
*     Autor       : Luis Lujano (13-10775)
*     GitHub      : https://github.com/Lujano
*     Sensors     : ---
* ###########################################################################

Descripcion:

"""
import cv2
import numpy as np
import time
import serial


def open_port():
    ser = serial.Serial('COM12', 9800)

    return ser

def close_port(port):
    port.close()

def send(string_num):
    num = int(string_num)
    vellow = 0x0f&num
    velup = 0xf0&num
    Trama_FREERUN = (0xf1, 0x00, 0x01, velup, vellow)
    print(Trama_FREERUN)
    port.write(bytearray([0xf1, 0x00, 0x01, velup, vellow]))

def main():
    port = open_port()
    print("Bienvenido")
    command = "ds"
    while command != "q":
        if command.isdigit():
                print(command)
                send(command)


        print("\nEscriba un comando a enviar a la demo: (q para salir)")
        command = input()

    print("Finished")
    close_port()

if __name__ == "__main__": main()