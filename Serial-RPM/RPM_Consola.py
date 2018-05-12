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
    ser = serial.Serial('COM3', 9600)

    return ser

def close_port(port):
    port.close()

def send(string_num, port):
    num = int(string_num)

    print(num)
    if (num < 65535) & (num > 0):
        Trama_FREERUN = bytearray([0xf1, 0x00, 0x01, 0x00, 0x00])
        velup = (0x0000ff00 & num)>>8
        vellow = 0x000000ff & num
        Trama_FREERUN[3] = velup
        Trama_FREERUN[4] = vellow
        number = (2**8)*velup+vellow
        print("Numero es = {}".format(number))
        port.write(Trama_FREERUN)
    else:
        print ("\nError")

    return

def main():
    port = open_port()
    print("Bienvenido")
    command = "ds"
    while command != "q":
        if command.isdigit():
                send(command,port)


        print("\nEscriba un comando a enviar a la demo: (q para salir)")
        command = input()

    print("Finished")
    close_port(port)

if __name__ == "__main__": main()