"""#################################INFORMACION###############################
*     Filename    : ConsolaMotores.py
*     Project     : ICIVA
*     Board       : Demoqe
*     Autor       : Luis Lujano (13-10775)
*     GitHub      : https://github.com/Lujano
*     Sensors     : ----
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



def open_port():
    ser = serial.Serial('COM3', 115200) # o "COM12" en windows
    return ser


def close_port(port):
    port.close()


def read_buffer(port):
    buff = np.array([], dtype="uint8")  # Matriz temporal donde se guardara lo almacenado en el buffer
    time.sleep(0.1)
    while (port.in_waiting != 0): # Mientras hallan bytes por leer
        data = port.read(1)
        buff = np.append(buff, [data], 0) # Se guardan como entero los datos recibidos
        time.sleep(0.01)

    return buff # retornar lecutra del buffer serial

def packet2string(packet):
    s = ""
    for data in packet:  # convertir buffer(enteros) a string
        s = s + chr(ord(data))
    return s

def send(Motor, Dir, PWM, port):
    Trama_FREERUN = bytearray([0xff, 0x00, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00])
    velup = (0xff00 & PWM)>>8
    vellow = 0x00ff & PWM
    Trama_FREERUN[4] = Motor
    Trama_FREERUN[5] = Dir
    Trama_FREERUN[6] = velup
    Trama_FREERUN[7] = vellow
    number = (2**8)*velup+vellow
    print("Motor = {}, Dir= {}, PWM ={}".format(Motor, Dir, number))
    port.write(Trama_FREERUN)

    return


def main():
    port = open_port()
    Motor = " "
    Dir = " "
    command = " "
    Estado = "Motor" # Dir, RPM
    print("Bienvenido")
    while command != "q":
        if Estado == "Motor":
            print("\nMotor  a utilizar (0 o 1)= ") # 0 es derecha, 1  izquierda (visto desde atras)
            Motor = input()
            if Motor == "0" or Motor == "1":
                Estado = "Dir"
            elif Motor == "q":
                command = "q"
            else:
                print("Error")

        if Estado == "Dir":
            print("\nDireccion  del Motor (0 o 1)= ") # 1 adelante, 0 atras
            Dir = input()
            if Dir == "0" or Dir == "1":
                Estado = "RPM"
            else:
                print("Error")
        if Estado == "RPM":
            print("\nPWM  del Motor (Max 65535)= ") # RPM
            RPM = input()
            if RPM.isdigit():
                num = int(RPM)
                if (num < 65535) & (num > 0): #  Cabecera, cmd, Motor, dir, RpmH, RpmL
                    Motor = int(Motor)
                    Dir = int(Dir)
                    send(Motor, Dir, num ,  port)
                    Estado = "Motor"
                else:
                     print("Error")

    print("Finished")
    close_port(port)



if __name__ == "__main__": main()
