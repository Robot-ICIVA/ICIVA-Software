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

def send(Motor, Dir, PWM, port):
    Trama_FREERUN = bytearray([0xf2, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00])
    velup = (0xff00 & PWM)>>8
    vellow = 0x00ff & PWM
    Trama_FREERUN[3] = Motor
    Trama_FREERUN[4] = Dir
    Trama_FREERUN[5] = velup
    Trama_FREERUN[6] = vellow
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