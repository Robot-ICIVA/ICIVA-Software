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
    Este programa estra diseñado para que el robot matenga una distancia fija del objetivo (menor a 15 cm) empleando 
    para ello un sensor infrarrojo de distancia, y  una camara (CMUcamv2.1)
"""


import time
import numpy as np
from Robot_Fase2.CamaraLib import *
from Robot_Fase2.SerialLib import *
import matplotlib.pyplot as plt
import cv2



def open_port ():
    ser = serial.Serial('COM3', 130000, timeout=1) # o "COM12" en windows
    return ser


def close_port(port):
    port.close()


def read_buffer(port):
    buff = np.array([], dtype="uint8")  # Matriz temporal donde se guardara lo almacenado en el buffer
    time.sleep(0.1)
    while port.in_waiting != 0: # Mientras hallan bytes por leer
        data = port.read(1)
        buff = np.append(buff, [data], 0) # Se guardan como entero los datos recibidos
        time.sleep(0.01)

    return buff # retornar lecutra del buffer serial

def packet2string(packet):
    s = ""
    for data in packet:  # convertir buffer(enteros) a string
        s = s + chr(ord(data))
    return s

"""def move_lateral(move_band):

    if move_band == 0: # Do nothing

    elif move_band == 1: #

    elif move_band == 2:
"""



def detect_data(port):
    packet_size = 0
    Trama_Camara = np.ones(packet_size+4, dtype="uint8")
    Trama_Camara[0] = 0xff
    Trama_Camara[1] = 0x00
    Trama_Camara[2] = 0
    Trama_Camara[3] = 0x03 #comando ADC
    port.write(bytearray(Trama_Camara))
    time.sleep(0.05)
    anuncio = port.read(1)
    anuncio = ord(anuncio) # convertir en entero

    if anuncio == 0xff:# Se detecta el byte de anuncio de trama
        n_bytes = port.read(1)
        ADC_up = port.read(1)
        ADC_low = port.read(1)
        ADC = (2 ** 7) * ord(ADC_up) + ord(ADC_low)

    else:
        ADC = 0
    return ADC


# La CAMARA tiene una resolucion de 80x143
cx = 40
cy = 70
offsety= 20
offsetx = 20
move_band = 0 # bandera de movimiento si es 1, se  debe mover a la dercha. Si es 2 se mueve hacia la izquierda.
 # Si es 0  no se mueve
def main():
    port = open_port()
    Motor = " "
    Dir = " "
    command = " "
    Estado = "Motor" # Dir, RPM

    T_Inicio = time.time()
    T_Final = time.time()
    Dif = T_Final-T_Inicio
    poly_infra = np.loadtxt('../Calibracion/Polinomio_Ajuste_Infra2.out') #Calibracion del sharp
    poly = np.poly1d(poly_infra)

    print("Inicio")
    Mircro_reset(port)
    ACK = Micro_comfirm_ACK(port)
    while(ACK != 1):
        print("El micro no se ha resetiado")
    print("Micro reseteado")
    force_idle(port)
    while True:
        port.reset_input_buffer()
        command = "TC 20 100 30 100 30 100"
        write(port, command)
        ACK = Micro_comfirm_ACK(port)
        if ACK == 1:
            print("Comando recibido")
        else:
            print("Comando no recibido")
        ack = comfirm_ACK(port)
        print("ACK = {}".format(ack))
        ack = comfirm_ACK(port)
        TC_image(port)
        time.sleep(2)
        # buff = read_buffer(port)  # leer buffer serial de entrada
        # time.sleep(0.1)
        # idle, packet = idle_state(buff)
        # print("aq")
        # mx, my, x1, y1, x2, y2, pixels, confidence = decode(packet)
        # print("mx={}, my={}, x1={}, y1={}, x2={}, y2={}".format(mx, my, x1, y1, x2, y2))
        # print("pixels = {}, confidende = {}".format(pixels, confidence))
        # force_idle(port)
        #
        # if mx < cx-offsetx : # el objeto se encuentra a la izquierda
        #     move_band = 2
        # elif mx > cx+offset: # el objeto se encuenra a la derecha
        #     move_band = 1
        # else:
        #     move_band = 0
        #
        #
        #
        # PWM = 35000
        # #send_PWM(1, 0, PWM, port)  # Enviar al motor izquierdo, PWM hacia adelante
        # time.sleep(0.05)
        # #send_PWM(0, 0, PWM, port)  # Enviar al motor derecho, PWM hacia adelante



    print("Finished")
    close_port(port)



if __name__ == "__main__": main()
