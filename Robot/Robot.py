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
    ser = serial.Serial('COM8', 115200) # o "COM12" en windows
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

def detect_data(port):
    packet_size = 0
    Trama_Camara = np.ones(packet_size+4, dtype="uint8")
    Trama_Camara[0] = 0xff
    Trama_Camara[1] = 0x00
    Trama_Camara[2] = 0
    Trama_Camara[3] = 0x03 #comando ADC
    port.write(bytearray(Trama_Camara))
    while True:
        anuncio = port.read(1)
        anuncio = ord(anuncio) # convertir en entero

        if anuncio == 0xff:# Se detecta el byte de anuncio de trama
            n_bytes = port.read(1)
            ADC_up = port.read(1)
            ADC_low = port.read(1)
            ADC = (2 ** 7) * ord(ADC_up) + ord(ADC_low)
            break
    return ADC


def main():
    port = open_port()
    Motor = " "
    Dir = " "
    command = " "
    Estado = "Motor" # Dir, RPM

    T_Inicio = time.time()
    T_Final = time.time()
    Dif = T_Final-T_Inicio
    poly_infra = np.loadtxt('../Calibracion/Polinomio_Ajuste_Infra2.out')
    poly = np.poly1d(poly_infra)
    while True:
        Amplitud_matrix = np.array([])
        Amplitud_filtrada = np.array([])
        while (Dif < 1):
            ADC = detect_data(port)
            y = ADC * 3.1 / (2 ** 12 - 1)  # Escalamiento, el voltaje de ref de adc es 3.1
            Amplitud_matrix = np.append(Amplitud_matrix, [y])
            T_Final = time.time()
            Dif = T_Final - T_Inicio

        Valor_min = Amplitud_matrix[np.argmin(Amplitud_matrix, 0)]
        indices, = np.where(Amplitud_matrix < (Valor_min + Valor_min * 0.1))

        for i in indices:
            Amplitud_filtrada = np.append(Amplitud_filtrada, [Amplitud_matrix[i]])
        distancia = poly(np.mean(Amplitud_filtrada))
        print(distancia)
        T_Inicio = time.time()
        T_Final = time.time()
        Dif = T_Final - T_Inicio

    print("Finished")
    close_port(port)



if __name__ == "__main__": main()
