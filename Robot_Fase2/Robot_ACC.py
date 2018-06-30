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
    Este programa estra diseñado para que el robot matenga una distancia fija del objetivo (menor a 15 cm).
    Este programa permite la comunicacion via serial entre la camara CMUcam1 y la pc mediante el empleo de python (3.6).
    Commandos:
    -Los commandos se ingresan por consola.
    Data:
    - Implementada la decodificacion de todos los tipos de paquete enviados por la camara
    - Se permite la visualizacion de los paquetes recibidos en formato string y como arreglo de enteros
    - Se puede visualizar la imagen tomada por la camara mediante el comando DF, y matplotlib (tiempo aprox 6 seg)
"""
from Robot_Fase2.MotorLib import *
from Robot_Fase2.SerialLib import *
import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import cv2

import MotorLib


def read_buffer(port):
    buff = np.array([], dtype="uint8")  # Matriz temporal donde se guardara lo almacenado en el buffer
    while (port.in_waiting != 0): # Mientras hallan bytes por leer
        data = port.read(1)
        buff = np.append(buff, [data], 0) # Se guardan como entero los datos recibidos


    return buff # retornar lecutra del buffer serial

def packet2string(packet):
    s = ""
    for data in packet:  # convertir buffer(enteros) a string
        s = s + chr(ord(data))
    return s

def write(port, command):
    packet_size = len(command)+1+1 # Comando_camara + \r + comando_micro
    Trama_Camara = np.ones(packet_size+2, dtype="uint8")
    # Cabecera
    Trama_Camara[0] = 0xff
    Trama_Camara[1] = packet_size

    # data
    Trama_Camara[2] = 0x02 #comando camara
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
    Motor = " "
    Dir = " "
    command = " "
    Estado = "Motor" # Dir, RPM

    T_Inicio = time.time()
    T_Final = time.time()
    Dif = T_Final-T_Inicio
    poly_infra_dir = np.loadtxt('../Calibracion/Polinomio_Ajuste_Infra2.out') #Calibracion del sharp
    poly_rd_dir = np.loadtxt('../Calibracion/Polinomio_RD.out')
    poly_ri_dir = np.loadtxt('../Calibracion/Polinomio_RI.out')
    poly = np.poly1d(poly_infra_dir)
    poly_rd =np.poly1d(poly_rd_dir)
    poly_ri =np.poly1d(poly_ri_dir)
    print("Inicio")
    Mircro_reset(port)

    ACK = Micro_comfirm_ACK(port)
    while(ACK != 1):
        port.reset_input_buffer()
        Mircro_reset(port)
        ACK = Micro_comfirm_ACK(port)
        print("El micro no se ha resetiado")
    print("Micro reseteado")

    while True:
        Amplitud_matrix = np.array([])
        Amplitud_filtrada = np.array([])

        # Mido el Voltaje del adc y filtro las medidas para obtener el valor de distancia
        T_Inicio = time.time()
        T_Final = time.time()
        Dif = T_Final - T_Inicio
        while (Dif < 0.2):
            ADC = detect_data(port)
            y = ADC * 3.1 / (2 ** 12 - 1)  # Escalamiento, el voltaje de ref de adc es 3.1
            Amplitud_matrix = np.append(Amplitud_matrix, [y])
            T_Final = time.time()
            Dif = T_Final - T_Inicio

        Valor_min = Amplitud_matrix[np.argmin(Amplitud_matrix, 0)]
        indices, = np.where(Amplitud_matrix < (Valor_min + Valor_min * 0.1))

        for i in indices: # Filtrado
            Amplitud_filtrada = np.append(Amplitud_filtrada, [Amplitud_matrix[i]])

        distancia = poly(np.mean(Amplitud_filtrada)) # Distancia medida
        print("Distancia = {0:0.2f} cm".format(distancia))
        if distancia >= 25.0 : # Si la distancia es menor a 15 cm
            delta = distancia-25 # Cuan cerca estoy de 25. Si estoy muy cerca disminuir velocidad
            if (delta <= 5 and delta >= 0):
                PWM_RD = 22000
            elif (delta > 5 and delta < 10):
                PWM_RD = 30000
            else:
                PWM_RD = 42000
            PWM_RI = PWM_RD
            #PWM_RD = 60000#int(round(poly_rd(50)))+1000
            #PWM_RI = 60000#int(round(poly_ri(50)))
            send_PWM(1, PWM_RI, 1, PWM_RD, port) # Enviar al motor izquierdo, PWM hacia adelante
            ACK = Micro_comfirm_ACK(port)
            if ACK == 1:
                print("Comando recibido")
            else:
                print("Comando no recibido")
        elif distancia < 15.0 and distancia >8.0:
            delta = 15-distancia
            if(delta <=5 and delta >= 0):
                PWM_RD = 22000
            elif(delta > 5 and delta <10):
                PWM_RD = 32000
            else:
                PWM_RD = 42000
            PWM_RI = PWM_RD
            #PWM_RD = 60000#int(round(poly_rd(50)))+5000
            #PWM_RI = 60000#Int(round(poly_ri(50)))
            send_PWM(0, PWM_RI, 0, PWM_RD, port)  # Enviar al motor izquierdo, PWM hacia adelante ACK = Micro_comfirm_ACK(port)
        if ACK == 1:
            print("Comando recibido")
        else:
            print("Comando no recibido")

        # else:
        #     send_PWM(0, 0, 0, 0, port)  # Enviar al motor izquierdo, PWM hacia adelante




    print("Finished")
    close_port(port)



if __name__ == "__main__": main()
