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
from CamaraLib import *
from SerialLib import *
import matplotlib.pyplot as plt
import cv2




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
    print("Inicio")
    time.sleep(10)
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
        print(distancia)
        if distancia >= 20.0 : # Si la distancia es menor a 15 cm
            PWM = 35000
            send_PWM(1, 1, 33000, port) # Enviar al motor izquierdo, PWM hacia adelante
            time.sleep(0.05)
            send_PWM(0, 1, PWM, port) # Enviar al motor derecho, PWM hacia adelante
        elif distancia < 20.0:
            PWM = 1
            send_PWM(1, 1, PWM, port)  # Enviar al motor izquierdo, PWM hacia adelante
            time.sleep(0.05)
            send_PWM(0, 1, PWM, port)  # Enviar al motor derecho, PWM hacia adelante



    print("Finished")
    close_port(port)



if __name__ == "__main__": main()
