"""
Aqui se implemento tomar 16 mediciones para tener un promedio del infrarrojo
"""

import serial
import matplotlib.pyplot as plt
import numpy as np
import time

def open_port():
    ser = serial.Serial('COM8', 38400)

    return ser

def close_port(port):
    port.close()

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
    i = 0.00
    y = 0.00
    print("Inicio")
    Amplitud_matrix = np.array([])
    Time_matrix = np.array([])
    T_Inicio = time.time()
    T_Final = time.time()
    Dif = T_Final-T_Inicio
    poly_infra = np.loadtxt('../Calibracion/Polinomio_Ajuste_Infra2.out')
    poly = np.poly1d(poly_infra)
    while(Dif < 1):

        ADC = detect_data(port)
        ti = time.time()- T_Inicio
        Time_matrix = np.append(Time_matrix, [ti])
        y = ADC*3.1/(2**12-1) # Escalamiento, el voltaje de ref de adc es 3.1
        Amplitud_matrix = np.append(Amplitud_matrix, [y])
        Valor_min = Amplitud_matrix[np.argmin(Amplitud_matrix, 0)]
        indices, = np.where(Amplitud_matrix < (Valor_min + Valor_min * 0.1))
        T_filtrado = np.array([])
        Amplitud_filtrada = np.array([])
        for i in indices:
            T_filtrado =   np.append(T_filtrado, [Time_matrix[i]])
            Amplitud_filtrada = np.append(Amplitud_filtrada, [Amplitud_matrix[i]])

        T_Final = time.time()
        Dif = T_Final - T_Inicio


    print(poly(np.mean(Amplitud_filtrada)))



if __name__ == "__main__": main()