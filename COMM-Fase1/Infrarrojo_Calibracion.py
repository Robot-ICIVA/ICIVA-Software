"""
Aqui se implemento tomar 16 mediciones para tener un promedio del infrarrojo
"""

import serial
import matplotlib.pyplot as plt
import numpy as np
import time

def open_port():
    ser = serial.Serial('COM3', 115200)

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
    print(Trama_Camara)
    port.write(bytearray(Trama_Camara))
    while True:
        anuncio = port.read(1)
        anuncio = ord(anuncio) # convertir en entero

        if anuncio == 0xff:# Se detecta el byte de anuncio de trama
            n_bytes = port.read(1)
            print(anuncio)
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

    while(Dif < 1):

        ADC = detect_data(port)
        ti = time.time()- T_Inicio
        y = ADC*3.1/(2**12-1) # Escalamiento, el voltaje de ref de adc es 3.1
        Amplitud_matrix = np.append(Amplitud_matrix, [y])
        Time_matrix = np.append(Time_matrix, [ti])
        T_Final = time.time()
        Dif = T_Final - T_Inicio

    close_port(port)
    # np.savetxt('InfraDC.out', Amplitud_matrix, fmt='%1.8e')
    print("Media = {}, Dev = {}, Nmediciones = {} ".format(np.mean(Amplitud_matrix), np.std(Amplitud_matrix, ddof=1),  Amplitud_matrix.shape[0]))
    # Graficas
    plt.figure()
    plt.scatter(Time_matrix, Amplitud_matrix)
    plt.title("Datos del sensor")
    plt.xlabel("Tiempo (s")
    plt.ylabel("Voltage(Volts)")
    plt.figure()
    plt.hist(Amplitud_matrix, bins='auto')
    plt.title("Histogramas de Infrarrojo Calibracion 2")
    plt.show()


if __name__ == "__main__": main()