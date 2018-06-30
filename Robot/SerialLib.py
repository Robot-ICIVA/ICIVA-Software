import serial
import numpy as np
import time
def open_port():

    ser = serial.Serial('COM8', 130000) # o "COM12" en windows
    return ser


def close_port(port):
    port.close()

def packet2string(packet):
    s = ""
    for data in packet:  # convertir buffer(enteros) a string
        s = s + chr(ord(data))
    return s

def send_PWM(Motor, Dir, PWM, port):
    Trama_FREERUN = bytearray([0xff, 0x00, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00])
    velup = (0xff00 & PWM)>>8
    vellow = 0x00ff & PWM
    Trama_FREERUN[4] = Motor
    Trama_FREERUN[5] = Dir # 1 hacia adelante
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
