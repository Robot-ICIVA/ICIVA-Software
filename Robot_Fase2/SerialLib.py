import serial
import numpy as np
import time

def open_port():

    ser = serial.Serial('COM3', 130000, timeout=1) # o "COM12" en windows 1 segundo de timeout
    return ser


def close_port(port):
    port.close()

def Micro_comfirm_ACK(port):
    ACK = np.frombuffer( port.read(2), dtype= np.uint8) # ACk, timeout
    if np.size(ACK) == 0:
        return 0
    else:
        if ACK[0] == 0xff and ACK[1]== 0x00:  # Si se recibio el ACK del comando
            return 1
        else:
            return 0

def Mircro_reset(port): # funcion que resetea el micro
    port.write(bytearray([0xff, 0x01, 0x04])) # Trama reset
    return 0

def detect_data(port):
    packet_size = 1 # solo comando del micro sera enviado
    Trama_Camara = np.ones(packet_size+2, dtype="uint8")
    Trama_Camara[0] = 0xff
    Trama_Camara[1] = 0x01
    Trama_Camara[2] = 0x03 #comando ADC
    port.write(bytearray(Trama_Camara))
    ACK = Micro_comfirm_ACK(port)
    while ACK != 1:
        port.reset_input_buffer()
        port.write(bytearray(Trama_Camara))
        ACK = Micro_comfirm_ACK(port)

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
