"""
Funciones implementadas para la comunicacion con la camara
"""
import numpy as np
import time
import cv2
import matplotlib.pyplot as plt
from Robot_Fase2.SerialLib import *
from Robot_Fase2.MotorLib import *


def comfirm_ACK(port):
    ACK = np.frombuffer( port.read(3), dtype= np.uint8)  # ACk
    print(np.size(ACK))
    print(ACK)
    ACK = chr(ACK[0]) + chr(ACK[1]) + chr(ACK[2])
    r = chr(ord(port.read(1)))
    print(r)
    if ACK == "ACK" :  # Si se recibio el ACK del comando
        return 1
    else:
        return 0

# def decode_TC(port):
#     buff = np.array([], dtype=np.uint8)
#     byte_int = port.read(1)
#     print(type(byte_int))
#     buff = np.append(buff, [byte_int], 0)
#     while byte != ord('\r'):
#         byte_int = port.read(1)
#         buff = np.append(buff, [byte_int], 0)
#     print(buff)
#     print(packet2string(buff))

def read_buffer(port):
    buff = np.frombuffer( port.read(1), dtype= np.uint8)  # Matriz temporal donde se guardara lo almacenado en el buffer
    packet_type = buff[0]  # Convertir a entero el Tipos : C, F(1), M, N y S
    if (packet_type == 1):  # Si el paquete es tipo F, se hace una lectura con mas tiempo de espera por paquete
        wait_time = 0.5  # si es mas grande, el buffer se llena y aparecen rayas de colores en la imagen recibida
        time.sleep(wait_time)
    else:
        wait_time = 0.05
        time.sleep(wait_time)
    buff = np.append(buff, [packet_type], 0)  # Agregar el tipo de paquete al comienzo del buffer
    if packet_type == ord('M'):
        size_max = 50
        while port.in_waiting != 0:  # Mientras hallan bytes por leer
            size_toread = port.in_waiting
            data = np.frombuffer(port.read(size_toread), dtype=np.uint8)
            for bytes in data:
                buff = np.append(buff, [bytes], 0)  # Se guardan como entero los datos recibidos
                if np.size(buff) > size_max:
                    return buff

    else:
        while port.in_waiting != 0:  # Mientras hallan bytes por leer
            size_toread = port.in_waiting
            data = np.frombuffer( port.read(size_toread), dtype= np.uint8)
            for bytes in data:
                buff = np.append(buff, [bytes], 0)  # Se guardan como entero los datos recibidos
            time.sleep(wait_time)

    return buff  # retornar lecutra del buffer serial


def idle_state(buff):  # Esta funcion verifica si la camara se encuentra en el estado idle examinando la data devuelta
    # por esta, y retorna el paquete recibido
    last_byte = buff[buff.size - 1]
    packet = buff
    #index,  = np.where(buff == ord(":")) # si el buffer contiene el caracter :
    if last_byte == ord(':'):
        packet = np.delete(buff, buff.size - 1, 0)  # Eliminar el caracter de estado idle del buffer
        return 1, packet
    else:
        return 0, packet

def TC_image(port):
    buff = read_buffer(port)  # leer buffer serial de entrada
    print("Buffer = {}".format(buff))
    idle, packet = idle_state(buff)
    print("Idle = {}".format(idle))
    print("Paquete = {}".format(packet))
    mx, my, x1, y1, x2, y2, pixels, confidence = decode(packet)
    print("mx={}, my={}, x1={}, y1={}, x2={}, y2={}".format(mx, my, x1, y1, x2, y2))
    print("pixels = {}, confidende = {}".format(pixels, confidence))
    force_idle(port)
    write(port, "DF")
    ACK = Micro_comfirm_ACK(port)
    while (ACK != 1):
        print("El micro no se ha resetiado")
    print("ACK command micro")
    ack = comfirm_ACK(port)
    buff = read_buffer(port)  # leer buffer serial de entrada
    idle, packet = idle_state(buff)
    print(idle)
    if idle == 1:
        image = decode(packet)
        image_raw = cv2.flip(image, -1)  # Reajuste a la imagen original vista por la camara
        cv2.rectangle(image_raw, (x1, y1), (x2, y2), (255, 0, 0), 1)
        cv2.circle(image_raw, (mx, my), 3, (255, 0, 0), -1)
        plt.figure("CMUcam1")
        image = cv2.flip(image, 0)
        plt.subplot(1, 2, 1)
        plt.title("Imagen cruda")
        plt.imshow(image_raw[..., ::-1])
        plt.subplot(1, 2, 2)
        plt.title("Imagen Flip")
        plt.imshow(image[..., ::-1])
        plt.show()
    else:
        force_idle(port)
        print("Error in idle, forzado a idle")

def TC(port):
    force_idle(port)
    time.sleep(1)
    buff = read_buffer(port)  # leer buffer serial de entrada
    # print("Buffer = {}".format(buff))
    # idle, packet = idle_state(buff)
    # print("Idle = {}".format(idle))
    # print("Paquete = {}".format(packet))
    # mx, my, x1, y1, x2, y2, pixels, confidence = decode(packet)
    # print("mx={}, my={}, x1={}, y1={}, x2={}, y2={}".format(mx, my, x1, y1, x2, y2))
    # print("pixels = {}, confidende = {}".format(pixels, confidence))
    return mx, my, x1, y1, x2, y2, pixels, confidence


def packet2string(packet):
    s = ""
    for data in packet:  # convertir buffer(enteros) a string
        s = s + chr(data)
    return s


def decode(packet):  # Decodificador de paquetes segun su tipo
    #packet = np.frombuffer(packet, dtype=np.uint8)
    print(packet[0])
    packet_type = chr(packet[0])  # Tipos : C, F, M, N y S
    last_byte = packet[packet.size - 1]

    if packet_type == chr(1) and last_byte == 3:  # Paquete tipo  F
        cols_index, = np.where(packet == 2)
        n_cols = cols_index.size
        n_rows = int((cols_index[1] - cols_index[0] - 1) / 3)  # restar indices de columnas -1, es el numero de filas*3
        print("Filas = {}, Columnas = {}".format(n_rows, n_cols))
        # Crear imagen
        image = np.zeros((n_rows, n_cols, 3), dtype="uint8")
        for col in range(n_cols):
            i = cols_index[col] - n_rows * 3  # indice del primer elemento (color r) en la trama de la columna
            for row in range(n_rows):
                r = i + row * 3  # indice del color rojo en cada fila de la trama columna enviada
                image[row, col] = [packet[r + 2], packet[r + 1], packet[r]]  # bgr (formato opencv)
        return image

    else:
        s = ""
        for data in packet:  # convertir buffer(enteros) a string
            s = s + chr(data)
        list_data = [int(s) for s in s.split() if s.isdigit()]
        data_array = np.array([0], dtype="uint8")
        for element in list_data:
            data_array = np.append(data_array, [element], 0)

        if packet_type == "C" and data_array[data_array.size - 1] == ord("\r"):  # Paquete tipo C
            x1, y1 = data_array[1], data_array[2]
            x2, y2, pixels, confidence = data_array[3], data_array[4], data_array[5], data_array[6]
            return x1, y1, x2, y2, pixels, confidence
        if packet_type == "M" and last_byte == ord("\r"):  # Paquete tipo M
            mx, my, x1, y1 = data_array[1], data_array[2], data_array[3], data_array[4]
            x2, y2, pixels, confidence = data_array[5], data_array[6], data_array[7], data_array[8]
            return mx, my, x1, y1, x2, y2, pixels, confidence

        if packet_type == "N" and last_byte == ord("\r"):  # Paquete tipo N
            spos, mx, my, x1, y1 = data_array[1], data_array[2], data_array[3], data_array[4], data_array[5]
            x2, y2, pixels, confidence = data_array[6], data_array[7], data_array[8], data_array[9]
            return spos, mx, my, x1, y1, x2, y2, pixels, confidence

        if packet_type == "S" and last_byte == ord("\r"):  # Paquete tipo M
            Rmean, Gmean, Bmean = data_array[1], data_array[2], data_array[3]
            Rdev, Gdev, Bdev = data_array[4], data_array[5], data_array[6]
            return Rmean, Gmean, Bmean, Rdev, Gdev, Bdev



def force_idle(port):  # Visualizar porq no recibe el ack
    idle = 0
    port.reset_input_buffer()
    write(port, "") # Escribo comando de paro
    ACK = Micro_comfirm_ACK(port)
    ack = comfirm_ACK(port)
    buff = read_buffer(port)
    idle, packet = idle_state(buff)
    while idle == 0:
        port.reset_input_buffer()
        write(port, "")  # finalizar stream
        ACK = Micro_comfirm_ACK(port)
        ack = comfirm_ACK(port)
        buff = read_buffer(port)
        idle, packet = idle_state(buff)
        time.sleep(0.1)

    print("Force idle = {}".format(idle))
    return

def write(port, command):
    packet_size = len(command)+1 +1# Comando + \r + comando micro
    Trama_Camara = np.ones(packet_size+2, dtype="uint8")
    Trama_Camara[0] = 0xff
    Trama_Camara[1] = packet_size
    Trama_Camara[2] = 0x02 #comando camara
    i = 3
    if command != "":
        for c in command:
            Trama_Camara[i] = ord(c)
            i = i +1
    Trama_Camara[i] = ord('\r')
    port.write(bytearray(Trama_Camara))

    return 0