"""#################################INFORMACION###############################
*     Filename    : ConsolaCMU.py
*     Project     : Autonomus
*     Board       : Demoqe
*     Autor       : Luis Lujano (13-10775)
*     GitHub      : https://github.com/Lujano
*     Sensors     : CMUcam1 (camera)
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
#import winsound


def open_port():
    ser = serial.Serial('COM3', 115200) # o "COM12" en windows
    return ser


def close_port(port):
    port.close()

def comfirm_ACK(port):
    ACK = port.read(3)#ACk
    ACK = chr(ACK[0])+chr(ACK[1])+chr(ACK[2])
    r = chr(ord(port.read(1)))
    if ACK == "ACK" and r == "\r": # Si se recibio el ACK del comando
        return 1
    else:
        return 0

def read_buffer(port):
    buff = np.array([], dtype="uint8")  # Matriz temporal donde se guardara lo almacenado en el buffer
    packet_type = ord(port.read(1))  #  Convertir a entero el Tipos : C, F(1), M, N y S
    if (packet_type == 1): # Si el paquete es tipo F, se hace una lectura con mas tiempo de espera por paquete
        wait_time = 0.2 # si es mas grande, el buffer se llena y aparecen rayas de colores en la imagen recibida
        time.sleep(wait_time)
    else:
        wait_time = 0.1
        time.sleep(wait_time)
    time.sleep(wait_time)
    buff = np.append(buff, [packet_type], 0) # Agregar el tipo de paquete al comienzo del buffer
    while (port.in_waiting != 0): # Mientras hallan bytes por leer
        size_toread = port.in_waiting
        data = port.read(size_toread)
        for bytes in data:
            buff = np.append(buff, [bytes], 0) # Se guardan como entero los datos recibidos
        time.sleep(wait_time)

    return buff # retornar lecutra del buffer serial

def idle_state(buff): # Esta funcion verifica si la camara se encuentra en el estado idle examinando la data devuelta
    # por esta, y retorna el paquete recibido
    last_byte = buff[buff.size-1]
    packet = buff
    if (chr(last_byte) == ":"):
        packet = np.delete(buff, buff.size-1, 0) # Eliminar el caracter de estado idle del buffer
        return  1, packet
    else:
        return  0, packet
def packet2string(packet):
    s = ""
    for data in packet:  # convertir buffer(enteros) a string
        s = s + chr(data)
    return s

def decode(packet): # Decodificador de paquetes segun su tipo
    packet_type = chr(packet[0]) #  Tipos : C, F, M, N y S
    last_byte = packet[packet.size-1]

    if packet_type == chr(1) and last_byte == 3:           # Paquete tipo  F
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
        s =""
        for data in packet: # convertir buffer(enteros) a string
            s = s+chr(data)
        list_data = [int(s) for s in s.split() if s.isdigit()]
        data_array = np.array([0], dtype="uint8")
        for element in list_data:
            data_array = np.append(data_array, [element], 0)
            
        if packet_type == "C" and data_array[data_array.size - 1] == ord("\r"):  # Paquete tipo C
            x1, y1 = data_array[1], data_array[2]
            x2, y2, pixels, confidence = data_array[3], data_array[4], data_array[5], data_array[6]
            return x1, y1, x2, y2, pixels, confidence
        if packet_type  == "M" and last_byte  == ord("\r"): # Paquete tipo M
            mx, my, x1, y1 = data_array[1], data_array[2], data_array[3], data_array[4]
            x2, y2, pixels, confidence = data_array[5], data_array[6], data_array[7], data_array[8]
            return mx, my, x1, y1, x2, y2, pixels, confidence

        if packet_type  == "N" and last_byte  == ord("\r"): # Paquete tipo N
            spos, mx, my, x1, y1 = data_array[1], data_array[2], data_array[3], data_array[4], data_array[5]
            x2, y2, pixels, confidence = data_array[6], data_array[7], data_array[8], data_array[9]
            return spos, mx, my, x1, y1, x2, y2, pixels, confidence

        if packet_type  == "S" and last_byte  == ord("\r"): # Paquete tipo M
            Rmean, Gmean, Bmean = data_array[1], data_array[2], data_array[3]
            Rdev, Gdev, Bdev = data_array[4], data_array[5], data_array[6]
            return Rmean, Gmean, Bmean, Rdev, Gdev, Bdev
def force_idle(port): # Visualizar porq no recibe el ack
    idle = 0
    while idle == 0 :
        port.reset_input_buffer()
        port.write("\r".encode("utf-8"))  # finalizar stream
        buff = read_buffer(port)  # leer buffer serial de entrada
        idle, packet = idle_state(buff)

    print("Force idle = {}".format(idle))
    return
def write(port, command):
    packet_size = len(command)+1 # Comando + \r
    Trama_Camara = np.ones(packet_size+4, dtype="uint8")
    Trama_Camara[0] = 0xff
    Trama_Camara[1] = 0x00
    Trama_Camara[2] = packet_size
    Trama_Camara[3] = 0x02 #comando camara
    i = 4
    for c in command:
        Trama_Camara[i] = ord(c)
        i =i +1
    Trama_Camara[i] = ord('\r')

    print(Trama_Camara)
    port.write(bytearray(Trama_Camara))

    return

def main():
    port = open_port()
    port.reset_input_buffer()
    print("Bienvenido\nEscriba el comando a enviar a la camara: (q para salir)")
    command = input()
    write(port, command)
    while command != "q":
        ack = comfirm_ACK(port)
        print("ACK = {}".format(ack))
        if ack == 1:
            tini = time.time()
            buff = read_buffer(port) # leer buffer serial de entrada
            print("Buffer = {}".format(buff))
            print("Tiempo de lectura = {} s".format(time.time() - tini))
            idle, packet = idle_state(buff)
            print("Idle = {}".format(idle))
            print("Paquete = {}".format(packet))

            if command == "DF": # DF\r, Dump File
                if idle == 1:
                    image = decode(packet)
                    image_raw = cv2.flip(image, -1) # Reajuste a la imagen original vista por la camara
                    plt.figure("CMUcam1")
                    image = cv2.flip(image, 0)
                    plt.subplot(1, 2, 1)
                    plt.title("Imagen cruda")
                    plt.imshow(image_raw[..., ::-1])
                    plt.subplot(1, 2, 2)
                    plt.title("Imagen Flip")
                    plt.imshow(image[..., ::-1])
                    plt.show()

            elif command == "": # Comando \r
                print("HO!")

            elif command == "GV":
                print("packet = "+packet2string(packet))
            elif command == "GM":
                port.write("\r".encode("utf-8")) # finalizar stream
                print("packet = " + packet2string(packet))
                if idle == 0:
                    Rmean, Gmean, Bmean, Rdev, Gdev, Bdev = decode(packet)
                    print("Rmean={}, Gmean={}, Bmean={}, Rdev={}, Gdev={}, Bdev={}".format(Rmean, Gmean, Bmean, Rdev, Gdev, Bdev))
                    force_idle(port)
            elif command[:2] ==  "TC":
                print("packet = " + packet2string(packet))
                if idle == 0:
                    mx, my, x1, y1, x2, y2, pixels, confidence = decode(packet)
                    print("mx={}, my={}, x1={}, y1={}, x2={}, y2={}".format(mx, my, x1, y1, x2, y2))
                    print("pixels = {}, confidende = {}".format(pixels, confidence))
                    force_idle(port)
                    port.write(("DF" + "\r").encode("utf-8"))
                    ack = comfirm_ACK(port)
                    buff = read_buffer(port)  # leer buffer serial de entrada
                    idle, packet = idle_state(buff)
                    if idle == 1:
                        image = decode(packet)
                        image_raw = cv2.flip(image, -1)  # Reajuste a la imagen original vista por la camara
                        cv2.rectangle(image_raw, (x1, y1), (x2, y2), (255, 0, 0), 1)
                        cv2.circle ( image_raw, (mx, my), 3, (255, 0, 0), -1)
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
                print("packet = " + packet2string(packet))
                print("Other command")
        else:
            print("Error in ack")

        port.reset_input_buffer()
        print("\nEscriba otro comando a enviar a la camara: (q para salir)")
        command = input()
        write(port, command)

    print("Finished")
    close_port(port)

if __name__ == "__main__": main()
