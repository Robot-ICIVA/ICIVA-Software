import time
import requests
import json
import matplotlib.pyplot as plt
import numpy as np
import cv2
from Trayectorias.lib.SerialLib import open_port, close_port, Micro_reset, Micro_comfirm_ACK
import sys
import os
from Trayectorias.lib.MotorLib import align, move_forward, control_w, rpd2pwm, send_PWM
#plt.ion() # activar plot interactivo


"""
Control Proporcional para el ICIVA
"""

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def norm_vector(vector):
    """ Returns the norm of the vector.  """
    return np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    angle_1 = np.degrees(np.arctan2(v1_u[1], v1_u[0]))
    angle_2 = np.degrees(np.arctan2(v2_u[1], v2_u[0]))
    angle = np.arccos(np.dot(v1_u, v2_u))

    if angle_1 < 0:
        angle_1 = angle_1+360
    if angle_2 < 0:
        angle_2 = angle_2+360

    angle_rest = angle_1 - angle_2

    if angle_rest < 180 and angle_rest > 0: # el vector 1 esta por encima del vector 2
        return -np.degrees(angle)
    elif angle_rest < -180 : # el vector 1 esta por encima del vector 2
        return -np.degrees(angle)
    else:
        return np.degrees(angle)

def circles_robot(response): # busca los circulos del robot en el paquete JSON
    x_cyan = None
    y_cyan = None
    radius_cyan = None
    x_yellow = None
    y_yellow = None
    radius_yellow = None
    for list in response:
        if 'CYAN' in list:
            x_cyan = list[0][0]*100
            y_cyan = list[0][1]*100
            radius_cyan = list[1]*100
        elif 'BLUE' in list:
            x_cyan = list[0][0]*100
            y_cyan = list[0][1]*100
            radius_cyan = list[1]*100
        elif 'YELLOW' in list:
            x_yellow = list[0][0]*100
            y_yellow = list[0][1]*100
            radius_yellow = list[1]*100


    return x_cyan, y_cyan, radius_cyan, x_yellow, y_yellow, radius_yellow

def get_balls(response): # busca los circulos del robot en el paquete JSON

    for list in response:
        if 'RED' in list:
            x_ball = list[0][0]*100
            y_ball = list[0][1]*100
            radius_ball = list[1]*100

    return (x_ball, y_ball, radius_ball)

def Request(ip, port_server): # el servo que controla phi (plano xy)
    url_FREERUN = "http://"+ ip +":"+ port_server

    try:


        req = requests.get(url_FREERUN)
        response = req.json()


    except:
        print("Error Sending Data")
        return None

    return response


def main():
    port = open_port() # puerto bluetooth

    Rapidez_prom_array = np.array([])



    # Trayectoria del robot
    tr_robot = np.zeros([0, 2]) # array para guardar x y y del robot

    # Trayectoria deseada
    tr_obj = np.zeros([0, 2])  # array para guardar x y y de la trayectoria deseada
    poly = np.poly1d([0.01, 0, 0])
    x_array = np.arange(0, 100, 8)
    y_array = poly(x_array)

    # Obtener la posicion del robot
    x_cyan = None
    x_yellow = None
    while (x_cyan == None) and (x_yellow == None):
        response = Request("127.0.0.1", "8000")
        x_cyan, y_cyan, radius_cyan, x_yellow, y_yellow, radius_yellow = circles_robot(response)
        if x_cyan != None and x_yellow != None:
            # Vectors
            robot_pos = np.array(
                [x_yellow + (x_cyan - x_yellow) / 2, y_yellow + (y_cyan - y_yellow) / 2])  # posicion del robot

    robot_ini = robot_pos

    # Ajustar Trayectoria deseada segun posicion inicial
    for i in range(np.size(x_array)):
        if x_array[i]<100.0 and y_array[i] < 100.0: # limitar trayectoria
            tr_obj = np.append(tr_obj, [[x_array[i]+robot_ini[0], y_array[i]+robot_ini[1]]], 0)

   # Reseteo de micro
    Micro_reset(port)
    ACK = Micro_comfirm_ACK(port)
    objp = np.zeros((100, 3), np.uint8)
    while (ACK != 1):
        port.reset_input_buffer()
        Micro_reset(port)
        ACK = Micro_comfirm_ACK(port)
        print("El micro no se ha resetiado")
    print("Micro reseteado")

    #Imagen para interrupcion
    objp = np.zeros((100, 3), np.uint8)


    # Grafica trayectoria deseada
    plt.figure(1)
    plt.xlim([0, 100])
    plt.ylim([0, 100])
    plt.scatter(tr_obj[:, 0], tr_obj[:, 1], c='g')
    plt.ylabel("y (cm)")
    plt.xlabel("x (cm)")
    plt.title("Trayectoria deseada")
    plt.waitforbuttonpress(5)
    plt.close(1)
    # Pedir rapidez con la cual se recorrera la treayectoria
    check = 0
    rapidez_string = input("Introduzcala rapidez (cm/s):\n")

    rapidez = float(rapidez_string)
    # while check != 1:
    #     if rapidez_string.isdigit() != 0:
    #         rapidez = int(rapidez_string)
    #         if (rapidez < 12) & (rapidez > 5.8):  # Cabecera, cmd, Motor, dir, RpmH, RpmL
    #             check = 1
    #         else:
    #             rapidez_string = input("Introduzca la rapidez (no max):\n")
    #     else:
    #         rapidez_string = input("Error! \nIntroduzca el PWM rueda derecha:\n")

    PWMrd = rpd2pwm("rd", rapidez)
    PWMri = rpd2pwm("ri", rapidez)

    # Variables de control

    # Movimiento en linea recta
    krd = 200*rapidez/8 # Kp rueda derecha
    kri = 300*rapidez/8 # Kp rueda izquierda
    # Constantes Movimiento angular
    kard = 1000 * rapidez / 10
    kari = 1300 * rapidez / 10
    # Indice de la trayectoria
    obj_index = 2  # indice del objetivo (segundo elemento de la trayectoria)
    flag_alineado = 0
    while obj_index < np.size(tr_obj): # Mientras no se alcance el ultimo punto de la trayectoria
        obj = tr_obj[obj_index]
        #Obtener la posicion del robot
        response = Request("127.0.0.1", "8000")
        # Buscar la posicion de los circulos del carrito
        x_cyan, y_cyan, radius_cyan, x_yellow, y_yellow, radius_yellow = circles_robot(response)
        if (x_cyan != None) and (x_yellow != None):
            # Vectors
            robot_pos = np.array(
                [x_yellow + (x_cyan - x_yellow) / 2, y_yellow + (y_cyan - y_yellow) / 2])  # posicion del robot
            tr_robot = np.append(tr_robot, [robot_pos], 0)
            # Vectores referenciados a la posicion del robot
            head_vector = np.array([x_cyan - robot_pos[0], y_cyan - robot_pos[1]])
            #ball_vector = np.array([1, 0])
            ball_vector = np.array([obj[0]-robot_pos[0], obj[1]- robot_pos[1]])
            angle = angle_between(head_vector, ball_vector)
            error = np.abs(angle)
            if error > 20:
                control_w(angle, PWMrd, PWMri, kard, kari, port)
                alineado = 1
                if alineado == 1:
                    flag_alineado = 1
                    print("alineado")
            else:
                control_w(angle, PWMrd, PWMri, krd, kri, port)

            dist_obj = norm_vector(ball_vector)

            if dist_obj< 4:
                print("siguiente punto")
                flag_alineado = 0
                obj_index = obj_index+1
            cv2.imshow("Q interrumpir", objp)
            c = cv2.waitKey(5)
            if c & 0xFF == ord("q"):
                break

    cv2.destroyAllWindows()
    print("El carro se ha alineado")
    time.sleep(1)


    plt.figure(2)
    plt.xlim([0, 100])
    plt.ylim([0, 100])
    plt.scatter(tr_robot[:, 0], tr_robot[:, 1], c='b')
    plt.scatter(tr_obj[:, 0], tr_obj[:, 1], c='g')
    # plt.quiver(robot_pos[0], robot_pos[1], head_vector[0], head_vector[1], color='r')
    # plt.quiver(robot_pos[0], robot_pos[1], obj[0], obj[1], color='g')
    plt.waitforbuttonpress()
    # last_pos = robot_pos
    # vector_rapidez_prom = np.array([])
    # print("Angulo final = {}".format(angle))
    #
    #
    # if iciva_arrive == 1:
    #
    #     rapidez_prom = np.mean(vector_rapidez_prom)
    #     print("Rapidez promedio = {}".format(rapidez_prom))
    #     command = input("Guardar Proceso?\n")
    #     if command == 'y':
    #         PWMrd_array = np.append(PWMrd_array, [PWMrd])
    #         PWMri_array = np.append(PWMri_array, [PWMri])
    #         Rapidez_prom_array = np.append(Rapidez_prom_array, [rapidez_prom])
    #         print("Array rd = {}".format(PWMrd_array))
    #         print("Array ri = {}".format(PWMri_array))
    #         print("Rapidez promedio = {}".format(Rapidez_prom_array))
    #         cv2.destroyAllWindows()
    #
    #


    close_port(port)
if __name__ == '__main__':
        main()

