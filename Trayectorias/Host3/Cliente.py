import time
import requests
import json
import matplotlib.pyplot as plt
import numpy as np
import cv2
from Trayectorias.Host3.lib.SerialLib import open_port, close_port, Micro_reset, Micro_comfirm_ACK
import sys
import os
from Trayectorias.Host3.lib.MotorLib import align, move_forward, control_w, rpd2pwm, send_PWM
#plt.ion() # activar plot interactivo


"""
Codigo para verficacion del modelo del carro
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

    PWMrd_array= np.array([])
    PWMri_array = np.array([])
    Rapidez_prom_array = np.array([])
    Rapidez_deseada = np.arange(5.7, 12, 0.5)
    index = 4 # indice de la rapidez deseada
    # Trayectoria del robot
    tr_robot = np.zeros([0, 2]) # array para guardar x y y del robot

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
    PWMrd = rpd2pwm("rd", Rapidez_deseada[index])
    PWMri = rpd2pwm("ri", Rapidez_deseada[index])
    objp = np.zeros((100, 3), np.uint8)
    while True:
        # Obtener la posicion inicial del robot
        iciva_arrive = 0
        while iciva_arrive != 1:
            threshold = 15 # angular
            response = Request("127.0.0.1", "8000")
            nro_balls = len(response)
            #print("Numero de circulos = {}".format(nro_balls))
            if nro_balls == 2:
                # Buscar la posicion de los circulos del carrito
                x_cyan, y_cyan, radius_cyan, x_yellow, y_yellow, radius_yellow = circles_robot(response)
                if (x_cyan != None) and (x_yellow != None):
                    # Vectors
                    robot_pos = np.array(
                        [x_yellow + (x_cyan - x_yellow) / 2, y_yellow + (y_cyan - y_yellow) / 2])  # posicion del robot
                    tr_robot = np.append(tr_robot, [robot_pos], 0)
                    # Vectores referenciados a la posicion del robot
                    head_vector = np.array([x_cyan - robot_pos[0], y_cyan - robot_pos[1]])
                    ball_vector = np.array([1, 0])
                    #ball_vector = np.array([obj[0]-robot_pos[0], obj[1]- robot_pos[1]])
                    angle = angle_between(head_vector, ball_vector)
                    print("El angulo 1 es: {}" .format(angle))
                    print("Posicion x es : {}".format(robot_pos[0]))
                    cv2.imshow("Im", objp)

                    c = cv2.waitKey(200)
                    if c & 0xFF == ord("q"):
                        cv2.destroyAllWindows()
                        iciva_arrive = 1

        print("El carro se ha alineado")
        time.sleep(1)

        # Comenzar prueba de calibracion

        # Mover hacia adelanta
        Dir2 = 0
        Dir1 = 1
        # Pedir PWM rd

        # Mover el robot con los PWM introducidos
        send_PWM(Dir1, PWMri, Dir2, PWMrd, port)

        # Obtener la posicion del robot
        nro_balls = 1
        while nro_balls != 2:
            response = Request("127.0.0.1", "8000")
            x_cyan, y_cyan, radius_cyan, x_yellow, y_yellow, radius_yellow = circles_robot(response)
            if x_cyan != None and x_yellow != None:
                # Vectors
                robot_pos = np.array(
                    [x_yellow + (x_cyan - x_yellow) / 2, y_yellow + (y_cyan - y_yellow) / 2])  # posicion del robot
                nro_balls = len(response)
            # esperar a que pase los 30 cm
        while robot_pos[0]< 30:
            time.sleep(0.03)
            send_PWM(Dir1, PWMri, Dir2, PWMrd, port)
            # Obtener la posicion del robot
            response = Request("127.0.0.1", "8000")
            nro_balls = len(response)
            #print("Numero de circulos = {}".format(nro_balls))
            if nro_balls == 2:
                x_cyan, y_cyan, radius_cyan, x_yellow, y_yellow, radius_yellow = circles_robot(response)
                if x_cyan != None and x_yellow != None:
                    robot_pos = np.array(
                        [x_yellow + (x_cyan - x_yellow) / 2, y_yellow + (y_cyan - y_yellow) / 2])  # posicion del robot


        # Medir si va derecho
        head_vector = np.array([x_cyan - robot_pos[0], y_cyan - robot_pos[1]])
        ball_vector = np.array([1, 0])
        angle = angle_between(head_vector, ball_vector)
        threshold = 10
        iciva_arrive = 0
        last_pos = robot_pos
        vector_rapidez_prom = np.array([])
        while True: # mientras se considere derecho
            T_inicio = time.time()
            time.sleep(0.03)
            send_PWM(Dir1, PWMri, Dir2, PWMrd, port)
            response = Request("127.0.0.1", "8000")
            nro_balls = len(response)
            print("Numero de circulos = {}".format(nro_balls))
            if nro_balls == 2:
                x_cyan, y_cyan, radius_cyan, x_yellow, y_yellow, radius_yellow = circles_robot(response)
                if x_cyan != None and x_yellow != None:
                    robot_pos = np.array(
                        [x_yellow + (x_cyan - x_yellow) / 2, y_yellow + (y_cyan - y_yellow) / 2])
                    head_vector = np.array([x_cyan - robot_pos[0], y_cyan - robot_pos[1]])
                    angle = angle_between(head_vector, ball_vector)
                    print("Angulo de alineacion running = {}".format(angle))
                    cv2.imshow("En el segmento", objp)
                    c = cv2.waitKey(200)
                    if c & 0xFF == ord("q"):
                        cv2.destroyAllWindows()
                        break
                    if robot_pos[0] > 60:
                        iciva_arrive = 1
                        send_PWM(1, 0, 1, 0, port) # Mandar a detener al robot
                        break
            T_final = time.time()
            T_Dif = T_final - T_inicio
            dist_recorrida = norm_vector(last_pos - robot_pos)
            rapidez = dist_recorrida / T_Dif
            vector_rapidez_prom = np.append(vector_rapidez_prom, [rapidez])
            print("rapidez = {}".format(rapidez))

            last_pos = robot_pos
        # Obtener la posicion del robot
        nro_balls = 1
        while nro_balls != 2:
            response = Request("127.0.0.1", "8000")
            x_cyan, y_cyan, radius_cyan, x_yellow, y_yellow, radius_yellow = circles_robot(response)
            if x_cyan != None and x_yellow != None:
                # Vectors
                robot_pos = np.array(
                    [x_yellow + (x_cyan - x_yellow) / 2,
                     y_yellow + (y_cyan - y_yellow) / 2])  # posicion del robot
                angle = angle_between(head_vector, ball_vector)
                nro_balls = len(response)
        print("Angulo final = {}".format(angle))


        if iciva_arrive == 1:

            rapidez_prom = np.mean(vector_rapidez_prom)
            print("Rapidez promedio = {}".format(rapidez_prom))
            print("Rapides deseada = {}".format(Rapidez_deseada[index]))

            command = input("Guardar Proceso?\n")
            if command == 'y':
                PWMrd_array = np.append(PWMrd_array, [PWMrd])
                PWMri_array = np.append(PWMri_array, [PWMri])
                Rapidez_prom_array = np.append(Rapidez_prom_array, [rapidez_prom])
                print("Array rd = {}".format(PWMrd_array))
                print("Array ri = {}".format(PWMri_array))
                print("Rapidez promedio = {}".format(Rapidez_prom_array))
                np.savetxt('PWMrd.out', PWMrd_array, fmt='%1.8e')
                np.savetxt('PWMri.out', PWMri_array, fmt='%1.8e')
                np.savetxt('Rapidez.out', Rapidez_prom_array, fmt='%1.8e')
                cv2.destroyAllWindows()
            index = index+1




    close_port(port)
if __name__ == '__main__':
        main()

