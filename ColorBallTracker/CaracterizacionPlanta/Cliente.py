import time
import requests
import json
import matplotlib.pyplot as plt
import numpy as np
import cv2
from ColorBallTracker.CaracterizacionPlanta.lib.SerialLib import open_port, close_port, Micro_reset, Micro_comfirm_ACK
import sys
import os
from ColorBallTracker.CaracterizacionPlanta.lib.MotorLib import align, move_forward, control_w, send_PWM
#plt.ion() # activar plot interactivo

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

    while True:
        # Obtener la posicion inicial del robot
        nro_balls = 1
        while nro_balls != 2:
            response = Request("127.0.0.1", "8000")
            x_cyan, y_cyan, radius_cyan, x_yellow, y_yellow, radius_yellow = circles_robot(response)
            if x_cyan != None and x_yellow != None:
                # Vectors
                robot_ini = np.array(
                    [x_yellow + (x_cyan - x_yellow) / 2, y_yellow + (y_cyan - y_yellow) / 2])  # posicion del robot
                nro_balls = len(response)


        last_pos = robot_ini
        vector_rapidez_prom = np.array([])

        # Llegar al punto de referencia
        obj_ini = np.array([15, 50])  # Punto de referencia inicial donde comienza el proceso
        iciva_arrive = 0
        while iciva_arrive != 1:
            threshold = 15 # angular
            response = Request("127.0.0.1", "8000")
            nro_balls = len(response)
            print("Numero de circulos = {}".format(nro_balls))
            if nro_balls == 2:
                # Buscar la posicion de los circulos del carrito
                x_cyan, y_cyan, radius_cyan, x_yellow, y_yellow, radius_yellow = circles_robot(response)
                if x_cyan != None and x_yellow != None:
                    # Vectors
                    robot_pos = np.array(
                        [x_yellow + (x_cyan - x_yellow) / 2, y_yellow + (y_cyan - y_yellow) / 2])  # posicion del robot
                    tr_robot = np.append(tr_robot, [robot_pos], 0)
                    # Vectores referenciados a la posicion del robot
                    head_vector = np.array([x_cyan - robot_pos[0], y_cyan - robot_pos[1]])
                    ball_vector = obj_ini-robot_pos
                    #ball_vector = np.array([obj[0]-robot_pos[0], obj[1]- robot_pos[1]])
                    angle = angle_between(head_vector, ball_vector)

                    if angle is not None:
                        alineado = align(angle, threshold, port)
                        if alineado == 1:
                            dist_obj = norm_vector(ball_vector)
                            if dist_obj is not None:
                                print("Distancia al objetivo = {}".format(dist_obj))
                                iciva_arrive = move_forward(dist_obj, 3, 3, port)

        # Alinear horizontalmente para comenzar la calibracion
        ball_vector = np.array([1, 0])
        angle = angle_between(head_vector, ball_vector)
        threshold = 1
        alineado = align(angle, threshold, port)
        while alineado != 1:
            T_Inicio = time.time()
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
                    alineado = align(angle, threshold, port)
            time.sleep(0.05)
            T_Final = time.time()
            Dif = T_Final - T_Inicio
            print("Tiempo = {}".format(Dif))
        print("Angulo de alineacion = {}".format(angle))
        print("El carro se ha alineado")
        time.sleep(4)

        # Comenzar prueba de calibracion

        # Mover hacia adelanta
        Dir2 = 0
        Dir1 = 1
        PWMrd = 30000
        PWMri = 30000
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
            send_PWM(Dir1, PWMri, Dir2, PWMrd, port)
            # Obtener la posicion del robot
            response = Request("127.0.0.1", "8000")
            nro_balls = len(response)
            print("Numero de circulos = {}".format(nro_balls))
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
        while np.abs(angle)< threshold: # mientras se considere derecho
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
                    if robot_pos[0] > 70:
                        iciva_arrive = 1
                        send_PWM(1, 0, 1, 0, port) # Mandar a detener al robot
        if iciva_arrive == 1:
            print("calibrado")
            print("el pwm derecho es: {}" .format(PWMrd))
            print("el pwm izquierdo es: {}".format(PWMri))
            break
        else:
            if angle > 0:
                PWMrd = PWMrd + 3000
            else:
                PWMri = PWMri + 3000

            print("no calibrado")
    T_Inicio = time.time()



    """
        cv2.imshow("Im", objp)
        c = cv2.waitKey(20)
        if c & 0xFF == ord("q"):
            plt.figure(1)
            plt.xlim([0, 100])
            plt.ylim([0, 100])
            plt.scatter(tr_robot[:, 0], tr_robot[:, 1], c='b')
            plt.scatter(tr_obj[:, 0], tr_obj[:, 1], c='g')
           # plt.quiver(robot_pos[0], robot_pos[1], head_vector[0], head_vector[1], color='r')
            #plt.quiver(robot_pos[0], robot_pos[1], obj[0], obj[1], color='g')
            plt.waitforbuttonpress(100)
            break

        T_Final = time.time()
        Dif = T_Final - T_Inicio
        if robot_pos[0] > 20.0:
            dist_recorrida = norm_vector(last_pos - robot_pos)
            rapidez = dist_recorrida/Dif
            vector_rapidez_prom = np.append(vector_rapidez_prom, [rapidez])
            print("rapidez = {}" .format(rapidez))

        last_pos = robot_pos

        if robot_pos[0] > 70.0:
            break
        print("Loop time = {}".format(Dif))
    rapidez_prom = np.mean(vector_rapidez_prom)
    print("Rapidez promedio = {}".format(rapidez_prom))
    
    """
    close_port(port)
if __name__ == '__main__':
        main()
