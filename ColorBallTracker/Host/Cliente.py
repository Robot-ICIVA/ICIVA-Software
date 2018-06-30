import time
import requests
import json
import matplotlib.pyplot as plt
import numpy as np
from ColorBallTracker.Host.lib.SerialLib import open_port, close_port, Micro_reset, Micro_comfirm_ACK
from ColorBallTracker.Host.lib.MotorLib import align, move_forward
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

    return (x_cyan, y_cyan, radius_cyan, x_yellow, y_yellow, radius_yellow)

def get_balls(response): # busca los circulos del robot en el paquete JSON

    for list in response:
        if 'RED' in list:
            x_ball = list[0][0]*100
            y_ball = list[0][1]*100
            radius_ball = list[1]*100

    return (x_ball, y_ball, radius_ball)

def Request(ip, port_server): # el servo que controla phi (plano xy)
    url_FREERUN = "http://"+ ip +":"+ port_server

    print(url_FREERUN)

    try:
        T_Inicio = time.time()

        req = requests.get(url_FREERUN)
        response = req.json()
        T_Final = time.time()
        Dif = T_Final - T_Inicio
        nro_balls = len(response)
        print(nro_balls)
        # Buscar la posicion de los circulos del carrito
        x_cyan, y_cyan, radius_cyan, x_yellow, y_yellow, radius_yellow = circles_robot(response)

        x_ball, y_ball, radius_ball = get_balls(response)
        plt.figure(1)
        plt.clf()
        p1 = [x_yellow, y_yellow]
        p2 = [x_cyan, y_cyan]

        # Vectors

        robot_pos= np.array([x_yellow+(x_cyan-x_yellow)/2,  y_yellow+(y_cyan-y_yellow)/2]) # posicion del robot

        # Vectores referenciados a la posicion del robot
        head_vector = np.array([x_cyan-robot_pos[0], y_cyan-robot_pos[1]])
        ball_vector = np.array([x_ball - robot_pos[0], y_ball - robot_pos[1]])
        angle = angle_between(head_vector, ball_vector)
        dist_obj = norm_vector(ball_vector)
        # x_matrix = np.array([])
        # y_matrix = np.array([])
        # x_matrix = np.append(x_matrix, [x_cyan])
        # x_matrix = np.append(x_matrix, [x_yellow])
        # y_matrix = np.append(y_matrix, [y_cyan])
        # y_matrix = np.append(y_matrix, [y_yellow])


        # Draw robot vector and trayectory vector
        """
        plt.quiver(robot_pos[0], robot_pos[1], head_vector[0], head_vector[1], color='r' )
        plt.quiver(robot_pos[0], robot_pos[1],  ball_vector[0], ball_vector[1], color='g')
        plt.plot(robot_pos[0], robot_pos[1], marker='+', c='g', label="Odometry")
        plt.plot(x_cyan, y_cyan, marker='o', c='b', label="Odometry")
        plt.plot(x_yellow, y_yellow, marker='o', c='y', label="Odometry")
        plt.plot(x_ball, y_ball, marker='o', c='r', label="Odometry")
        plt.xlim([0,100])   
        plt.ylim([0, 100])
        #plt.plot(x_matrix, y_matrix)
        # plt.axis('equal')
        # plt.title("Frame %d\nCum. err. %.2fm" % (fIdx, t_error))
        # plt.legend()

        plt.draw()
        plt.waitforbuttonpress(0.02)
        """








    except:
        print("Error Sending Data")
        return None, None

    return angle, dist_obj


def main():
    port = open_port() # puerto bluetooth
    Micro_reset(port)
    ACK = Micro_comfirm_ACK(port)
    while (ACK != 1):
        port.reset_input_buffer()
        Micro_reset(port)
        ACK = Micro_comfirm_ACK(port)
        print("El micro no se ha resetiado")
    print("Micro reseteado")

    while True:
        time.sleep(0.05)
        threshold = 15
        angle, dist_obj = Request("127.0.0.1", "8000")
        if angle is not None:
            print(angle)
            alineado = align(angle, threshold, port)
            if alineado == 1:
                if dist_obj is not None:
                    print("Distancia al objetivo = {}".format(dist_obj))
                    move_forward(dist_obj, 20, 3, port)


if __name__ == '__main__':
    main()
