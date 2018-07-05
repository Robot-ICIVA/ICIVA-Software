import time
import requests
import json
import matplotlib.pyplot as plt
import numpy as np
from lib.SerialLib import

plt.ion() # activar plot interactivo

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
    return np.degrees(np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)))

def circles_robot(response): # busca los circulos del robot en el paquete JSON

    for list in response:
        if 'CYAN' in list:
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
        head_vector = np.array([x_cyan-robot_pos[0] , y_cyan-robot_pos[1]])
        head_vector_u = unit_vector(head_vector)
        print(head_vector_u)
        tray_x =  x_ball-robot_pos[0]
        tray_y = y_ball-robot_pos[1]
        tray_vector = (tray_x, tray_y)
        # x_matrix = np.array([])
        # y_matrix = np.array([])
        # x_matrix = np.append(x_matrix, [x_cyan])
        # x_matrix = np.append(x_matrix, [x_yellow])
        # y_matrix = np.append(y_matrix, [y_cyan])
        # y_matrix = np.append(y_matrix, [y_yellow])


        # Draw robot vector and trayectory vector
        R_head = np.array([[x_cyan-robot_pos[0], y_cyan-robot_pos[1]]])
        B_head = np.array([[x_ball - robot_pos[0], y_ball - robot_pos[1]]])
        #origin = [robot_x], [robot_y]  # origin point
        plt.quiver(robot_pos[0], robot_pos[1], head_vector_u[0], head_vector_u[1], color='r', scale = norm_vector(head_vector) )
        plt.quiver(robot_pos[0], robot_pos[1],  B_head[:, 0], B_head[:, 1], color='g')
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








    except:
        print("Error Sending Data")

    return


def main():
    # port = open_port() # puerto bluetooth
    # Mircro_reset(port)
    # ACK = Micro_comfirm_ACK(port)
    # while (ACK != 1):
    #     port.reset_input_buffer()
    #     Mircro_reset(port)
    #     ACK = Micro_comfirm_ACK(port)
    #     print("El micro no se ha resetiado")
    # print("Micro reseteado")
    ax = plt.axes()
    while True:
        time.sleep(2)
        plt.figure(1)
        x = np.linspace(0, 1, 11)
        y = np.linspace(1, 0, 11)
        u = v = np.zeros((11, 11))
        u[5, 5] = 0.2

        plt.axis('equal')
        plt.quiver(x, y, u, v, scale=1, units='xy')
        #plt.quiver(robot_pos[0], robot_pos[1], B_head[:, 0], B_head[:, 1], color='g')
        #plt.plot(1, 1, marker='+', c='g', label="Odometry")
        #plt.xlim([0, 10])
        #plt.ylim([0, 10])
        # plt.plot(x_matrix, y_matrix)
        # plt.axis('equal')
        # plt.title("Frame %d\nCum. err. %.2fm" % (fIdx, t_error))
        # plt.legend()

        plt.draw()
        plt.waitforbuttonpress(0.02)


if __name__ == '__main__':
    main()
