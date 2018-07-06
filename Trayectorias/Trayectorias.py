import time
import requests
import json
import matplotlib.pyplot as plt
import numpy as np


def get_balls(response): # busca los circulos del robot en el paquete JSON

    for list in response:
        if 'RED' in list:
            x_ball = list[0][0]*100
            y_ball = list[0][1]*100
            radius_ball = list[1]*100

    return (x_ball, y_ball, radius_ball)

def draw_circle(xo, yo, radio, resol):

    x_array = np.arange(xo-radio, radio+xo+resol, resol)
    y_array = np.array([])
    for x in x_array:
        y = yo +np.sqrt(radio**2 -(x-xo)**2)
        y_array = np.append(y_array, [y])

    for x in x_array[::-1]:
        y = yo -np.sqrt(radio**2 -(x-xo)**2)
        y_array = np.append(y_array, [y])
    x_array = np.append(x_array, [x_array[::-1]])
    plt.figure(3)
    plt.xlim([0, 100])
    plt.ylim([0, 100])
    plt.scatter(x_array, y_array, c='b')
    plt.waitforbuttonpress()
    return

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
    poly = np.poly1d([0.01, 0, 0])
    x_array = np.arange(0, 100, 5)
    y_array = poly(x_array)
    print("polinomio = {}".format(poly))    # print("Micro reseteado")
    plt.scatter(x_array, y_array )
    plt.plot(1, 1, marker='o', c='b', label="Odometry")
    plt.waitforbuttonpress(-1)
    #draw_circle(50.0, 50.0, 20.0, 1.0)



if __name__ == '__main__':
    main()
