from Trayectorias.lib.RejillaLib import *
from Trayectorias.lib.MotorLib import align, move_forward, control_w, rpd2pwm, send_PWM
from Trayectorias.lib.SerialLib import open_port, close_port, Micro_reset, Micro_comfirm_ACK
from Trayectorias.lib.ServerLib import *
import matplotlib.pyplot as plt
import time
import numpy as np
import cv2
import pygame as pg
vec = pg.math.Vector2

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


def get_trayectoria(robot_pos, obstacles, balls, ancho, largo, Resolucion, TILESIZE, NAME):
    game = Visualize(ancho, largo, Resolucion, TILESIZE, NAME)
    game.new()
    for obstacle in obstacles:
        x = int(obstacle[0])
        y = int(obstacle[1])
        game.Grid.P_Malas.append(vec(x, y))
    for ball in balls:
        x = int(ball[0])
        y = int(ball[1])
        game.Grid.centroide.append(vec(x, y))
    game.Grid.ICIVA = vec(int(robot_pos[0]), int(robot_pos[1]))

    print(game.Grid.ICIVA.x)
    print(game.Grid.centroide)
    print(game.Grid.P_Malas)
    FPS = 10
    # game.set_initial_state()
    while game.Bandera != 1:
        game.Clock.tick(FPS)
        game.events()
        game.update()
        game.drawing()
    print(game.TRAYECTORIA)

    return  game.TRAYECTORIA # Tipo numpy


Resolucion = 1 # centimetros

# Medidas en metros
Ancho_Real = 1.40 #metros (minimo 30cm )b
Largo_Real = 1.40 #metros (maximo 1.5 m)

# Medidas en centimetros
Ancho_Real = Ancho_Real*100
Largo_Real = Largo_Real*100

# Medidas de la cancha
TILESIZE = 4*Resolucion #4 pixeles sera 1cm
Fact_Rej = int(24/Resolucion) #24cm son 3 pelotas 8cm de diametro * 3 pelotas = 24 cm --> 1 cuadro del piso a lo largo o ancho
ajuste = (32/TILESIZE)

# Cantidad de cuadros
Ancho_C = int(Ancho_Real/(Resolucion*ajuste))
Largo_C = int(Largo_Real/(Resolucion*ajuste))
#Fijado para probar
#Ancho_C=10
#Largo_C=10
ancho, largo = Ancho_C * 32, Largo_C * 32

#port = open_port()  # puerto bluetooth
robot_ini, _, balls, obstacles = get_all()  # obtener la posicion del robot
last_pos = robot_ini
print("Posicion inicial = {}".format(robot_ini))
NAME = "Rejillas Probabilísticas de Ocupación"
trayectoria = get_trayectoria(robot_ini, obstacles, balls, ancho, largo, Resolucion, TILESIZE, NAME)

# Trayectoria obtenida
print("Trayectoria obtenida")
tr_obj = np.zeros([0, 2])  # array para guardar x y y de la trayectoria deseada
tr_obj = np.append(tr_obj, [robot_ini], 0) # Primer punto de la trayectoria, la posicion actual del robot
i = 0
ball_ini = trayectoria[len(trayectoria)-1]
for centro in trayectoria:
    point = np.array([centro[0], centro[1]])
    ball_vector = np.array([point[0] - ball_ini[0], point[1] - ball_ini[1]])
    dist_obj = norm_vector(ball_vector)
    if dist_obj > 8:
        if i == 12:
            tr_obj = np.append(tr_obj, [point], 0)
            i = 0
        i = i +1
print(tr_obj)
# Trayectoria del robot (inicializacion vacia)
tr_robot = np.zeros([0, 2])  # array para guardar x y y del robot

# Reseteo de micro
Micro_reset(port)
ACK = Micro_comfirm_ACK(port)
while (ACK != 1):
    port.reset_input_buffer()
    Micro_reset(port)
    ACK = Micro_comfirm_ACK(port)
    print("El micro no se ha resetiado")
print("Micro reseteado")

# Grafica trayectoria deseada
plt.figure(1)
plt.xlim([0, 100])
plt.ylim([0, 100])
plt.scatter(tr_obj[:, 0], tr_obj[:, 1], c='g')
plt.scatter(ball_ini[0], ball_ini[1], c=(0, 1, 1))
plt.ylabel("y (cm)")
plt.xlabel("x (cm)")
plt.title("Trayectoria deseada")
plt.waitforbuttonpress(5)
plt.close(1)

# Pedir rapidez con la cual se recorrera la treayectoria
rapidez_string = input("Introduzcala rapidez (cm/s):\n")
rapidez = float(rapidez_string)
vector_rapidez_prom = np.array([])
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
krd = 200 * rapidez / 8  # Kp rueda derecha
kri = 300 * rapidez / 8  # Kp rueda izquierda
# Constantes Movimiento angular
kard = 1500 * rapidez / 9 #1000
kari = 1300 * rapidez / 9 #1300
# Indice de la trayectoria
obj_index = 1  # indice del objetivo (segundo elemento de la trayectoria)
last_obj_index = 0
# Imagen para interrupcion
objp = np.zeros((100, 3), np.uint8)
nro_obj, _ = np.shape(tr_obj)
while obj_index < nro_obj:  # Mientras no se alcance el ultimo punto de la trayectoria
    obj = tr_obj[obj_index]
    # Obtener la posicion del robot
    T_inicio = time.time()
    robot_pos, head_vector = get_robot_pos()
    tr_robot = np.append(tr_robot, [robot_pos], 0)
    # ball_vector = np.array([1, 0])
    ball_vector = np.array([obj[0] - robot_pos[0], obj[1] - robot_pos[1]])
    angle = angle_between(head_vector, ball_vector)
    dist_obj = norm_vector(ball_vector)
    error = np.abs(angle)

    if last_obj_index != obj_index : # Si se obtiene una nuevo objetivo
        print("Angulo 2 = {}".format(angle))
        if error < 10:
            last_obj_index = obj_index
            control_w(angle, PWMrd, PWMri, krd, kri, port)
        if error > 10 and error < 20:
            last_obj_index = obj_index
            control_w(angle, PWMrd, PWMri, kard, kari, port)
        else:
            alineado = align(angle, 5, 200, 200, port)
            while alineado != 1:
                # Obtener la posicion del robot
                print("Alineando con rapidez 0")
                robot_pos, head_vector = get_robot_pos()
                tr_robot = np.append(tr_robot, [robot_pos], 0)
                angle = angle_between(head_vector, ball_vector)
                alineado = align(angle, 5, 300, 300, port)

            last_obj_index = obj_index
    else:
        print("Distancia ={} y angulo = {} ".format(dist_obj, angle))
        if error < 20:
            control_w(angle, PWMrd, PWMri, krd, kri, port)
        else:
            control_w(angle, PWMrd, PWMri, kard, kari, port)
        T_final = time.time()
        T_Dif = T_final - T_inicio
        dist_recorrida = norm_vector(last_pos - robot_pos)
        rapidez = dist_recorrida / T_Dif
        vector_rapidez_prom = np.append(vector_rapidez_prom, [rapidez])
        last_pos = robot_pos
        print("rapidez = {}".format(rapidez))

    if dist_obj < 4.5:
        print("siguiente punto")
        obj_index = obj_index + 1

    cv2.imshow("Q interrumpir", objp)
    c = cv2.waitKey(5)
    if c & 0xFF == ord("q"):
        break


robot_pos, head_vector = get_robot_pos()
ball_vector = np.array([ball_ini[0] - robot_pos[0], ball_ini[1] - robot_pos[1]])
angle = angle_between(head_vector, ball_vector)
alineado = align(angle, 5, 200, 200, port)
while alineado != 1:
    # Obtener la posicion del robot
    print("Alineando con rapidez 0")
    robot_pos, head_vector = get_robot_pos()
    tr_robot = np.append(tr_robot, [robot_pos], 0)
    angle = angle_between(head_vector, ball_vector)
    alineado = align(angle, 5, 300, 300, port)

cv2.destroyAllWindows()
print("El carro se ha alineado")
rapidez_prom = np.mean(vector_rapidez_prom)
print("Rapidez promedio en la trayectoria= {}".format(rapidez_prom))
time.sleep(1)

plt.figure(2)
plt.xlim([0, Ancho_Real])
plt.ylim([0, Largo_Real])
plt.scatter(tr_robot[:, 0], tr_robot[:, 1], c='b')
plt.scatter(tr_obj[:, 0], tr_obj[:, 1], c='r')
plt.scatter(robot_ini[0], robot_ini[1], c='g')
plt.scatter(ball_ini[0], ball_ini[1], c=(0, 1, 1))
# plt.quiver(robot_pos[0], robot_pos[1], head_vector[0], head_vector[1], color='r')
# plt.quiver(robot_pos[0], robot_pos[1], obj[0], obj[1], color='g')
plt.waitforbuttonpress()
close_port(port)



pg.quit()

#03749-78188
