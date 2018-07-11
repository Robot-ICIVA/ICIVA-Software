from Trayectorias.lib.RejillaLib import *
import pygame as pg
from Trayectorias.lib.MotorLib import align, move_forward, control_w, rpd2pwm, send_PWM
from Trayectorias.lib.SerialLib import open_port, close_port, Micro_reset, Micro_comfirm_ACK
from Trayectorias.lib.ServerLib import *
import matplotlib.pyplot as plt
vec = pg.math.Vector2

Resolucion = 1 # centimetros

# Medidas en metros
Ancho_Real = 1.44 #metros (minimo 30cm )
Largo_Real = 1.44 #metros (maximo 1.5 m)

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

NAME="Rejillas Probabilísticas de Ocupación"





game = Visualize(ancho, largo, Resolucion, TILESIZE, NAME)

#port = open_port()  # puerto bluetooth
#robot_ini, _ = get_robot_pos()  # obtener la posicion del robot
robot_ini = np.array([50, 50])
last_pos = robot_ini
print(robot_ini)

game.new()
#game.Grid.ENEMIGO = vec(robot_ini[0]+10, robot_ini[1])
#game.Grid.centroide = [vec(robot_ini[0], robot_ini[1]+10)]
#game.Grid.P_Malas = [game.Grid.ENEMIGO, vec(robot_ini[0], robot_ini[1]+10)]
#game.Grid.ICIVA = vec(robot_ini[0], robot_ini[1])

#game.set_initial_state()
while game.playing:
    game.Clock.tick(FPS)
    game.events()
    game.update()
    game.drawing()
print(game.TRAYECTORIA)

# Trayectoria obtenida
print("Trayectoria obtenida")
tr_obj = np.zeros([0, 2])  # array para guardar x y y de la trayectoria deseada
tr_obj = np.append(tr_obj, [robot_ini], 0) # Primer punto de la trayectoria, la posicion actual del robot
for centro in self.Grid.centroide:
    tr_obj = np.append(tr_obj, [[centro.x, centro.y]], 0)

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
# plt.quiver(robot_pos[0], robot_pos[1], head_vector[0], head_vector[1], color='r')
# plt.quiver(robot_pos[0], robot_pos[1], obj[0], obj[1], color='g')
plt.waitforbuttonpress()
close_port(port)



pg.quit()

#03749-78188
