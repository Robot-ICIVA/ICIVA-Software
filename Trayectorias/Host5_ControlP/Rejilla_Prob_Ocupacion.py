import numpy as np
import pygame as pg
import matplotlib.pyplot as plt
from settings import *
import math
import scipy
from scipy.stats import norm
import time
import requests
import json
import cv2
from Trayectorias.lib.SerialLib import open_port, close_port, Micro_reset, Micro_comfirm_ACK
import sys
import os
from Trayectorias.lib.MotorLib import align, move_forward, control_w, rpd2pwm, send_PWM
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
    url = "http://"+ ip +":"+ port_server
    try:

        req = requests.get(url)
        response = req.json()
    except:
        return None

    return response


def get_robot_pos():
    x_cyan = None
    y_cyan = None
    x_yellow = None
    robot_pos = None
    head_vector = None
    while isinstance(x_cyan, type(None)) or isinstance(x_yellow, type(None)):
        response = Request("127.0.0.1", "8000")
        if isinstance(response, type(None)):
            print("Server error...Retrying")
            time.sleep(0.5)
        else:
            x_cyan, y_cyan, radius_cyan, x_yellow, y_yellow, radius_yellow = circles_robot(response)
            if isinstance(x_cyan, type(None)) or isinstance(x_yellow, type(None)):
                print("Error robot pos")
                time.sleep(0.5)
            else:
                # Vectors
                robot_pos = np.array(
                    [x_yellow + (x_cyan - x_yellow) / 2, y_yellow + (y_cyan - y_yellow) / 2])  # posicion del robot
                head_vector = np.array([x_cyan - robot_pos[0], y_cyan - robot_pos[1]])

    return robot_pos, head_vector
class Ball():

    def __init__(self):
        self.diametro = 8
        self.relleno = []
        self.centro = []



class Grid():

    def __init__(self):
        self.aliveNodes = []
        self.centroide = []
        self.P_Malas = []
        I_x = 0
        I_y = 0
        self.ICIVA = None
        E_x = 1
        E_y = 1
        self.ENEMIGO = None #vec(E_x, E_y)
        #self.connections = [vec(1, 0), vec(-1, 0), vec(0, 1), vec(0, -1), vec(1, 1), vec(-1, 1), vec(-1, -1),vec(1, -1)]
        self.trayectoria = []

    def vec2int(self, vec):
        return (int(vec.x), int(vec.y))

    def drawNodes(self):
        pass


    def update(self):
        #self.todie = list(filter(self.checkAlives, self.aliveNodes))
        #bornedNodes = list(filter(self.checkDeaths, self.deathNodes))
        #self.aliveNodes.extend(bornedNodes)
        #self.aliveNodes = list(filter(self.isNotAboutToDie, self.aliveNodes))
        #del self.deathNodes[:]

        # self.aliveNodes.extend(bornedNodes)
        pass

class Visualize():
    def __init__(self):
        pg.init()
        self.ventana = pg.display.set_mode((ancho, largo))
        pg.display.set_caption(NAME)
        self.Clock = pg.time.Clock()

    def new(self):
        # Se presenta el Mapa
        self.playing = True
        self.estado = "neutro"
        self.Grid = Grid()
        self.matrix = np.zeros((int((Largo_C*32)/(TILESIZE)),int((Ancho_C*32)/(TILESIZE))))
        self.Prob = np.zeros((int((Largo_C * 32) / (TILESIZE)), int((Ancho_C * 32) / (TILESIZE))))
        self.MP_Tray = np.zeros((int((Largo_C * 32) / (TILESIZE)), int((Ancho_C * 32) / (TILESIZE))))
        #print(self.matrix)
        #print(np.size(self.matrix,0))
        self.Ball = Ball()
        #self.Balls = []

        Coord_X_m = 0.96
        Coord_Y_m = 0.96
        Coord_X_cm = Coord_X_m*100
        Coord_Y_cm = Coord_Y_m*100
        Coord_X = int(Coord_X_cm/(Resolucion))#-1
        Coord_Y = -int(Coord_Y_cm/(Resolucion))#+1

        self.Grid.aliveNodes = []#[vec(Coord_X, Coord_Y)] # [vec(0, 0), vec(0, 1), vec(0, -1)]
        self.Grid.centroide = []#[vec(Coord_X, Coord_Y),vec(Coord_X+20, Coord_Y+10)]
        self.Grid.P_Malas = []

    def run(self):
        # Lazo de Visualizacion
        # Inicializacion
        port = open_port()  # puerto bluetooth
        robot_ini, _ = get_robot_pos() # obtener la posicion del robot
        last_pos = robot_ini
        self.Grid.ICIVA= vec(robot_ini[0],robot_ini[1])
        while self.playing:
            self.Clock.tick(FPS)
            self.events()
            self.update()
            self.drawing()
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




    def events(self):
        # Eventos
        for event in pg.event.get():
            if event.type == pg.QUIT:
                if self.playing == True:
                    self.playing = False
            if event.type == pg.KEYDOWN:
                if event.key == pg.K_b: #Pelotas buenas
                    self.estado = "P_Buenas"
                elif event.key == pg.K_m: #Pelotas malas
                    self.estado = "P_Malas"
                elif event.key == pg.K_i: #Vehiculo ICIVA
                    self.estado = "ICIVA"
                elif event.key == pg.K_e: #Vehiculo Enemigo
                    self.estado = "ENEMIGO"
                elif event.key == pg.K_q: #Vehiculo Enemigo
                    if self.playing == True:
                        self.playing = False

            if event.type == pg.MOUSEBUTTONDOWN:
                mouse_pos = vec(pg.mouse.get_pos())   # getting the tile coordinates of the mouse
                mouse_pos = vec(mouse_pos[0], -mouse_pos[1]+largo)// TILESIZE
                #mouse_pos = (vec(pg.mouse.get_pos()) - vec(0, largo - 1 * TILESIZE)) // TILESIZE
                # spawning or destroying a cell

                if self.estado == "P_Buenas":
                    if (mouse_pos in self.Grid.centroide):# or (mouse_pos in self.Ball.relleno):
                        self.Grid.centroide.remove(mouse_pos)
                        #self.Grid.aliveNodes.remove(mouse_pos)
                        #self.Ball.centro.remove(mouse_pos)
                        #self.Ball.relleno = []
                    else:
                        self.Grid.centroide.append(mouse_pos)

                    #print(int(mouse_pos.x))
                    #print(int(-mouse_pos.y))q

                    #print(self.matrix[int(-mouse_pos.y)][int(mouse_pos.x)])
                    #self.matrix.itemset((int(-mouse_pos.y), int(mouse_pos.x)), 1)
                    #print(self.matrix[int(-mouse_pos.y)][int(mouse_pos.x)])

                    ##self.Ball.centro.append(mouse_pos)

                elif self.estado == "P_Malas" or self.estado == "ENEMIGO":

                    if (mouse_pos in self.Grid.P_Malas):
                        self.Grid.P_Malas.remove(mouse_pos)
                    else:
                        self.Grid.P_Malas.append(mouse_pos)

                    if self.estado == "ENEMIGO":
                        if (mouse_pos == self.Grid.ENEMIGO):
                            self.Grid.ENEMIGO = None

                        else:
                            if self.Grid.ENEMIGO in self.Grid.P_Malas:
                                self.Grid.P_Malas.remove(self.Grid.ENEMIGO)
                            self.Grid.ENEMIGO = mouse_pos


                elif self.estado == "ICIVA":
                    if mouse_pos == self.Grid.ICIVA:
                        self.Grid.ICIVA = None
                    else:
                        self.Grid.ICIVA = mouse_pos


    def Distance(self, centroide, x, y):
        X = int(centroide.x)
        Y = int(centroide.y)
        Xo = int(x)
        Yo = int(y)
        Distancia = math.sqrt((X-Xo)**2 + (Y-Yo)**2)
        return Distancia

    def Ocupacion(self, Grid):

        Dist = np.zeros((int((Largo_C*32)/(TILESIZE)),int((Ancho_C*32)/(TILESIZE))))

        #Calculo de matriz de posiciones
        yn = np.arange((int((Largo_C * 32) / (TILESIZE))))
        xn = np.arange((int((Ancho_C * 32) / (TILESIZE))))
        matrix_coord = np.meshgrid(xn, yn)
        index_matrix_coord = np.dstack((matrix_coord[0], matrix_coord[1]))

        # Centrar
        sigma = 0

        # Escala
        escala = 1
        alpha = 6 * escala
        pqc = 10 * escala

        for centro in Grid.centroide:
            Dif = index_matrix_coord - np.array((centro.y, centro.x))
            Dist = np.linalg.norm(Dif, axis= 2)
            # M = pqc * scipy.stats.norm(sigma, alpha).pdf(Dist)
            # self.Prob = self.Prob + M
            # ind = self.Prob > 1
            # self.Prob[ind] = 1

        for centro in Grid.P_Malas:
            Dif = index_matrix_coord - np.array((centro.y, centro.x))
            Dist = np.linalg.norm(Dif, axis= 2)
            M = pqc * scipy.stats.norm(sigma, alpha).pdf(Dist)
            self.Prob = self.Prob - M
            ind = self.Prob < -1
            self.Prob[ind] = -1

        # if (len(Grid.centroide) == 0 and len(Grid.P_Malas) == 0):
        #     div = 1
        # else:
        #     # print(type(self.Prob))
        #     max = np.ndarray.max(abs(self.Prob))
        #     # print(max)
        #     div = max
        # self.Prob = self.Prob / div

        #print("Prob 2: ")
        #print(self.Prob)



    def PintarProb(self, screen):
        #Se pintan los cuadritos con probabilidad en la matriz con distinta intensidad

        #""""
        #print("Pintar: ")
        #print(self.Prob)
        self.MP_Tray = self.Prob
        self.Prob = np.flip(self.Prob, 1)

        #L= largo
        for i in range(self.Prob.shape[0]):
            #L = largo-1*TILESIZE
            for j in range(self.Prob.shape[1]):
                rect = pg.Rect(((i * TILESIZE), (-j * TILESIZE)+ largo-1*TILESIZE), (TILESIZE, TILESIZE))
                #print("X = "+str(i)+" Y = "+str(j))
                G = abs(int(self.Prob[i][j]*255))
                #print(G)
                COLOR = (G, G, G)
                #print(COLOR)
                pg.draw.rect(screen, COLOR, rect)
        #self.MP_Tray = self.Prob
        self.Prob = np.zeros((int((Largo_C * 32) / (TILESIZE)), int((Ancho_C * 32) / (TILESIZE))))

        #"""
        #pass

    def Trayectoria(self, Grid, screen):
        valor = 0
        posiciones = []
        pos = self.Grid.ICIVA
        #print(pos)

        if pos != None:
            pos_x = int(pos.x)
            pos_y = int(pos.y)
            #print(pos_x)
            #print(pos_y)
            while (valor < 1):
                # * TILESIZE para pintar en pantalla
                pos_x = int(pos.x)
                pos_y = int(pos.y)
                if pos_x == (self.Prob.shape[0]-1):
                    pos_x = pos_x - 1
                    print(pos_x)
                if pos_y == (self.Prob.shape[1]-1):
                    pos_y = pos_y - 1
                    print(pos_y)
                posiciones = [vec(pos_x + 1, pos_y), vec(pos_x - 1, pos_y), vec(pos_x, pos_y + 1), vec(pos_x, pos_y - 1), vec(pos_x + 1, pos_y + 1), vec(pos_x - 1, pos_y + 1), vec(pos_x - 1, pos_y - 1), vec(pos_x + 1, pos_y - 1)]
                clasificador = [self.MP_Tray[pos_x + 1][pos_y], self.MP_Tray[pos_x - 1][pos_y], self.MP_Tray[pos_x][pos_y + 1], self.MP_Tray[pos_x][pos_y - 1], self.MP_Tray[pos_x + 1][pos_y + 1], self.MP_Tray[pos_x - 1][pos_y + 1], self.MP_Tray[pos_x - 1][pos_y - 1], self.MP_Tray[pos_x + 1][pos_y - 1]]
                pos_aux = np.argmax(clasificador)
                valor = clasificador[pos_aux]
                pos = posiciones[pos_aux]
                Grid.trayectoria.append(pos)
                #print(posiciones)
                #print(clasificador)
                #print(pos_x)
                #print(pos_y)
                #print(self.MP_Tray)
            #print("Pos: "+str(pos))


            #"""
            for node in Grid.trayectoria:
                # Trayectoria
                rect = pg.Rect(((node.x * TILESIZE), (node.y * TILESIZE)), (TILESIZE, TILESIZE))
                pg.draw.rect(screen, ORANGE, rect)
            #"""

    def update(self):
        self.Grid.update()

    def drawing(self):
        # Dibujo de la Grid
        self.ventana.fill(DARKGREY)
        self._draw_ball_(self.Grid, self.Ball, self.matrix)
        self.Ocupacion(self.Grid)
        #self.PintarProb(self.ventana)
        self._draw_nodes(self.Grid, self.ventana)
        self._draw_grid()
        self._draw_grid1()
        image = pg.transform.flip(self.ventana, 0, 1)
        pg.image.save(image,'abc.jpg')
        self.ventana.blit(image, (0,0))

        # self.draw_grid()
        pg.display.flip()

    def _draw_nodes(self, Grid, screen):
        """
        for node in Grid.aliveNodes:
            rect = pg.Rect(((node.x * TILESIZE), (node.y * TILESIZE)), (TILESIZE, TILESIZE))
            pg.draw.rect(screen, GREY, rect)
        """
        for node in Grid.centroide:
            x=int(node.x*TILESIZE+0.5*TILESIZE)
            y = int(node.y * TILESIZE+0.5*TILESIZE)
            pg.draw.circle(screen, RED, (x,y), int(C_cuadros/2)*TILESIZE, 0)
            #print("Centroide: "+str(node))

        for node in Grid.P_Malas:
            x=int(node.x * TILESIZE+0.5*TILESIZE)
            y = int(node.y * TILESIZE+0.5*TILESIZE)
            if node != self.Grid.ENEMIGO:
                pg.draw.circle(screen, BLUE, (x,y), int(C_cuadros/2)*TILESIZE, 0)

        if self.Grid.ICIVA != None:
            # Vehiculo ICIVA
            x = int(self.Grid.ICIVA.x * TILESIZE + 0.5 * TILESIZE)
            y = int(self.Grid.ICIVA.y * TILESIZE + 0.5 * TILESIZE)
            pg.draw.circle(screen, YELLOW, (x, y), int(C_cuadros / 2) * TILESIZE, 0)

        if self.Grid.ENEMIGO != None:
            # Vehiculo ENEMIGO
            rect = pg.Rect(((self.Grid.ENEMIGO.x * TILESIZE), (self.Grid.ENEMIGO.y * TILESIZE)), (TILESIZE, TILESIZE))
            pg.draw.rect(screen, GREEN_APPLE, rect)

    def _draw_ball_(self, Grid, Ball, matrix):
        for node in Grid.centroide:
            #print("node: "+str(node))
            for i in range(C_cuadros):#*TILESIZE):
                #print("i: " + str(i))
                for j in range(C_cuadros):#*TILESIZE):
                    #print("j: " + str(j))
                    coord_X = node.x * TILESIZE - int(C_cuadros/2)*TILESIZE + i * TILESIZE
                    coord_Y = node.y * TILESIZE - int(C_cuadros/2)*TILESIZE + j * TILESIZE
                    #print("Coordenada X: "+str(int(coord_X/TILESIZE)))
                    #print("Coordenada Y: "+str(int(coord_Y/TILESIZE)))
                    #self.matrix.itemset((int(coord_Y/TILESIZE), int(coord_X/TILESIZE)), 0.9)
                    Grid.aliveNodes.append((vec(coord_X, coord_Y))// TILESIZE)
                    Ball.relleno.append((vec(coord_X, coord_Y))// TILESIZE)
        #print(matrix)
                    #print("La coordenada es: "+str(coord_X)+", "+str(coord_Y))
        #print("Grid.aliveNodes: ")
        #print(Grid.aliveNodes())
        #self._draw_nodes_ball(self.Grid, self.ventana)

        # self._draw_nodes(self.Grid,self.ventana)


    def _draw_grid(self):
        for x in range(0, ancho, TILESIZE):
            pg.draw.line(self.ventana, MEDGREY, (x, 0), (x, largo))
        for y in range(0, largo, TILESIZE):
            pg.draw.line(self.ventana, MEDGREY, (0, y), (ancho, y))

    def _draw_grid1(self):
        for x in range(0, ancho, TILESIZE*Fact_Rej):
            pg.draw.line(self.ventana, GREEN, (x, 0), (x, largo))
        for y in range(0, largo, TILESIZE*Fact_Rej):
            pg.draw.line(self.ventana, GREEN, (0, y), (ancho, y))

    def set_initial_state(self):
        waiting = True

        while waiting:
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    pg.quit()

                if event.type == pg.MOUSEBUTTONDOWN:
                    mouse_pos = (vec(pg.mouse.get_pos()) - vec(0, largo-1*TILESIZE)) // TILESIZE  # getting the tile coordinates of the mouse

                    # spawning or destroying a cell
                    if mouse_pos in self.Grid.aliveNodes:
                        self.Grid.aliveNodes.remove(mouse_pos)
                    else:
                        self.Grid.aliveNodes.append(mouse_pos)

                if event.type == pg.KEYDOWN:
                    if event.key == pg.K_a:
                        waiting = False

            self.ventana.fill(DARKGREY)

            self._draw_ball_(self.Grid, self.ventana)
            self._draw_nodes(self.Grid, self.ventana)
            self._draw_grid()
            self._draw_grid1()
            print("Estoy en el inicio")

            #self.Grid.draw_grid(self)

            pg.display.flip()


game = Visualize()
game.new()
#game.set_initial_state()
game.run()

pg.quit()

#03749-78188
