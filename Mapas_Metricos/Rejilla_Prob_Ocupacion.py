import numpy as np
import pygame as pg
import matplotlib.pyplot as plt
from settings import *
import math
import scipy
from scipy.stats import norm
vec = pg.math.Vector2


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
        self.ICIVA = None #vec(I_x, I_y)
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
        self.M_Pint = np.zeros((int((Largo_C * 32) / (TILESIZE)), int((Ancho_C * 32) / (TILESIZE))))
        self.index_matrix_coord = np.zeros((int((Largo_C * 32) / (TILESIZE)), int((Ancho_C * 32) / (TILESIZE))))
        self.TRAYECTORIA = None
        self.Prob_malas = np.zeros((int((Largo_C * 32) / (TILESIZE)), int((Ancho_C * 32) / (TILESIZE))))
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
        while self.playing:
            self.Clock.tick(FPS)
            self.events()
            self.update()
            self.drawing()

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

            if event.type == pg.MOUSEBUTTONDOWN:
                mouse_pos = (vec(pg.mouse.get_pos()) - vec(0, 0)) // TILESIZE  # getting the tile coordinates of the mouse
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
                    #print(int(-mouse_pos.y))

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
        matrix_coord = np.meshgrid(yn, xn)
        self.index_matrix_coord = np.dstack((matrix_coord[0], matrix_coord[1]))

        # Centrar
        sigma = 0

        # Escala
        escala = 1
        alpha = 6 * escala
        pqc = 10 * escala

        for centro in Grid.centroide:
            Dif = self.index_matrix_coord - np.array((centro.y, centro.x))
            Dist = np.linalg.norm(Dif, axis= 2)
            M = pqc * scipy.stats.norm(sigma, alpha).pdf(Dist)
            Y = M * 255
            #Y = Y.astype(int)
            ymax = np.ndarray.max(Y)
            Y = Y / ymax
            #Y = np.flip(Y , 1)
            self.Prob = self.Prob + Y
            #self.Prob = self.Prob + M
            ind = self.Prob > 1
            self.Prob[ind] = 1

        for centro in Grid.P_Malas:
            Dif = self.index_matrix_coord - np.array((centro.y, centro.x))
            Dist = np.linalg.norm(Dif, axis= 2)
            M = pqc * scipy.stats.norm(sigma, alpha).pdf(Dist)
            Y = M * 255
            Y = Y.astype(int)
            ymax = np.ndarray.max(Y)
            Y = Y/ymax
            self.Prob_malas = self.Prob_malas + Y
            #Y = np.flip(Y, 1)
            #self.Prob = self.Prob - M
            self.Prob = self.Prob - Y
            ind = self.Prob < -1
            self.Prob[ind] = -1

        if (len(Grid.centroide) == 0 and len(Grid.P_Malas) == 0):
            div = 1
        else:
            # print(type(self.Prob))
            max = np.ndarray.max(abs(self.Prob))
            # print(max)
            div = max
        self.Prob = self.Prob / div

        #print("Prob 2: ")
        #print(self.Prob)



    def PintarProb(self, screen):
        #Se pintan los cuadritos con probabilidad en la matriz con distinta intensidad
        self.MP_Tray = self.Prob
        self.M_Pint =  self.Prob
        self.M_Pint = np.flip(self.M_Pint , 1)

        #Funcion a pintar

        self.Pintar_Signo(screen)

        #self.Pintar_Degradado(screen)

        self.Prob = np.zeros((int((Largo_C * 32) / (TILESIZE)), int((Ancho_C * 32) / (TILESIZE))))
        self.M_Pint = np.zeros((int((Largo_C * 32) / (TILESIZE)), int((Ancho_C * 32) / (TILESIZE))))
        #"""
        #pass

    def Pintar_Degradado(self, screen):
        for i in range(self.M_Pint.shape[0]):
            #L = largo-1*TILESIZE
            for j in range(self.M_Pint.shape[1]):
                rect = pg.Rect(((i * TILESIZE), (-j * TILESIZE)+ largo-1*TILESIZE), (TILESIZE, TILESIZE))
                #print("X = "+str(i)+" Y = "+str(j))
                G = abs(int(self.M_Pint[i][j]*255))
                #print(G)
                COLOR = (G, G, G)
                #print(COLOR)
                pg.draw.rect(screen, COLOR, rect)

    def Pintar_Signo(self, screen):
        for i in range(self.M_Pint.shape[0]):
            # L = largo-1*TILESIZE
            for j in range(self.M_Pint.shape[1]):
                rect = pg.Rect(((i * TILESIZE), (-j * TILESIZE) + largo - 1 * TILESIZE), (TILESIZE, TILESIZE))
                # print("X = "+str(i)+" Y = "+str(j))
                p = self.M_Pint[i][j]
                #G=255
                G = abs(int(p*255))
                CN = (abs(int(p * 36)), abs(int(p * 231)), abs(int(p * 17)))
                if p > 0:
                    COLOR = (G, G, 0)
                elif p == 0:
                    COLOR = (0, 0, 0)
                elif p < 0:
                    COLOR = CN
                    #COLOR = GREEN_SHINE
                # print(COLOR)
                pg.draw.rect(screen, COLOR, rect)

    def Trayectoria(self, Grid, screen):
        valor = 0
        posiciones = []
        Magnitud = []
        pos = self.Grid.ICIVA

        trayectoria = np.zeros((0, 2), dtype=np.float)
        Nodos_Inter = np.zeros((0, 2), dtype=np.float)
        Grid.trayectoria = []
        if pos != None:
            pos_x = int(pos.x)
            pos_y = int(pos.y)
            #print(pos_x)
            #print(pos_y)
            pos_ant = None
            while (valor < 1):
                # * TILESIZE para pintar en pantalla

                #pos_ant = pos - pos_ant
                #pos_norm = np.linalg.norm(pos_ant, axis=2)

                pos_x = int(pos.x)
                pos_y = int(pos.y)

                if valor > 0:
                    self.MP_Tray[pos_x][pos_y] = 0

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
                v = np.asarray(pos)
                trayectoria = np.vstack([trayectoria, v])

                #print(pos)
                #print(posiciones)
                #print(clasificador)
                #print(pos_x)
                #print(pos_y)
                #print(self.MP_Tray)
            #print("Pos: "+str(pos))

            #Correccion = self.Prob != Grid.trayectoria
            #Vecinos = []

            #"""
            Vecinos = np.empty((trayectoria.shape[0],))

            #print(Vecinos.shape)
            #print(len(Grid.trayectoria))

            for idx, node in enumerate(Grid.trayectoria):
                vecinos = 0
                pos_x = int(node.x)
                pos_y = int(node.y)

                posiciones = [vec(pos_x + 1, pos_y), vec(pos_x - 1, pos_y), vec(pos_x, pos_y + 1),
                              vec(pos_x, pos_y - 1), vec(pos_x + 1, pos_y + 1), vec(pos_x - 1, pos_y + 1),
                              vec(pos_x - 1, pos_y - 1), vec(pos_x + 1, pos_y - 1)]
                for pos in posiciones:
                    if pos in Grid.trayectoria:
                        vecinos += 1

                Vecinos[idx] = vecinos

            Nodos_Inter = trayectoria[Vecinos == 2]
            c = Nodos_Inter[1] - Nodos_Inter[0]
            d = np.linalg.norm(c, )

            if d > np.sqrt(2):
                p = [int(self.Grid.ICIVA.x), int(self.Grid.ICIVA.y)]
                Nodos_Inter = np.insert(Nodos_Inter, 0, p, 0)



            #hueco_pos =[]
            pto_ini = []
            pto_fin = []
            p_i = []
            p_f = []
            ind_ = []
            for i in range(len(Nodos_Inter)):
                if i != 0:
                    c = Nodos_Inter[i]-Nodos_Inter[i-1]
                    #print("c es: "+str(c))
                    d = np.linalg.norm(c,)

                    if d > np.sqrt(2):
                        pto_ini.append(Nodos_Inter[i-1])
                        pto_fin.append(Nodos_Inter[i])
                        ind_.append(i)
                        #print(pto_ini)
                        #print(pto_fin)

                        #print("coord ini: "+str(p_i))
                        #print("coord fin: "+str(p_f))

                        #print("d es: " + str(d))

                else: d = 0

            if len(pto_ini) >= 1:

                l = len(pto_fin)
                p_i = pto_ini[0]
                #print(len(pto_ini))
                p_f = pto_fin[l - 1]
                #print(pto_ini)
                #print(pto_fin)
                #print(p_i)
                #print(p_f)
                #pto_ini = []
                #pto_fin = []

            #Calculo de linea recta

                # Centrar
                sigma = 0

                # Escala
                escala = 1
                alpha = 6 * escala
                pqc = 10 * escala

                Dif = self.index_matrix_coord - np.array((int(p_f[1]), int(p_f[0])))
                #p_f = []
                Dist = np.linalg.norm(Dif, axis=2)
                M = pqc * scipy.stats.norm(sigma, alpha).pdf(Dist)
                Y = M * 255
                # Y = Y.astype(int)
                ymax = np.ndarray.max(Y)
                Y = Y / ymax
                # Y = np.flip(Y , 1)
                self.Prob = self.Prob + Y
                # self.Prob = self.Prob + M
                ind = self.Prob > 1
                self.Prob[ind] = 1

                self.Prob = self.Prob - self.Prob_malas
                ind = self.Prob < -1
                self.Prob[ind] = -1

                ###---------------
                self.MP_Tray = self.Prob

                #self.M_Pint = self.Prob
                #self.M_Pint = np.flip(self.M_Pint, 1)
                #self.Pintar_Signo(screen)

                valor = 0
                posiciones = []
                Magnitud = []
                #pos = p_f

                trayectoria = np.zeros((0, 2), dtype=np.float)
                #Nodos_Inter = np.zeros((0, 2), dtype=np.float)
                #Grid.trayectoria = []
                if pos != None:
                    pos_x = int(p_i[0])
                    pos_y = int(p_i[1])
                    #p_i = []
                    # print(pos_x)
                    # print(pos_y)
                    pos_ant = None
                    while (valor < 1):
                        # * TILESIZE para pintar en pantalla

                        # pos_ant = pos - pos_ant
                        # pos_norm = np.linalg.norm(pos_ant, axis=2)

                        if valor > 0:
                            self.MP_Tray[pos_x][pos_y] = 0

                        if pos_x == (self.Prob.shape[0] - 1):
                            pos_x = pos_x - 1
                            print(pos_x)

                        if pos_y == (self.Prob.shape[1] - 1):
                            pos_y = pos_y - 1
                            print(pos_y)
                        posiciones = [vec(pos_x + 1, pos_y), vec(pos_x - 1, pos_y), vec(pos_x, pos_y + 1),
                                      vec(pos_x, pos_y - 1), vec(pos_x + 1, pos_y + 1), vec(pos_x - 1, pos_y + 1),
                                      vec(pos_x - 1, pos_y - 1), vec(pos_x + 1, pos_y - 1)]
                        clasificador = [self.MP_Tray[pos_x + 1][pos_y], self.MP_Tray[pos_x - 1][pos_y],
                                        self.MP_Tray[pos_x][pos_y + 1], self.MP_Tray[pos_x][pos_y - 1],
                                        self.MP_Tray[pos_x + 1][pos_y + 1], self.MP_Tray[pos_x - 1][pos_y + 1],
                                        self.MP_Tray[pos_x - 1][pos_y - 1], self.MP_Tray[pos_x + 1][pos_y - 1]]
                        pos_aux = np.argmax(clasificador)
                        valor = clasificador[pos_aux]
                        pos = posiciones[pos_aux]
                        #Grid.trayectoria.append(pos)
                        v = np.asarray(pos)

                        pos_x = int(pos.x)
                        pos_y = int(pos.y)

                        trayectoria = np.vstack([trayectoria, v])

            ###---------------


            self.Prob = np.zeros((int((Largo_C * 32) / (TILESIZE)), int((Ancho_C * 32) / (TILESIZE))))

            #print("Vecinos: ")
            #print(Vecinos)
            #print("Nodos:")
            #print(Nodos_Inter)


            #"""

            #Nodos_Inter
            #trayectoria
            #i-1
            #print("PRIMERO")
            #print(Nodos_Inter)
            #print(type(Nodos_Inter))
            #print("Nodos: ")
            #print(p_i)
            #print(p_f)
            #print("-----")
            #print(trayectoria)
            #print(type(trayectoria))
            #print("ind_: ")
            #print(ind_)
            #print(len(ind_))

            #"""
            self.TRAYECTORIA = Nodos_Inter
            if len(ind_) >= 1:
                agr = len(trayectoria) - 1
                i = int(ind_[0])
                f = int(ind_[len(ind_) - 1])
                # print(ind_)
                Nodos_Inter = np.delete(Nodos_Inter, np.s_[i:f], 0)
                trayectoria = np.delete(trayectoria, agr, 0)

                self.TRAYECTORIA = np.insert(Nodos_Inter, i, trayectoria, 0)
            #"""
            #print("SEGUNDO")
            #print(Nodos_Inter)
            #print("trayectoria borrada:")
            #print(trayectoria)
            #print(type(Nodos_Inter))
            #print(trayectoria)
            #print(type(trayectoria))
            for node in Grid.trayectoria:
                # Trayectoria
                rect = pg.Rect(((node.x * TILESIZE), (node.y * TILESIZE)), (TILESIZE, TILESIZE))
                pg.draw.rect(screen, ORANGE, rect)

            for node in Nodos_Inter:
                # Trayectoria

                node_x = int(node[0])
                node_y = int(node[1])
                rect = pg.Rect(((node_x * TILESIZE), (node_y * TILESIZE)), (TILESIZE, TILESIZE))
                pg.draw.rect(screen, GREEN, rect)

            if len(ind_) >= 1:
                for node in trayectoria:
                    # Trayectoria
                    #print(node)
                    node_x = int(node[0])
                    node_y = int(node[1])
                    rect = pg.Rect(((node_x * TILESIZE), (node_y * TILESIZE)), (TILESIZE, TILESIZE))
                    pg.draw.rect(screen, BLUE, rect)

                for node in self.TRAYECTORIA:
                    # Trayectoria
                    #print(node)
                    node_x = int(node[0])
                    node_y = int(node[1])
                    rect = pg.Rect(((node_x * TILESIZE), (node_y * TILESIZE)), (TILESIZE, TILESIZE))
                    pg.draw.rect(screen, WHITE, rect)
            #"""

    def update(self):
        self.Grid.update()

    def drawing(self):
        # Dibujo de la Grid
        self.ventana.fill(DARKGREY)
        self._draw_ball_(self.Grid, self.Ball, self.matrix)
        self.Ocupacion(self.Grid)
        self.PintarProb(self.ventana)
        #self.Pintar_Signo(self.ventana)
        self._draw_nodes(self.Grid, self.ventana)
        self._draw_grid()
        self._draw_grid1()
        if len(self.Grid.centroide) >= 1:
            self.Trayectoria(self.Grid, self.ventana)


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
                pg.draw.circle(screen, GREEN_TREE, (x,y), int(C_cuadros/2)*TILESIZE, 0)

        if self.Grid.ICIVA != None:
            # Vehiculo ICIVA
            rect = pg.Rect(((self.Grid.ICIVA.x * TILESIZE), (self.Grid.ICIVA.y * TILESIZE)), (TILESIZE, TILESIZE))
            pg.draw.rect(screen, YELLOW, rect)

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
