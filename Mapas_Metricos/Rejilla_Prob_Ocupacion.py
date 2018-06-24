import numpy as np
import pygame as pg
import matplotlib.pyplot as plt
from settings import *
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
        #self.connections = [vec(1, 0), vec(-1, 0), vec(0, 1), vec(0, -1), vec(1, 1), vec(-1, 1), vec(-1, -1),vec(1, -1)]

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
        self.Grid = Grid()
        self.Ball = Ball()
        self.Balls = []
        Coord_X_m = 0.96
        Coord_Y_m = 0.96
        Coord_X_cm = Coord_X_m*100
        Coord_Y_cm = Coord_Y_m*100
        Coord_X = int(Coord_X_cm/(Resolucion))-1
        Coord_Y = -int(Coord_Y_cm/(Resolucion))+1
        self.Grid.aliveNodes = []#[vec(Coord_X, Coord_Y)] # [vec(0, 0), vec(0, 1), vec(0, -1)]
        self.Grid.centroide = []#[vec(Coord_X, Coord_Y),vec(Coord_X+20, Coord_Y+10)]
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
            if event.type == pg.MOUSEBUTTONDOWN:
                mouse_pos = (vec(pg.mouse.get_pos()) - vec(0, largo - 1 * TILESIZE)) // TILESIZE  # getting the tile coordinates of the mouse

                # spawning or destroying a cell
                if (mouse_pos in self.Grid.centroide):# or (mouse_pos in self.Ball.relleno):
                    self.Grid.centroide.remove(mouse_pos)
                    #self.Grid.aliveNodes.remove(mouse_pos)
                    #self.Ball.centro.remove(mouse_pos)
                    #self.Ball.relleno = []
                else:
                    self.Grid.centroide.append(mouse_pos)
                    #self.Ball.centro.append(mouse_pos)

    def update(self):
        self.Grid.update()

    def drawing(self):
        # Dibujo de la Grid
        self.ventana.fill(DARKGREY)
        self._draw_ball_(self.Grid, self.Ball)
        self._draw_nodes(self.Grid, self.ventana)
        self._draw_grid()
        self._draw_grid1()

        # self.draw_grid()
        pg.display.flip()

    def _draw_nodes(self, Grid, screen):
        for node in Grid.aliveNodes:
            #print("_draw_nodes:"+str(node))
            #rect = pg.Rect(((node.x * TILESIZE) + ancho // 2, (node.y * TILESIZE) + largo // 2), (TILESIZE, TILESIZE))
            rect = pg.Rect(((node.x * TILESIZE), (node.y * TILESIZE) + largo-1*TILESIZE), (TILESIZE, TILESIZE))
            pg.draw.rect(screen, GREY, rect)
            #rect1 = pg.Rect(((42 * TILESIZE), (-100 * TILESIZE + largo - 1 * TILESIZE)), (TILESIZE, TILESIZE))
            #pg.draw.rect(screen, RED, rect1)
        #pg.draw.circle(screen, RED, (int(ancho/2), int(largo/2)), int(C_cuadros/2)* TILESIZE, 3)
        for node in Grid.centroide:
            x=int(node.x*TILESIZE)
            y=int(node.y*TILESIZE+ largo-1*TILESIZE)
            #print(str(x)+" "+str(y))
            pg.draw.circle(screen, RED, (x,y), int(C_cuadros/2)*TILESIZE, 16)
            #pg.draw.circle(screen, RED, ((node.x * TILESIZE), (node.y * TILESIZE) + largo-1*TILESIZE), 4*TILESIZE, 1)

    def _draw_ball_(self, Grid, Ball):
        for node in Grid.centroide:
            #print("node: "+str(node))
            for i in range(C_cuadros):#*TILESIZE):
                #print("i: " + str(i))
                for j in range(C_cuadros):#*TILESIZE):
                    #print("j: " + str(j))
                    coord_X = node.x * TILESIZE - int(C_cuadros/2)*TILESIZE + i * TILESIZE
                    coord_Y = node.y * TILESIZE - int(C_cuadros/2)*TILESIZE + j * TILESIZE
                    Grid.aliveNodes.append((vec(coord_X, coord_Y))// TILESIZE)
                    Ball.relleno.append((vec(coord_X, coord_Y))// TILESIZE)

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