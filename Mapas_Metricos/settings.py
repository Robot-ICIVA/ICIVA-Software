#Resolucion->Tilesize->ajuste
#1 = 2**0 -> 4  = 2**2  ->16 -------con 10 da 80 cuadros
#2 = 2**1 -> 8  = 2**3  ->4  -------con 10 da 40 cuadros
#4 = 2**2 -> 16 = 2**4  ->2  -------con 10 da 20 cuadros

# Medidas de la cancha
Resolucion = 1 # centimetros
TILESIZE = 4*Resolucion #4 pixeles sera 1cm
Fact_Rej = int(24/Resolucion) #24cm son 3 pelotas 8cm de diametro * 3 pelotas = 24 cm --> 1 cuadro del piso a lo largo o ancho
ajuste = (32/TILESIZE)
C_cuadros = int(8/Resolucion)
#if(Resolucion == 1):
    #ajuste = 8
#elif(Resolucion == 2):
    #ajuste = 4
#elif(Resolucion == 4):
    #ajuste = 2

# Medidas en metros
Ancho_Real = 1.44 #metros (minimo 30cm )
Largo_Real = 1.44 #metros (maximo 1.5 m)

# Medidas en centimetros
Ancho_Real = Ancho_Real*100
Largo_Real = Largo_Real*100

# Cantidad de cuadros
Ancho_C = int(Ancho_Real/(Resolucion*ajuste))
Largo_C = int(Largo_Real/(Resolucion*ajuste))
#Fijado para probar
#Ancho_C=10
#Largo_C=10
ancho, largo = Ancho_C * 32, Largo_C * 32

#a=2
#l=2
#CP = a*l/(8*10**-2)
#print("Cantidad de pelotas:"+str(CP))

FPS=10
NAME="Rejillas Probabilísticas de Ocupación"

WHITE=(255,255,255)
BLACK=(0,0,0)
RED=(255,0,0)
BLUE=(0,0,255)
GREEN=(0,255,0)
GREY=(155,155,155)
MEDGREY=(80,80,80)
DARKGREY=(20,20,20)
YELLOW = (255, 255, 0)
ORANGE = (255, 160, 0)
GREEN_APPLE =(36, 231, 17)
